// Specifies that the standard library is not used
#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use panic_halt as _;

// Our Custom Software!
mod dynamics;
mod navigation;
mod utils;

#[cfg(all(feature = "rp2040"))]
compile_error!("RP2040 Support Deprecated");

// HAL Access
#[cfg(feature = "rp2350")]
use rp235x_hal as hal;

// Monotonics
#[cfg(feature = "rp2350")]
use rtic_monotonics::rp235x::prelude::*;
#[cfg(feature = "rp2350")]
rp235x_timer_monotonic!(Mono);

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
#[cfg(feature = "rp2350")]
pub static IMAGE_DEF: rp235x_hal::block::ImageDef = rp235x_hal::block::ImageDef::secure_exe();

#[rtic::app(
    device = hal::pac,
    dispatchers = [PIO2_IRQ_0, PIO2_IRQ_1],
)]

mod app {

    use super::*;
    use dynamics::state::State;
    use navigation::reaction_wheel::ReactionWheel;

    // Core Functions
    use core::fmt::Write;
    use embedded_hal::{delay::DelayNs, digital::{OutputPin, StatefulOutputPin}, pwm::SetDutyCycle};
    use bno055::mint;
    use hal::{
        gpio::{
            self, bank0::{Gpio25, Gpio17, Gpio16}, FunctionSio, PullNone, SioOutput
        }, 
        sio::Sio,
    };
    use fugit::RateExtU32;
    use rp235x_hal::{
        clocks::{init_clocks_and_plls, ClocksManager}, i2c, pac::{accessctrl::watchdog, I2C1, PWM}, timer, uart::{
            DataBits, StopBits, UartConfig, UartPeripheral, ValidUartPinout, 
        }, Clock, Watchdog, I2C
    };
    use heapless::String;
    const XTAL_FREQ_HZ: u32 = 12_000_000u32;

    type UARTBus = UartPeripheral<rp235x_hal::uart::Enabled, rp235x_hal::pac::UART0, (gpio::Pin<Gpio16, gpio::FunctionUart, gpio::PullDown>, gpio::Pin<Gpio17, gpio::FunctionUart, gpio::PullDown>)>;
    type I2C1Bus = I2C<I2C1, (gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionI2c, gpio::PullUp>, gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionI2c, gpio::PullUp>)>;
    type BNO055_Device = bno055::Bno055<I2C<I2C1, (gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionI2c, gpio::PullUp>, gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionI2c, gpio::PullUp>)>>;

    #[shared]
    struct Shared {
        uart0: UARTBus,
        uart0_buffer: heapless::String<2056>,
        dynamics: State,
        dynamics_last: State,
        bno: BNO055_Device,
        timer1: rp235x_hal::Timer<rp235x_hal::timer::CopyableTimer1>,
        // reaction_wheels: ReactionWheel,
    }

    #[local]
    struct Local {
        led: gpio::Pin<Gpio25, FunctionSio<SioOutput>, PullNone>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        // Reset the spinlocks - this is scipped by soft-reset
        unsafe {
            hal::sio::spinlock_reset();
        }

        let mut timer_instant = Mono::now();

        // Set up clocks
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XTAL_FREQ_HZ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        ).ok().unwrap();

        Mono::start(ctx.device.TIMER0, &ctx.device.RESETS);


        // The single-cycle I/O block controls our GPIO pins
        let sio = Sio::new(ctx.device.SIO);

        // Set the pins to their default state
        let pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );


        // Configure GPIO25 as an output
        let mut led_pin = pins.gpio25.into_pull_type::<PullNone>().into_push_pull_output();
        led_pin.set_low().unwrap();

        let uart_pins =  (pins.gpio16.into_function(), pins.gpio17.into_function());
        let mut uart_peripheral = UartPeripheral::new(ctx.device.UART0, uart_pins, &mut ctx.device.RESETS).enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq()
        ).unwrap();

        let sda_pin = pins.gpio14.reconfigure();
        let scl_pin = pins.gpio15.reconfigure();
        let i2c1: I2C<I2C1, (gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionI2c, gpio::PullUp>, gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionI2c, gpio::PullUp>)> = I2C::i2c1(
            ctx.device.I2C1, 
            sda_pin, 
            scl_pin, 
            400.kHz(), 
            &mut ctx.device.RESETS, 
            &clocks.system_clock
        );
        let mut imu: bno055::Bno055<I2C<I2C1, (gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionI2c, gpio::PullUp>, gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionI2c, gpio::PullUp>)>> = bno055::Bno055::new(i2c1).with_alternative_address();
        let mut timer1: rp235x_hal::Timer<rp235x_hal::timer::CopyableTimer1> = hal::Timer::new_timer1(ctx.device.TIMER1, &mut ctx.device.RESETS, &clocks);
        uart_peripheral.write_full_blocking(b"Terminus Flight Software\n");

        imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut timer1).expect("Could not set IMU Mode...");
        imu.set_power_mode(bno055::BNO055PowerMode::NORMAL);
        let mut status = imu.get_calibration_status().unwrap();
        while(!imu.is_fully_calibrated().unwrap()){
            status = imu.get_calibration_status().unwrap();
            uart_peripheral.write_fmt(format_args!("{:?}\n", status));
            timer1.delay_ms(1000);
        }
        uart_peripheral.write_full_blocking(b"Calibrated!");
        let uart0_buffer: String<2056> = heapless::String::new();
        let calib = imu.calibration_profile(&mut timer1).unwrap();
        imu.set_calibration_profile(calib, &mut timer1).unwrap();                 
        

        let reaction_wheel = ReactionWheel::initialize(650.0, 13.7, 0.144);
        let pwm = reaction_wheel.get_pwm(10.0);

        let mut pwm_slices = hal::pwm::Slices::new(ctx.device.PWM, &mut ctx.device.RESETS);
        let mut pwm1 = &mut pwm_slices.pwm1;
        pwm1.set_ph_correct();
        pwm1.enable();
        pwm1.channel_a.output_to(pins.gpio2);
        pwm1.channel_b.output_to(pins.gpio3);
        let mut pwm2 = &mut pwm_slices.pwm2;
        pwm2.enable();
        pwm2.channel_a.output_to(pins.gpio4);

        let pwm_channel = &pwm2.channel_a;

        pwm1.channel_a.set_duty_cycle(0);
        pwm1.channel_b.set_duty_cycle(0);
        pwm2.channel_a.set_duty_cycle(0);
        
        let dynamics_state = State::initialize();
        let dynamics_state_last = State::initialize();



        heartbeat::spawn().ok();
        telemetry::spawn().ok();
        ngc::spawn().ok();
        (
            Shared {
                uart0: uart_peripheral,
                uart0_buffer: uart0_buffer,
                dynamics: dynamics_state,
                dynamics_last: dynamics_state_last,
                bno: imu,
                timer1: timer1,
                // instant: timer_instant,
                // reaction_wheels: reaction_wheel.clone(),
            }, 
            Local {
                led: led_pin,
            }
        )
    }

    // Heartbeats the main led
    #[task(local = [led], priority = 2)]
    async fn heartbeat(ctx: heartbeat::Context) {
        loop {
            _ = ctx.local.led.toggle();

            Mono::delay(500.millis()).await;
        }
    }

    #[task(shared=[uart0, bno, dynamics, dynamics_last], priority = 1)]
    async fn ngc(mut ctx: ngc::Context) {
        loop{
            ctx.shared.bno.lock(|imu|{
                ctx.shared.uart0.lock(|uart0|{
                    // uart0.write_full_blocking(b"Here...");
                });
                    let ab = imu.accel_data().expect("Could not, get accelerometer data.");
                    let wb = imu.gyro_data().expect("Could not, get gyroscopic data.");
                    let m = imu.mag_data().expect("Could not, get magnetometer data.");
                    let quat: mint::Quaternion<f32> = imu.quaternion().expect("Could not get quaternion data.");

                    ctx.shared.dynamics.lock(|dynamics|{
                            ctx.shared.uart0.lock(|uart0|{
                                // uart0.write_full_blocking(b"Here...");
                            });
                            ctx.shared.dynamics_last.lock( |dynamics_last|{
                                dynamics_last.clone_from(dynamics);
                            });
                            ctx.shared.uart0.lock(|uart0|{
                                // uart0.write_full_blocking(b"Here...");
                            });
                            // dynamics.t = ctx.;
                            dynamics.ab[0] = ab.x;
                            dynamics.ab[1] = ab.y;
                            dynamics.ab[2] = ab.z;
                            dynamics.wb[0] = wb.x;
                            dynamics.wb[1] = wb.y;
                            dynamics.wb[2] = wb.z;
                            dynamics.m[0] = m.x;
                            dynamics.m[1] = m.y;
                            dynamics.m[2] = m.z;
                            dynamics.qb[0] = quat.v.x;
                            dynamics.qb[1] = quat.v.y;
                            dynamics.qb[2] = quat.v.z;
                            dynamics.qb[3] = quat.s;

                            // Integrate States
                            ctx.shared.uart0.lock(|uart0|{
                                // uart0.write_full_blocking(b"Here...");
                            });
                        }
                    );
                }
            );
            Mono::delay(10.millis()).await; 
        }
    }

    // Telemetry
    #[task(shared=[uart0, uart0_buffer, dynamics], priority = 2)]
    async fn telemetry(mut ctx: telemetry::Context) {
        let mut dynamics_local = dynamics::state::State::initialize();
        loop {
            ctx.shared.dynamics.lock(|dynamics|{
                    dynamics_local = dynamics.clone();
                }
            );
            ctx.shared.uart0.lock(|uart0|{
                // uart0.write_full_blocking(b"Telemetry.");
                            ctx.shared.uart0_buffer.lock(|uart_buffer|{
                            *uart_buffer = serde_json_core::to_string(&dynamics_local).unwrap();
                            uart0.write_fmt(format_args!("{}\n", uart_buffer)).expect("Could not write telemetry.");
                        }
                    );
                // uart0.write_full_blocking(b"Telemetry.");
                }                
            );
            Mono::delay(20.millis()).await;
        }
    }

}