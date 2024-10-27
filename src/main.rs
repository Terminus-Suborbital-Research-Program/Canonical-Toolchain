// Specifies that the standard library is not used
#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use panic_halt as _;

mod dynamics;

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
    // Core Functions
    use core::fmt::Write;
    use dynamics::state::dynamics::State;
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use bno055::mint;
    use hal::{
        gpio::{
            self, bank0::{Gpio25, Gpio17, Gpio16}, FunctionSio, PullNone, SioOutput
        }, 
        sio::Sio,
    };
    use fugit::RateExtU32;
    use rp235x_hal::{
        clocks::init_clocks_and_plls, i2c, pac::{accessctrl::watchdog, I2C1}, uart::{
            DataBits, StopBits, UartConfig, UartPeripheral, ValidUartPinout, 
        }, Clock, Watchdog, I2C
    };
    const XTAL_FREQ_HZ: u32 = 12_000_000u32;

    type UARTBus = UartPeripheral<rp235x_hal::uart::Enabled, rp235x_hal::pac::UART0, (gpio::Pin<Gpio16, gpio::FunctionUart, gpio::PullDown>, gpio::Pin<Gpio17, gpio::FunctionUart, gpio::PullDown>)>;
    type I2C1Bus = I2C<I2C1, (gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionI2c, gpio::PullUp>, gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionI2c, gpio::PullUp>)>;
    type BNO055_Device = bno055::Bno055<I2C<I2C1, (gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionI2c, gpio::PullUp>, gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionI2c, gpio::PullUp>)>>;

    #[shared]
    struct Shared {
        uart0: UARTBus,
        dynamics: State,
        bno: BNO055_Device
    }

    #[local]
    struct Local {
        led: gpio::Pin<Gpio25, FunctionSio<SioOutput>, PullNone>
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        // Reset the spinlocks - this is scipped by soft-reset
        unsafe {
            hal::sio::spinlock_reset();
        }

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

        heartbeat::spawn().ok();

        let uart_pins =  (pins.gpio16.into_function(), pins.gpio17.into_function());
        let uart_peripheral = UartPeripheral::new(ctx.device.UART0, uart_pins, &mut ctx.device.RESETS).enable(
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

        let mut timer = hal::Timer::new_timer1(ctx.device.TIMER1, &mut ctx.device.RESETS, &clocks);
        let mut imu: bno055::Bno055<I2C<I2C1, (gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionI2c, gpio::PullUp>, gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionI2c, gpio::PullUp>)>> = bno055::Bno055::new(i2c1).with_alternative_address();
        imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut timer).expect("Could not set IMU Mode...");
        imu.set_power_mode(bno055::BNO055PowerMode::NORMAL);
        let calib = imu.calibration_profile(&mut timer).unwrap();
        imu.set_calibration_profile(calib, &mut timer).unwrap();

        let dynamics = State::default();


        controls::spawn().ok();
        telemetry::spawn().ok();
        imu_sample::spawn().ok();
        (
            Shared {
                uart0: uart_peripheral,
                dynamics: dynamics,
                bno: imu
            }, 
            Local {
                led: led_pin
            }
        )
    }

    // Heartbeats the main led
    #[task(local = [led], priority = 1)]
    async fn heartbeat(ctx: heartbeat::Context) {
        loop {
            _ = ctx.local.led.toggle();

            Mono::delay(500.millis()).await;
        }
    }

    // Controls
    #[task(priority = 2)]
    async fn controls(ctx: controls::Context) {
        loop {
            Mono::delay(500.millis()).await;
        }
    }

    #[task(shared=[uart0, dynamics, bno], priority = 1)]
    async fn imu_sample(mut ctx: imu_sample::Context) {
        loop{
            ctx.shared.bno.lock(|imu|{
                    let ab = imu.accel_data().expect("Could not, get accelerometer data.");
                    ctx.shared.dynamics.lock(|dynamics|{
                            dynamics.ab[0] = ab.x as f64;
                            dynamics.ab[1] = ab.y as f64;
                            dynamics.ab[2] = ab.z as f64;
                        }
                    );
                }
            );
        }
    }

    // Telemetry
    #[task(shared=[uart0, dynamics], priority = 2)]
    async fn telemetry(mut ctx: telemetry::Context) {
        let mut dynamics_local = dynamics::state::dynamics::State::default();
        loop {

            ctx.shared.dynamics.lock(|dynamics|{
                    dynamics_local = dynamics.clone();
                }
            );
            ctx.shared.uart0.lock(|uart0|{
                    let string: heapless::String<256> = serde_json_core::to_string(&dynamics_local).unwrap();
                    uart0.write_fmt(format_args!("{}\n", string.as_str())).expect("Could not write telemetry.");
                }
            );
            Mono::delay(1000.millis()).await;
        }
    }

}