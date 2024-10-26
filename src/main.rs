// Specifies that the standard library is not used
#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use panic_halt as _;

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
    use defmt::println;
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use hal::{
        gpio::{
            self, bank0::{Gpio25, Gpio17, Gpio16}, FunctionSio, PullNone, SioOutput
        }, 
        sio::Sio,
    };
    use fugit::RateExtU32;
    use rp235x_hal::{
        clocks::init_clocks_and_plls, 
        uart::{
            DataBits, State, StopBits, UartConfig, UartPeripheral, ValidUartPinout, 
        },
        Clock, 
        Watchdog
    };
    const XTAL_FREQ_HZ: u32 = 12_000_000u32;

    type UARTBus = UartPeripheral<rp235x_hal::uart::Enabled, rp235x_hal::pac::UART0, (gpio::Pin<Gpio16, gpio::FunctionUart, gpio::PullDown>, gpio::Pin<Gpio17, gpio::FunctionUart, gpio::PullDown>)>;

    #[shared]
    struct Shared {
        uart0: UARTBus
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
        uart_peripheral.write_full_blocking(b"UART Started.\n");

        controls::spawn().ok();
        telemetry::spawn().ok();

        (
            Shared {
                uart0: uart_peripheral
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

    // Telemetry
    #[task(shared=[uart0], priority = 2)]
    async fn telemetry(mut ctx: telemetry::Context) {
        loop {
            ctx.shared.uart0.lock(|uart0|{
                    uart0.write_full_blocking(b"RTIC Yeet\n");
                }
            );
            Mono::delay(100.millis()).await;
        }
    }
}