// Specifies that the standard library is not used
#![no_std]
#![no_main]

use panic_halt as _;

#[cfg(all(feature = "rp2350", feature = "rp2040"))]
compile_error!("Too many features!");

// HAL Access
#[cfg(feature = "rp2350")]
use rp235x_hal as hal;
#[cfg(feature = "rp2040")]
use rp2040_hal as hal;

// Monotonics
#[cfg(feature = "rp2350")]
use rtic_monotonics::rp235x::prelude::*;
#[cfg(feature = "rp2350")]
rp235x_timer_monotonic!(Mono);
#[cfg(feature = "rp2040")]
use rtic_monotonics::rp2040::prelude::*;
#[cfg(feature = "rp2040")]
rp2040_timer_monotonic!(Mono);

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
#[cfg(feature = "rp2350")]
pub static IMAGE_DEF: rp235x_hal::block::ImageDef = rp235x_hal::block::ImageDef::secure_exe();

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
#[cfg(feature = "rp2040")]
pub static IMAGE_DEF: rp2040_hal::block::ImageDef = rp2040::block::ImageDef::secure_exe();

#[rtic::app(
    device = hal::pac
)]
mod app {
    use super::*;
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use hal::{
        clocks, gpio::{
            self, bank0::Gpio25, FunctionSio, PullDown, PullNone, SioOutput
        }, pac, sio::Sio, timer::Timer, watchdog::Watchdog
    };

    use fugit::RateExtU32;

    const XTAL_FREQ_HZ: u32 = 12_000_000u32;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: gpio::Pin<Gpio25, FunctionSio<SioOutput>, PullNone>
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);

        #[cfg(feature = "rp2350")]
        Mono::start(ctx.device.TIMER0, &ctx.device.RESETS);
        #[cfg(feature = "rp2040")]
        Mono::start(ctx.device.TIMER, &ctx.device.RESETS);

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
        blink::spawn().ok();
        (Shared {}, Local {led: led_pin})
    }

    #[task(local = [led])]
    async fn blink(ctx: blink::Context) {
        loop {
            _ = ctx.local.led.toggle();

            Mono::delay(1000.millis()).await;
        }
    }
}