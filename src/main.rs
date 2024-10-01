// Specifies that the standard library is not used
#![no_std]
#![no_main]

use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac)]
mod app {
    use fugit::MicrosDurationU32;

    use embedded_hal::digital::OutputPin;

    use rp2040_hal::fugit::Duration;
    use rp2040_hal::timer::Alarm;
    use rp2040_hal::timer::CountDown;
    use rp_pico::hal;
    use rp_pico::pac;

    use rp_pico::XOSC_CRYSTAL_FREQ; // Frequency of the crystal on the board

    // Blink every 5 seconds
    const BLINK_TIME_US: u32 = 5_000_000;

    #[shared]
    struct Shared {
        timer: hal::Timer,
        alarm: hal::timer::Alarm0,
        led_on: bool,
        led_pin: rp2040_hal::gpio::Pin<rp2040_hal::gpio::bank0::Gpio25, rp2040_hal::gpio::FunctionSio<rp2040_hal::gpio::SioOutput>, rp2040_hal::gpio::PullDown>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let mut resets =  c.device.RESETS;
        let mut watchdog = hal::watchdog::Watchdog::new(c.device.WATCHDOG);

        let clocks = hal::clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ, 
            c.device.XOSC, 
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        ).ok().unwrap();

        // GPIO init
        let sio = hal::Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(c.device.IO_BANK0, c.device.PADS_BANK0, sio.gpio_bank0, &mut resets);
        let mut led = pins.led.into_push_pull_output();
        led.set_low().unwrap();

        // Timer init
        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets, &clocks);
        let mut alarm = timer.alarm_0().unwrap();
        let duration = MicrosDurationU32::micros(BLINK_TIME_US);
        alarm.schedule(duration).unwrap();

        (Shared {
            timer,
            alarm,
            led_on: false,
            led_pin: led,
        }, Local {})
        
    }
}