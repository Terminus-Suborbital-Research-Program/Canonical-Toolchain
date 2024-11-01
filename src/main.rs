// Specifies that the standard library is not used
#![no_std]
#![no_main]

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
            DataBits, StopBits, UartConfig, UartPeripheral, 
        },
        Clock, 
        Watchdog
    };
    const XTAL_FREQ_HZ: u32 = 12_000_000u32;

    use usb_device::{class_prelude::*, prelude::*};
    use usbd_serial::SerialPort;

    type UARTBus = UartPeripheral<rp235x_hal::uart::Enabled, rp235x_hal::pac::UART0, (gpio::Pin<Gpio16, gpio::FunctionUart, gpio::PullDown>, gpio::Pin<Gpio17, gpio::FunctionUart, gpio::PullDown>)>;

    static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    #[shared]
    struct Shared<'a> {
        uart0: UARTBus,
        usb_serial: SerialPort<'static, hal::usb::UsbBus>,
        usb_device: UsbDevice<'static, hal::usb::UsbBus>
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

        // Set up USB Device allocator
        let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
            ctx.device.USB, 
            ctx.device.USB_DPRAM, 
            clocks.usb_clock, 
            true,
            &mut ctx.device.RESETS 
        ));
        unsafe {
            USB_BUS = Some(usb_bus);
        }
        let usb_bus_ref = unsafe {
            USB_BUS.as_ref().unwrap()
        };

        let mut serial = SerialPort::new(usb_bus_ref);
        let mut usb_dev = UsbDeviceBuilder::new(usb_bus_ref, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[StringDescriptors::default()
                .manufacturer("UAH TERMINUS PROGRAM")
                .product("Canonical Toolchain USB Serial Port")
                .serial_number("TEST")])
            .unwrap()
            .device_class(2)
            .build();

        usb_serial_hello::spawn().ok();
        telemetry::spawn().ok();

        (
            Shared {
                uart0: uart_peripheral,
                usb_device: usb_dev,
                usb_serial: serial
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

    // USB Serial Echo
    #[task(priority = 2, shared=[usb_device, usb_serial])]
    async fn usb_serial_hello(mut ctx: usb_serial_hello::Context) {
        // Set up usb device
        loop {
            ctx.shared.usb_device.lock(|usb_dev| {
                ctx.shared.usb_serial.lock(|serial| {  
                    if usb_dev.poll(&mut [serial]) {
                        let mut buf = [0u8; 64];
                        match serial.read(&mut buf) {
                            Ok(count) => {
                                // Convert to upper
                                buf.iter_mut().take(count).for_each(|b| {
                                    b.make_ascii_uppercase();
                                });

                                // Send back
                                let mut wr_ptr = &buf[..count];
                                while !wr_ptr.is_empty() {
                                    match serial.write(wr_ptr) {
                                        Ok(len) => wr_ptr = &wr_ptr[len..],

                                        Err(_) => break,
                                    }
                                }
                            }

                            Err(_) => {

                            }
                        }
                    }
                });
            });

            Mono::delay(100.micros()).await;
        }
        
    }

    // UART Serial
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