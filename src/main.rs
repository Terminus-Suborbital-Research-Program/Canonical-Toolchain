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
    dispatchers = [PIO2_IRQ_0, PIO2_IRQ_1, DMA_IRQ_0],
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
        clocks::init_clocks_and_plls, gpio::{bank0::{Gpio0, Gpio1}, new_pin}, uart::{
            DataBits, StopBits, UartConfig, UartPeripheral, 
        }, Clock, Watchdog
    };
    const XTAL_FREQ_HZ: u32 = 12_000_000u32;

    use usb_device::{class_prelude::*, prelude::*};
    use usbd_serial::SerialPort;

    use rtic_sync::{
        channel::{
            Receiver,
            Sender,
        },
        make_channel,
    };

    type UARTBus = UartPeripheral<rp235x_hal::uart::Enabled, rp235x_hal::pac::UART0, (gpio::Pin<Gpio0, gpio::FunctionUart, gpio::PullDown>, gpio::Pin<Gpio1, gpio::FunctionUart, gpio::PullDown>)>;

    static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    // Max number of lines to buffer in the USB console
    const MAX_USB_LINES: usize = 5;
    // Default length of buffer strings used
    const HEAPLESS_STRING_ALLOC_LENGTH: usize = 64;
    // Typedef for heapless string
    type HeaplessString = heapless::String<HEAPLESS_STRING_ALLOC_LENGTH>;

    #[shared]
    struct Shared {
        uart0: UARTBus,
        usb_serial: SerialPort<'static, hal::usb::UsbBus>,
        usb_device: UsbDevice<'static, hal::usb::UsbBus>,
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

        let (s,r) = make_channel!(HeaplessString, MAX_USB_LINES);        

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

        // Pin setup for UART
        let uart_pins =  (pins.gpio0.into_function(), pins.gpio1.into_function());
        let uart_peripheral = UartPeripheral::new(ctx.device.UART0, uart_pins, &mut ctx.device.RESETS).enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq()
        ).unwrap();

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

        let serial = SerialPort::new(usb_bus_ref);
        let usb_dev = UsbDeviceBuilder::new(usb_bus_ref, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[StringDescriptors::default()
                .manufacturer("UAH TERMINUS PROGRAM")
                .product("Canonical Toolchain USB Serial Port")
                .serial_number("TEST")])
            .unwrap()
            .device_class(2)
            .build();

        send_usb_lines::spawn(r).ok();
        usb_console_reader::spawn(s).ok();

        (
            Shared {
                uart0: uart_peripheral,
                usb_device: usb_dev,
                usb_serial: serial,
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

    #[task(priority = 3, shared = [usb_device, usb_serial])]
    async fn usb_console_reader(mut ctx: usb_console_reader::Context, mut usb_line_sender: Sender<'static, heapless::String::<HEAPLESS_STRING_ALLOC_LENGTH>, MAX_USB_LINES>) {
        let mut buf = [0u8; 64];
        let mut command_buffer = heapless::String::<HEAPLESS_STRING_ALLOC_LENGTH>::new();

        let mut end_of_line = false;

        loop {
            ctx.shared.usb_device.lock(|usb_dev| {
                ctx.shared.usb_serial.lock(|serial| {
                    if usb_dev.poll(&mut [serial]) {
                        // For the moment, we're just going to echo back the input, after a newline
                        match serial.read(&mut buf) {
                            Ok(count) if count > 0 => {
                                // Collect buffer into an array
                                let bytes = &buf[..count];
                                for byte in bytes.iter() {
                                    // Conv to char
                                    let c = *byte as char;

                                    // Detect eol
                                    if c == '\n' {
                                        end_of_line = true;
                                    }

                                    // Append to buffer
                                    command_buffer.push(c).ok();
                                }
                            }

                            _ => {
                                // Ignore errors on read, assume it was just a desync
                            }
                        }
                    }
                })
            });

            if end_of_line {
                // Now that we've released the usb lock, we can send it back
                usb_line_sender.try_send(command_buffer.clone()).ok();
                end_of_line = false;
            }

            // Wait for a bit to poll again
            Mono::delay(100.micros()).await;
        }
    }

    #[task(priority = 3, shared = [usb_device, usb_serial])]
    async fn send_usb_lines(mut ctx: send_usb_lines::Context, mut reciever: Receiver<'static, heapless::String::<HEAPLESS_STRING_ALLOC_LENGTH>, MAX_USB_LINES>) {
        while let Ok(line) = reciever.recv().await {
            ctx.shared.usb_device.lock(|_usb_dev| {
                ctx.shared.usb_serial.lock(|serial| {
                    let mut wr_ptr = line.as_bytes();
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            Err(_) => break
                        }
                    }
                })
            })
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