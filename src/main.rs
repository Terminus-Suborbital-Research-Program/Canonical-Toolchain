// Specifies that the standard library is not used
#![no_std]
#![no_main]

mod hc12;
mod serial_handler;
mod utilities;

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
    use canonical_toolchain::{print, println};
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use embedded_io::{Read, ReadReady};
    use fugit::RateExtU32;
    use hal::{
        gpio::{self, FunctionSio, PullNone, SioOutput},
        sio::Sio,
    };
    use rp235x_hal::{
        clocks::init_clocks_and_plls,
        uart::{DataBits, StopBits, UartConfig, UartPeripheral},
        Clock, Watchdog,
    };
    const XTAL_FREQ_HZ: u32 = 12_000_000u32;

    use usb_device::{class_prelude::*, prelude::*};
    use usbd_serial::{embedded_io::Write, SerialPort};

    use hc12::{BaudRate, HC12};

    use rtic_sync::{
        channel::{Receiver, Sender},
        make_channel,
    };

    pub type GPIO2 = gpio::Pin<hal::gpio::bank0::Gpio2, gpio::FunctionSioOutput, gpio::PullDown>;
    pub type UARTBus = UartPeripheral<
        rp235x_hal::uart::Enabled,
        rp235x_hal::pac::UART0,
        (
            gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
        ),
    >;

    static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    use serial_handler::{HeaplessString, HEAPLESS_STRING_ALLOC_LENGTH, MAX_USB_LINES};

    #[shared]
    struct Shared {
        hc12: HC12<UARTBus, GPIO2>,
        usb_serial: SerialPort<'static, hal::usb::UsbBus>,
        usb_device: UsbDevice<'static, hal::usb::UsbBus>,
        serial_console_writer: serial_handler::SerialWriter,
        clock_freq_hz: u32,
    }

    #[local]
    struct Local {
        led: gpio::Pin<gpio::bank0::Gpio25, FunctionSio<SioOutput>, PullNone>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        // Reset the spinlocks - this is scipped by soft-reset
        unsafe {
            hal::sio::spinlock_reset();
        }

        // Channel for sending strings to the USB console
        let (usb_console_line_sender, usb_console_line_receiver) =
            make_channel!(HeaplessString, MAX_USB_LINES);

        // Channel for incoming commands from the USB console
        let (usb_console_command_sender, usb_console_command_receiver) =
            make_channel!(HeaplessString, MAX_USB_LINES);

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
        )
        .ok()
        .unwrap();

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
        let mut led_pin = pins
            .gpio25
            .into_pull_type::<PullNone>()
            .into_push_pull_output();
        led_pin.set_low().unwrap();
        // Start the heartbeat task
        heartbeat::spawn().ok();

        // Get clock frequency
        let clock_freq = clocks.peripheral_clock.freq();

        // Pin setup for UART
        let uart_pins = (pins.gpio0.into_function(), pins.gpio1.into_function());
        let mut uart_peripheral =
            UartPeripheral::new(ctx.device.UART0, uart_pins, &mut ctx.device.RESETS)
                .enable(
                    UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
                    clocks.peripheral_clock.freq(),
                )
                .unwrap();
        uart_peripheral.enable_rx_interrupt(); // Make sure we can drive our interrupts

        // Use pin 4 (GPIO2) as the HC12 configuration pin
        let hc12_configure_pin = pins.gpio2.into_push_pull_output();
        let hc12 = HC12::new(uart_peripheral, hc12_configure_pin).unwrap();

        // Set up USB Device allocator
        let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
            ctx.device.USB,
            ctx.device.USB_DPRAM,
            clocks.usb_clock,
            true,
            &mut ctx.device.RESETS,
        ));
        unsafe {
            USB_BUS = Some(usb_bus);
        }
        let usb_bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

        let serial = SerialPort::new(usb_bus_ref);
        let usb_dev = UsbDeviceBuilder::new(usb_bus_ref, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[StringDescriptors::default()
                .manufacturer("UAH TERMINUS PROGRAM")
                .product("Canonical Toolchain USB Serial Port")
                .serial_number("TEST")])
            .unwrap()
            .device_class(2)
            .build();

        usb_serial_console_printer::spawn(usb_console_line_receiver).ok();
        usb_console_reader::spawn(usb_console_command_sender).ok();
        command_handler::spawn(usb_console_command_receiver).ok();

        // Serial Writer Structure
        let serial_console_writer = serial_handler::SerialWriter::new(usb_console_line_sender);

        (
            Shared {
                hc12,
                usb_device: usb_dev,
                usb_serial: serial,
                serial_console_writer,
                clock_freq_hz: clock_freq.to_Hz(),
            },
            Local { led: led_pin },
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

    // Updates the HC12 module on the serial interrupt
    #[task(binds = UART0_IRQ, shared = [hc12])]
    fn uart_interrupt(mut ctx: uart_interrupt::Context) {
        ctx.shared.hc12.lock(|hc12| {
            hc12.update().ok();
        });
    }

    #[task(priority = 3, shared = [usb_device, usb_serial, serial_console_writer])]
    async fn usb_console_reader(
        mut ctx: usb_console_reader::Context,
        mut command_sender: Sender<
            'static,
            heapless::String<HEAPLESS_STRING_ALLOC_LENGTH>,
            MAX_USB_LINES,
        >,
    ) {
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

                                    // Write to serial to echo
                                    serial.write(&[*byte]).ok();

                                    // Detect eol
                                    if c == '\r' || c == '\n' {
                                        end_of_line = true;
                                        serial.write_all("\r\n".as_bytes()).ok();
                                    }

                                    if c == '\x08' || c == '\x7F' {
                                        command_buffer.pop();
                                        serial.write_all("\x08 \x08".as_bytes()).ok();
                                    } else {
                                        // Append to buffer
                                        command_buffer.push(c).ok();
                                    }
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
                end_of_line = false;
                // Send the command to the command handler
                command_sender.try_send(command_buffer.clone()).ok();
                command_buffer.clear();
            }

            // Wait for a bit to poll again
            Mono::delay(1000.micros()).await;
        }
    }

    #[task(priority = 3, shared = [usb_device, usb_serial])]
    async fn usb_serial_console_printer(
        mut ctx: usb_serial_console_printer::Context,
        mut reciever: Receiver<
            'static,
            heapless::String<HEAPLESS_STRING_ALLOC_LENGTH>,
            MAX_USB_LINES,
        >,
    ) {
        while let Ok(mut line) = reciever.recv().await {
            // If the line ends with a newline, pop it off, and then add a \r\n
            if line.ends_with('\n') {
                line.pop();
                line.push_str("\r\n").ok();
            }

            ctx.shared.usb_device.lock(|_usb_dev| {
                ctx.shared.usb_serial.lock(|serial| {
                    let mut wr_ptr = line.as_bytes();
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            Err(_) => break,
                        }
                    }
                })
            })
        }
    }

    // Command Handler
    #[task(shared=[serial_console_writer, hc12, clock_freq_hz], priority = 2)]
    async fn command_handler(
        mut ctx: command_handler::Context,
        mut reciever: Receiver<
            'static,
            heapless::String<HEAPLESS_STRING_ALLOC_LENGTH>,
            MAX_USB_LINES,
        >,
    ) {
        while let Ok(line) = reciever.recv().await {
            // Split into commands and arguments, on whitespace
            let mut parts = line.split_whitespace();

            // Get the command
            let command = parts.next().unwrap_or_default();

            match command {
                "usb-reboot" => {
                    // Reboots to the USB bootloader interface
                    println!(ctx, "Rebooting...");

                    hal::reboot::reboot(
                        hal::reboot::RebootKind::BootSel {
                            picoboot_disabled: false,
                            msd_disabled: false,
                        },
                        hal::reboot::RebootArch::Normal,
                    );
                }

                "hc-selftest" => {
                    hc12_selftest::spawn().ok();
                }

                // Sends all characters after the command to the HC12 module
                "hc-send-string" => {
                    // Get the string to send
                    let string = parts.collect::<heapless::String<64>>();

                    ctx.shared.hc12.lock(|hc12| {
                        println!(ctx, "Writing string: {}", string);
                        match hc12.write_str(&string) {
                            Ok(_) => {
                                println!(ctx, "String wrote successfully!");
                            }

                            Err(e) => {
                                println!(ctx, "Error writing string: {:?}", e);
                            }
                        }

                        match hc12.flush() {
                            Ok(_) => {
                                println!(ctx, "String flushed successfully!");
                            }

                            Err(e) => {
                                println!(ctx, "Error flushing string: {:?}", e);
                            }
                        }
                    });
                }

                "hc-set-channel" => {
                    // Try to get channel, or inform user of error
                    let value = parts.next().and_then(|s| s.parse::<u8>().ok()).unwrap_or(0);

                    ctx.shared.hc12.lock(|hc12| {
                        if hc12.set_mode(hc12::HC12Mode::Configuration).is_err() {
                            println!(ctx, "Error entering configuration mode");
                            return;
                        }
                    });

                    // Delay for 100ms
                    Mono::delay(100.millis()).await;

                    ctx.shared.hc12.lock(|hc12| {
                        println!(ctx, "Setting channel to: {}", value);
                        match hc12.set_channel(value) {
                            Ok(_) => {
                                println!(ctx, "Channel set successfully!");
                            }

                            Err(e) => {
                                println!(ctx, "Error setting channel: {:?}", e);
                            }
                        }
                    });

                    // Delay for 100ms and clear the buffer
                    Mono::delay(100.millis()).await;

                    ctx.shared.hc12.lock(|hc12| {
                        hc12.clear();
                    });
                }

                "clock-freq" => {
                    // Print the current clock frequency
                    ctx.shared.clock_freq_hz.lock(|freq| {
                        println!(ctx, "Clock Frequency: {} Hz", freq);
                    });
                }

                "hc-set-baudrate" => {
                    // Get the baudrate and make sure it's valid
                    let baudrate = parts
                        .next()
                        .and_then(|s| s.parse::<u32>().ok());

                    let baudrate = match baudrate {
                        Some(num) => {
                            match BaudRate::from_u32(num) {
                                Ok(rate) => rate,
                                Err(_) => {
                                    println!(ctx, "Invalid Baudrate: {}", num);
                                    return;
                                }
                            }
                        }

                        None => {
                            println!(ctx, "Bad String: {}", line);
                            return;
                        }
                    };
                    
                    let mut freq = 0;
                    ctx.shared.clock_freq_hz.lock(|clock_freq| {
                        freq = *clock_freq;
                    });

                    // Set the baudrate
                    ctx.shared.hc12.lock(|hc12| {
                        match hc12.set_baudrate(baudrate, freq.Hz()) {
                            Ok(_) => {
                                println!(ctx, "Baudrate set successfully!");
                            }

                            Err(e) => {
                                println!(ctx, "Error setting baudrate: {:?}", e);
                            }
                        }
                    });
                }

                "hc-set-power" => {
                    // Try to get power, or inform user of error
                    let value = parts.next().and_then(|s| s.parse::<u8>().ok()).unwrap_or(0);

                    ctx.shared.hc12.lock(|hc12| {
                        if hc12.set_mode(hc12::HC12Mode::Configuration).is_err() {
                            println!(ctx, "Error entering configuration mode");
                            return;
                        }
                    });

                    // Delay for 100ms
                    Mono::delay(100.millis()).await;

                    ctx.shared.hc12.lock(|hc12| {
                        println!(ctx, "Setting power to: {}", value);
                        match hc12.set_power(value) {
                            Ok(_) => {
                                println!(ctx, "Power set successfully!");
                            }

                            Err(e) => {
                                println!(ctx, "Error setting power: {:?}", e);
                            }
                        }
                    });

                    // Delay for 100ms and clear the buffer
                    Mono::delay(100.millis()).await;

                    ctx.shared.hc12.lock(|hc12| {
                        hc12.clear();
                    });
                }

                "sp" => {
                    // Print the stack pointer
                    println!(
                        ctx,
                        "Stack Pointer: 0x{:08X}",
                        utilities::get_stack_pointer()
                    );
                }

                _ => {
                    println!(ctx, "Invalid command: {}", command);
                }
            }
        }
    }

    #[task(shared = [serial_console_writer, hc12], priority = 2)]
    async fn hc12_selftest(mut ctx: hc12_selftest::Context) {
        ctx.shared.hc12.lock(|hc12| {
            println!(ctx, "Running HC12 Self Test...");

            // Flush the incoming buffer
            hc12.clear();

            println!(ctx, "Entering Configuration Mode...");
            // hc12.set_mode(hc12::HC12Mode::Configuration).unwrap();
            match hc12.set_mode(hc12::HC12Mode::Configuration) {
                Ok(_) => {
                    println!(ctx, "Entered Configuration Mode Successfully!");
                }

                Err(e) => {
                    println!(ctx, "Error entering configuration mode: {:?}", e);
                }
            }
        });

        Mono::delay(1000.millis()).await;

        ctx.shared.hc12.lock(|hc12| match hc12.check_at() {
            Err(e) => {
                println!(ctx, "Error checking AT: {:?}", e);
            }

            Ok(_) => {
                println!(ctx, "AT Check Successful, waiting for response...");
            }
        });

        Mono::delay(1000.millis()).await;

        ctx.shared.hc12.lock(|hc12| {
            match hc12.check_ok() {
                true => {
                    println!(ctx, "OK ACK Received!");
                }

                false => {
                    println!(ctx, "Error: No OK Response Received!");
                }
            }

            println!(ctx, "Exiting Configuration Mode...");
            hc12.set_mode(hc12::HC12Mode::Normal).unwrap();

            println!(ctx, "HC12 Self-Test Complete!");

            hc12.clear();
        });
    }
}
