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

// MonotonicTimertonics
#[cfg(feature = "rp2350")]
use rtic_monotonics::rp235x::prelude::*;
#[cfg(feature = "rp2350")]
rp235x_timer_monotonic!(MonotonicTimer);

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

    pub type GPIO3 = gpio::Pin<hal::gpio::bank0::Gpio3, gpio::FunctionSioOutput, gpio::PullNone>;
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
        hc12: HC12<UARTBus, GPIO3>,
        usb_serial: SerialPort<'static, hal::usb::UsbBus>,
        usb_device: UsbDevice<'static, hal::usb::UsbBus>,
        serial_console_writer: serial_handler::SerialWriter,
        clock_freq_hz: u32,
        echo_hc12: bool,
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

        MonotonicTimer::start(ctx.device.TIMER0, &ctx.device.RESETS);

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
        let mut hc12_configure_pin = pins.gpio3
            .into_pull_type::<PullNone>().into_push_pull_output();
        hc12_configure_pin.set_high().unwrap();
        hc12_configure_pin.set_drive_strength(gpio::OutputDriveStrength::TwelveMilliAmps);
        
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
                echo_hc12: false,
            },
            Local { led: led_pin },
        )
    }

    // Heartbeats the main led
    #[task(local = [led], priority = 1)]
    async fn heartbeat(ctx: heartbeat::Context) {
        loop {
            _ = ctx.local.led.toggle();

            MonotonicTimer::delay(500.millis()).await;
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
            MonotonicTimer::delay(1000.micros()).await;
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
    #[task(shared=[serial_console_writer, hc12, clock_freq_hz, echo_hc12], priority = 2)]
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

                "echo" => {
                    hc12_echo::spawn().ok();
                }

                "mode" => {
                    ctx.shared.hc12.lock(|hc12| {
                        let mode = hc12.config_pin.is_set_high().unwrap();
                        let drive_strength = hc12.config_pin.get_drive_strength();
                        let output_override = hc12.config_pin.get_output_override();

                        println!(ctx, "Config pin is set high: {}", mode);
                        println!(ctx, "Drive Strength: {:?}", drive_strength);
                        println!(ctx, "Output Override: {:?}", output_override);
                    });
                }

                // Sends all characters after the command to the HC12 module
                "hc-sendline" => {
                    // Get the string to send
                    let mut string = parts.collect::<heapless::String<65>>();
                    string.push('\n').ok();

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

                    // Delay for 100ms
                    MonotonicTimer::delay(100.millis()).await;

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
                    MonotonicTimer::delay(100.millis()).await;

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

                "hc-mode" => {
                    // config -> configuration mode
                    // normal -> normal mode

                    let mode = parts.next().unwrap_or_default();

                    let mode = match mode {
                        "config" => hc12::HC12Mode::Configuration,
                        "normal" => hc12::HC12Mode::Normal,
                        _ => {
                            println!(ctx, "Invalid Mode: {}", mode);
                            return;
                        }
                    };

                    ctx.shared.hc12.lock(|hc12| {
                        match hc12.set_mode(mode) {
                            Ok(_) => {
                                println!(ctx, "Mode set successfully!");
                            }

                            Err(e) => {
                                println!(ctx, "Error setting mode: {:?}", e);
                            }
                        }
                    });
                }

                "rx" => {
                    // Gets parameters for the HC12 module
                    ctx.shared.hc12.lock(|hc12| {
                        let x = hc12.parameters();

                        match x {
                            Err(e) => {
                                println!(ctx, "Error getting parameters: {:?}", e);
                            }

                            _ => {}
                        }
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

                    // Delay for 100ms and clear the buffer
                    MonotonicTimer::delay(100.millis()).await;

                    ctx.shared.hc12.lock(|hc12| {
                        hc12.clear();
                    });
                }

                "sr" => {
                    // Sends and receives a string, then prints the time taken to do so
                    // 100 bytes
                    let mut string = heapless::String::<100>::new();
                    for i in 0..25 {
                        string.push(((i % 10) as u8 + 48) as char).ok();
                    }

                    let start_time = MonotonicTimer::now();

                    ctx.shared.hc12.lock(|hc12| {
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

                    while ctx.shared.hc12.lock(|hc12| hc12.bytes_available()) < 100 {
                        ctx.shared.hc12.lock(|hc12| {
                            if hc12.bytes_available() > 25 {
                                println!(ctx, "Bytes Available: {}", hc12.bytes_available());
                            }
                            hc12.update().ok();
                        });
                        MonotonicTimer::delay(100.nanos()).await;
                    }

                    let end_time = MonotonicTimer::now();

                    let time = end_time - start_time;

                    println!(ctx, "Sent/received 200 bytes in total in {} ms", time.to_millis());
                    println!(ctx, "Data Rate: {:.3} KBps", (200.0 / time.to_millis() as f32));
                }

                "hc-set-power" => {
                    // Try to get power, or inform user of error
                    let value = parts.next().and_then(|s| s.parse::<u8>().ok()).unwrap_or(0);

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
                    MonotonicTimer::delay(1000.millis()).await;

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


                "line" => {
                    let mut line_found = false;
                    let mut first_char_recvd = false;
                    let mut first_char_at = MonotonicTimer::now();
                    let mut last_char_at = MonotonicTimer::now();
                    let start_time = MonotonicTimer::now();
                    let timed_out = 10_000.millis();
                    let end_time = start_time + timed_out;

                    while MonotonicTimer::now() < end_time && !line_found {
                        ctx.shared.hc12.lock(|hc12| {
                            let time = MonotonicTimer::now();
                            if hc12.bytes_available() > 0 && !first_char_recvd {
                                first_char_recvd = true;
                                first_char_at = time;
                            }
                            last_char_at = time;

                            if hc12.line_available() {
                                line_found = true;
                            }
                        });

                        MonotonicTimer::delay(100.millis()).await;
                    }

                    if line_found {
                        println!(ctx, "Line Received!");
                        let line = match ctx.shared.hc12.lock(|hc12| hc12.read_line()) {
                            Ok(line) => line,
                            Err(e) => {
                                println!(ctx, "Error reading line: {:?}", e);
                                return;
                            }
                        };
                        let len = line.len();
                        let time = last_char_at - first_char_at;

                        println!(ctx, "Line: {}", line);
                        println!(ctx, "Received {} bytes in {} ms", len, time.to_millis());
                        println!(ctx, "Data Rate: {:.3} KBps", (len as f32 / time.to_millis() as f32));
                    } else {
                        println!(ctx, "Timed out!");
                    }
                }

                "readall" => {
                    ctx.shared.hc12.lock(|hc12| {
                        while let Ok(str) = hc12.read_line() {
                            print!(ctx, "{}", str);
                        }
                    });
                }

                "firmware" => {
                    ctx.shared.hc12.lock(|hc12| {
                        hc12.send_at("V").ok();
                    })
                }

                "default-config" => {
                    ctx.shared.hc12.lock(|hc12| {
                        hc12.set_mode(hc12::HC12Mode::Configuration).ok();
                        let frequency = ctx.shared.clock_freq_hz.lock(|freq| *freq);
                        hc12.set_baudrate(BaudRate::B9600, frequency.Hz()).ok();
                        hc12.set_power(8).ok();
                        hc12.set_channel(120).ok();
                        hc12.set_mode(hc12::HC12Mode::Normal).ok();
                    })
                }

                _ => {
                    println!(ctx, "Invalid command: {}", command);
                }
            }
        }
    }

    // Repeats lines back across the HC12 module when received, while running
    #[task(shared = [hc12, echo_hc12, serial_console_writer], priority = 2)]
    async fn hc12_echo(mut ctx: hc12_echo::Context) {
        loop {
            ctx.shared.hc12.lock(|hc12| {
               if let Ok(line) = hc12.read_line() {
                   hc12.write_str(&line).ok();
                   println!(ctx, "Line: {}", line);
                   hc12.flush().ok();
               }
            });

            MonotonicTimer::delay(100.millis()).await;
        }
    }

    #[task(shared = [serial_console_writer, hc12], priority = 2)]
    async fn hc12_selftest(mut ctx: hc12_selftest::Context) {
        ctx.shared.hc12.lock(|hc12| {
            println!(ctx, "Running HC12 Self Test...");

            // Flush the incoming buffer
            hc12.clear();

            println!(ctx, "Entering Configuration Mode...");
            match hc12.set_mode(hc12::HC12Mode::Configuration) {
                Ok(_) => {
                    println!(ctx, "Entered Configuration Mode Successfully!");
                }

                Err(e) => {
                    println!(ctx, "Error entering configuration mode: {:?}", e);
                }
            }
        });

        MonotonicTimer::delay(1000.millis()).await;

        ctx.shared.hc12.lock(|hc12| match hc12.check_at() {
            Err(e) => {
                println!(ctx, "Error checking AT: {:?}", e);
            }

            Ok(_) => {
                println!(ctx, "AT Check Successful, waiting for response...");
            }
        });

        MonotonicTimer::delay(1000.millis()).await;

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
