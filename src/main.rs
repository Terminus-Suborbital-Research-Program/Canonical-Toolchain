// Specifies that the standard library is not used
#![no_std]
#![no_main]

// Our Modules
pub mod actuators;
pub mod communications;
pub mod sensors;
pub mod utilities;

// We require an allocator for some heap stuff - unfortunatly bincode serde
// doesn't have support for heapless vectors yet
extern crate alloc;
use linked_list_allocator::LockedHeap;

#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();
static mut HEAP_MEMORY: [u8; 1024 * 64] = [0; 1024 * 64];

static min_duty: u32 = 2200;
static max_duty: u32 = 8200;

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
    use crate::{actuators::servo::Servo, communications::link_layer::Device};

    use super::*;
    use actuators::*;
    use application_layer::{CommandPacket, ScientificPacket};
    use bincode::error::DecodeError::UnexpectedVariant;
    use communications::{link_layer::LinkLayerDevice, serial_handler::HeaplessString, *};
    use sensors::*;
    use utilities::*;

    use canonical_toolchain::{print, println};
    use embedded_hal::{
        digital::{OutputPin, StatefulOutputPin},
        pwm::SetDutyCycle,
    };
    use fugit::{ExtU32, RateExtU32};
    use hal::{
        gpio::{self, FunctionSio, PullNone, SioOutput},
        sio::Sio,
    };
    use rp235x_hal::{
        clocks::init_clocks_and_plls,
        pwm::{Channel, CountFallingEdge, FreeRunning, InputHighRunning, Pwm7, Slice, Slices, B},
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
    use serial_handler::{HEAPLESS_STRING_ALLOC_LENGTH, MAX_USB_LINES};

    pub type UART0Bus = UartPeripheral<
        rp235x_hal::uart::Enabled,
        rp235x_hal::pac::UART0,
        (
            gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
        ),
    >;

    pub type GPIO7 = gpio::Pin<hal::gpio::bank0::Gpio7, gpio::FunctionSioOutput, gpio::PullDown>;
    pub type UART1Bus = UartPeripheral<
        rp235x_hal::uart::Enabled,
        rp235x_hal::pac::UART1,
        (
            gpio::Pin<gpio::bank0::Gpio8, gpio::FunctionUart, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio9, gpio::FunctionUart, gpio::PullDown>,
        ),
    >;

    pub type EjectorServoChannel = Servo<
        Channel<Slice<Pwm7, FreeRunning>, B>,
        gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionPwm, gpio::PullDown>,
    >;

    static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    use core::fmt::Write as CoreWrite;

    #[shared]
    struct Shared {
        uart0: UART0Bus,
        uart0_buffer: heapless::String<HEAPLESS_STRING_ALLOC_LENGTH>,
        ejector_pin: EjectorServoChannel,
        radio_link: LinkLayerDevice<HC12<UART1Bus, GPIO7>>,
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
        // Reset the spinlocks - this is skipped by soft-reset
        unsafe {
            hal::sio::spinlock_reset();
        }

        // Set up the global allocator
        unsafe {
            ALLOCATOR
                .lock()
                .init(HEAP_MEMORY.as_ptr() as *mut u8, HEAP_MEMORY.len());
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
        let bank0_pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        // Configure GPIO25 as an output
        let mut led_pin = bank0_pins
            .gpio25
            .into_pull_type::<PullNone>()
            .into_push_pull_output();
        led_pin.set_low().unwrap();
        // Start the heartbeat task
        heartbeat::spawn().ok();

        // Get clock frequency
        let clock_freq = clocks.peripheral_clock.freq();

        // Pin setup for UART0
        let uart0_pins = (
            bank0_pins.gpio0.into_function(),
            bank0_pins.gpio1.into_function(),
        );
        let mut uart0_peripheral =
            UartPeripheral::new(ctx.device.UART0, uart0_pins, &mut ctx.device.RESETS)
                .enable(
                    UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
                    clocks.peripheral_clock.freq(),
                )
                .unwrap();
        uart0_peripheral.enable_rx_interrupt(); // Make sure we can drive our interrupts
        let uart0_buffer: heapless::String<HEAPLESS_STRING_ALLOC_LENGTH> = heapless::String::new(); // Allocate uart0_buffer

        // Pin setup for UART1
        let uart1_pins = (
            bank0_pins.gpio8.into_function(),
            bank0_pins.gpio9.into_function(),
        );
        let mut uart1_peripheral =
            UartPeripheral::new(ctx.device.UART1, uart1_pins, &mut ctx.device.RESETS)
                .enable(
                    UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
                    clocks.peripheral_clock.freq(),
                )
                .unwrap();
        uart1_peripheral.enable_rx_interrupt(); // Make sure we can drive our interrupts

        // Use pin 4 (GPIO2) as the HC12 configuration pin
        let hc12_configure_pin = bank0_pins.gpio7.into_push_pull_output();
        let hc12 = HC12::new(uart1_peripheral, hc12_configure_pin).unwrap();
        let radio_link = LinkLayerDevice {
            device: hc12,
            me: Device::Ejector,
        };

        // Servo
        let pwm_slices = Slices::new(ctx.device.PWM, &mut ctx.device.RESETS);
        let mut pwm = pwm_slices.pwm7;
        pwm.enable();
        pwm.set_div_int(48);

        let mut channel_b = pwm.channel_b;
        let channel_pin = channel_b.output_to(bank0_pins.gpio15);
        let cycle = min_duty + ((max_duty - min_duty) * 90) / 200;
        channel_b.set_duty_cycle(cycle as u16).unwrap();
        channel_b.set_enabled(true);

        let ejection_servo = Servo::new(channel_b, channel_pin);

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
        radio_flush::spawn().ok();

        // Serial Writer Structure
        let serial_console_writer = serial_handler::SerialWriter::new(usb_console_line_sender);

        (
            Shared {
                uart0: uart0_peripheral,
                uart0_buffer,
                radio_link,
                ejector_pin: ejection_servo,
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

            Mono::delay(500_u64.millis()).await;
        }
    }

    // Updates the radio module on the serial interrupt
    #[task(binds = UART0_IRQ, shared = [radio_link, serial_console_writer])]
    fn uart_interrupt(mut ctx: uart_interrupt::Context) {
        ctx.shared.radio_link.lock(|radio| {
            radio.device.update().ok();
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
            Mono::delay(1000_u64.micros()).await;
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

    // Radio Flush Task
    #[task(shared = [radio_link], priority = 2)]
    async fn radio_flush(mut ctx: radio_flush::Context) {
        let mut on_board_baudrate: BaudRate = BaudRate::B9600;
        let bytes_to_flush = 16;

        loop {
            ctx.shared.radio_link.lock(|radio| {
                radio.device.flush(bytes_to_flush).ok();
                on_board_baudrate = radio.device.get_baudrate();
            });

            // Need to wait wait the in-air baudrate, or the on-board baudrate
            // whichever is slower

            let mut slower =
                core::cmp::min(on_board_baudrate.to_u32(), on_board_baudrate.to_in_air_bd());

            // slower is bps, so /1000 to get ms
            slower = slower / 1000;

            // Delay for that times the number of bytes flushed
            Mono::delay((slower as u64 * bytes_to_flush as u64).millis()).await;
        }
    }

    // Command Handler
    #[task(shared=[serial_console_writer, radio_link, clock_freq_hz, ejector_pin], priority = 2)]
    #[cfg(debug_assertions)]
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

                "set-servo" => {
                    // Parse arg as int or fail
                    let arg = parts
                        .next()
                        .unwrap_or_default()
                        .parse::<u32>()
                        .unwrap_or_default();
                    //channel_b.set_duty_cycle_percent(0).unwrap();
                    ctx.shared.ejector_pin.lock(|channel| {
                        //let cycle = min_duty + ((max_duty - min_duty) * arg) / 200;
                        channel.set_angle(arg as u16);
                    });
                }

                "packet-test" => {
                    // Create a command packet
                    let packet = CommandPacket::MoveServoDegrees(90);

                    // Print it
                    println!(ctx, "{:?}", packet);

                    // Serialize it and print the vector
                    let serialized =
                        bincode::encode_to_vec(&packet, bincode::config::standard()).unwrap();

                    for byte in serialized.iter() {
                        print!(ctx, "{:02X} ", byte);
                        Mono::delay(100_u64.millis()).await;
                    }
                    println!(ctx, "");

                    // Deserialize it and print the packet
                    let deserialized: CommandPacket;
                    let _bytes: usize;

                    match bincode::decode_from_slice(&serialized, bincode::config::standard()) {
                        Ok((packet, bytes)) => {
                            deserialized = packet;
                            _bytes = bytes;

                            println!(ctx, "{:?}", deserialized);
                        }

                        Err(e) => match e {
                            // Unexpected varient
                            UnexpectedVariant {
                                type_name,
                                allowed,
                                found,
                            } => {
                                println!(ctx, "Unexpected:");
                                Mono::delay(1000_u64.millis()).await;
                                println!(ctx, "Type Name: {}", type_name);
                                Mono::delay(1000_u64.millis()).await;
                                println!(ctx, "Allowed: {:?}", allowed);
                                Mono::delay(1000_u64.millis()).await;
                                println!(ctx, "Found: {:?}", found);
                            }

                            _ => {
                                println!(ctx, "Error deserializing packet: {:?}", e);
                            }
                        },
                    }
                }

                // Peeks at the buffer, printing it to the console
                "link-peek" => {
                    ctx.shared.radio_link.lock(|radio| {
                        let buffer = radio.device.clone_buffer();
                        println!(ctx, "Radio Buffer: {}", buffer);
                    });
                }

                "clock-freq" => {
                    // Print the current clock frequency
                    ctx.shared.clock_freq_hz.lock(|freq| {
                        println!(ctx, "Clock Frequency: {} Hz", freq);
                    });
                }

                "sp" => {
                    // Print the stack pointer
                    println!(
                        ctx,
                        "Stack Pointer: 0x{:08X}",
                        utilities::arm::get_stack_pointer()
                    );
                }

                _ => {
                    println!(ctx, "Invalid command: {}", command);
                }
            }
        }
    }
}
