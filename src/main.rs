// Specifies that the standard library is not used
#![no_std]
#![no_main]

// Our Modules
pub mod actuators;
pub mod communications;
pub mod sensors;
pub mod startup;
pub mod tasks;
pub mod usb_commands;
pub mod usb_io;
pub mod utilities;

// We require an allocator for some heap stuff - unfortunatly bincode serde
// doesn't have support for heapless vectors yet
extern crate alloc;
use linked_list_allocator::LockedHeap;

use crate::tasks::*;
use crate::usb_commands::*;
use crate::usb_io::*;

#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();
static mut HEAP_MEMORY: [u8; 1024 * 64] = [0; 1024 * 64];

use panic_halt as _;

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
    peripherals = true,
)]
mod app {
    use crate::{
        actuators::servo::{
            EjectionServo, EjectionServoMosfet, LockingServo, LockingServoMosfet, Servo,
            LOCKING_SERVO_LOCKED,
        },
        communications::{
            hc12::{UART1Bus, GPIO10},
            link_layer::{Device, LinkPacket},
        },
    };

    use super::*;

    use bin_packets::packets::{CommandPacket, ConnectionTest};
    use bincode::{
        config::standard,
        error::DecodeError::{self, UnexpectedVariant},
    };
    use communications::{
        link_layer::{LinkLayerDevice, LinkLayerPayload},
        serial_handler::HeaplessString,
        *,
    };

    use canonical_toolchain::{print, println};
    use embedded_hal::digital::OutputPin;
    use fugit::RateExtU32;
    use hal::{
        gpio::{self, FunctionSio, PullNone, SioOutput},
        sio::Sio,
    };
    use rp235x_hal::{
        clocks::init_clocks_and_plls,
        pwm::Slices,
        uart::{DataBits, StopBits, UartConfig, UartPeripheral},
        Clock, Watchdog,
    };
    pub const XTAL_FREQ_HZ: u32 = 12_000_000u32;

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

    pub static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    #[shared]
    pub struct Shared {
        //uart0: UART0Bus,
        //uart0_buffer: heapless::String<HEAPLESS_STRING_ALLOC_LENGTH>,
        pub ejector_driver: EjectionServo,
        pub locking_driver: LockingServo,
        pub radio_link: LinkLayerDevice<HC12<UART1Bus, GPIO10>>,
        pub usb_serial: SerialPort<'static, hal::usb::UsbBus>,
        pub usb_device: UsbDevice<'static, hal::usb::UsbBus>,
        pub serial_console_writer: serial_handler::SerialWriter,
        pub clock_freq_hz: u32,
    }

    #[local]
    pub struct Local {
        pub led: gpio::Pin<gpio::bank0::Gpio25, FunctionSio<SioOutput>, PullNone>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        startup::startup(ctx)
    }

    extern "Rust" {
        // Heartbeats the main led
        #[task(local = [led], priority = 2)]
        async fn heartbeat(ctx: heartbeat::Context);

        // Takes care of incoming packets
        #[task(shared = [radio_link, serial_console_writer], priority = 1)]
        async fn incoming_packet_handler(mut ctx: incoming_packet_handler::Context);
    }

    extern "Rust" {
        // USB Console Reader
        #[task(priority = 3, shared = [usb_device, usb_serial, serial_console_writer])]
        async fn usb_console_reader(
            mut ctx: usb_console_reader::Context,
            mut command_sender: Sender<
                'static,
                heapless::String<HEAPLESS_STRING_ALLOC_LENGTH>,
                MAX_USB_LINES,
            >,
        );

        // USB Console Printer
        #[task(priority = 3, shared = [usb_device, usb_serial])]
        async fn usb_serial_console_printer(
            mut ctx: usb_serial_console_printer::Context,
            mut reciever: Receiver<
                'static,
                heapless::String<HEAPLESS_STRING_ALLOC_LENGTH>,
                MAX_USB_LINES,
            >,
        );

        // Command Handler
        #[task(shared=[serial_console_writer, radio_link, clock_freq_hz, ejector_driver, locking_driver], priority = 2)]
        async fn command_handler(
            mut ctx: command_handler::Context,
            mut reciever: Receiver<
                'static,
                heapless::String<HEAPLESS_STRING_ALLOC_LENGTH>,
                MAX_USB_LINES,
            >,
        );
    }

    // Updates the radio module on the serial interrupt
    #[task(binds = UART1_IRQ, shared = [radio_link, serial_console_writer])]
    fn uart_interrupt(mut ctx: uart_interrupt::Context) {
        ctx.shared.radio_link.lock(|radio| {
            radio.device.update().ok();
        });
    }

    // Radio Flush Task
    #[task(shared = [radio_link], priority = 1)]
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
}
