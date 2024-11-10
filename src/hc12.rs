use core::{fmt::Error, str::FromStr};

use embedded_hal::digital::OutputPin;
use rp235x_hal::{gpio::{self, DynPinId}, uart::UartPeripheral};
use heapless::String;
use usbd_serial::embedded_io::Write;

pub type UARTBus = UartPeripheral<rp235x_hal::uart::Enabled, rp235x_hal::pac::UART0, (gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>, gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>)>;
pub type GenericGPIOOutputPin = gpio::Pin<DynPinId, gpio::FunctionSioOutput, gpio::DynPullType>;

#[derive(Debug)]
struct ATProgrammingError {
    message: String<64>,
}

#[derive(Debug)]
struct ATNoResponseError;

#[derive(Debug)]
enum ATError {
    Programming(ATProgrammingError),
    NoResponse(ATNoResponseError),
}

#[derive(Debug, Clone)]
struct IncorrectModeError;

#[derive(Debug)]
pub enum Mode {
    FU1,
    FU2,
    FU3,
    AT,
}

pub enum HostBaudrate {
    B1200,
    B2400,
    B4800,
    B9600,
    B19200,
    B38400,
    B57600,
    B115200,
}

impl HostBaudrate {
    fn to_string(&self) -> String<6> {
        match self {
            HostBaudrate::B1200 => String::from_str("1200").unwrap(),
            HostBaudrate::B2400 => String::from_str("2400").unwrap(),
            HostBaudrate::B4800 => String::from_str("4800").unwrap(),
            HostBaudrate::B9600 => String::from_str("9600").unwrap(),
            HostBaudrate::B19200 => String::from_str("19200").unwrap(),
            HostBaudrate::B38400 => String::from_str("38400").unwrap(),
            HostBaudrate::B57600 => String::from_str("57600").unwrap(),
            HostBaudrate::B115200 => String::from_str("115200").unwrap(),
        }
    }

    fn to_u32(&self) -> u32 {
        match self {
            HostBaudrate::B1200 => 1200,
            HostBaudrate::B2400 => 2400,
            HostBaudrate::B4800 => 4800,
            HostBaudrate::B9600 => 9600,
            HostBaudrate::B19200 => 19200,
            HostBaudrate::B38400 => 38400,
            HostBaudrate::B57600 => 57600,
            HostBaudrate::B115200 => 115200,
        }
    }
}

pub struct HC12 {
    uart: UARTBus,
    configure_pin: GenericGPIOOutputPin,
    mode: Mode,
}

impl HC12 {
    pub fn new(uart: UARTBus, configure_pin: GenericGPIOOutputPin) -> HC12 {
        HC12 {
            uart,
            configure_pin,
            mode: Mode::FU3,
        }
    }

    pub fn send_string(&mut self, data: &str) {
        self.uart.write_all(data.as_bytes());
    }

    pub fn enter_configuration_mode(&mut self) -> Result<(), ATError> {
        // Set pin low to enter configuration mode
        self.configure_pin.set_low().map_err(|_| ATError::Programming(ATProgrammingError {
            message: String::from_str("Failed to set configure pin low").unwrap(),
        }))?;

        // Wait for the module to enter configuration mode
        // This is a blocking operation
        // The module will respond with "OK"
        let mut response = [0u8; 3];
        self.uart.read_full_blocking(&mut response).map_err(|_| ATError::NoResponse(ATNoResponseError))?;

        // Check if the response is "OK"
        if response != [b'O', b'K', b'\r'] {
            return Err(ATError::Programming(ATProgrammingError {
                message: String::from_str("Did not receive OK").unwrap(),
            }));
        }

        // Set mode to AT
        self.mode = Mode::AT;

        Ok(())
    }
}