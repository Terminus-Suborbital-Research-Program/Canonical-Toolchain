use core::fmt::Write;

use embedded_io::{self as io, ErrorType};
use embedded_hal::digital::OutputPin;
use heapless::String;

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HC12Mode {
    Normal,
    Configuration,
}

#[allow(dead_code)]
pub struct HC12<Uart, ConfigPin> {
    uart: Uart,
    config_pin: ConfigPin,
    mode: HC12Mode,
}

#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
#[repr(u32)]
pub enum BaudRate {
    B1200 = 1200,
    B2400 = 2400,
    B4800 = 4800,
    B9600 = 9600,
    B19200 = 19200,
    B38400 = 38400,
    B57600 = 57600,
    B115200 = 115200,
}


#[derive(Debug)]
pub enum ConfigurationError {
    BadBaudrate,
    WrongMode,
}

impl BaudRate {
    pub fn to_u32(&self) -> u32 {
        *self as u32
    }

    pub fn try_from_u32(value: u32) -> Result<Self, ConfigurationError> {
        match value {
            1200 => Ok(BaudRate::B1200),
            2400 => Ok(BaudRate::B2400),
            4800 => Ok(BaudRate::B4800),
            9600 => Ok(BaudRate::B9600),
            19200 => Ok(BaudRate::B19200),
            38400 => Ok(BaudRate::B38400),
            57600 => Ok(BaudRate::B57600),
            115200 => Ok(BaudRate::B115200),
            _ => Err(ConfigurationError::BadBaudrate),
        }
    }

    // Convert to string
    fn to_string(&self) -> String::<6> {
        let mut string = String::<6>::new();
        match self {
            BaudRate::B1200 => string.push_str("1200").unwrap(),
            BaudRate::B2400 => string.push_str("2400").unwrap(),
            BaudRate::B4800 => string.push_str("4800").unwrap(),
            BaudRate::B9600 => string.push_str("9600").unwrap(),
            BaudRate::B19200 => string.push_str("19200").unwrap(),
            BaudRate::B38400 => string.push_str("38400").unwrap(),
            BaudRate::B57600 => string.push_str("57600").unwrap(),
            BaudRate::B115200 => string.push_str("115200").unwrap(),
        }

        string
    }
}

pub struct HC12WrongModeError;

pub enum HC12Error<UartError> {
    Uart(UartError),
    WrongMode(HC12WrongModeError),
}

#[allow(dead_code)]
impl<Uart, PIN> HC12<Uart, PIN>
where
    Uart: io::Read + io::Write + io::ReadReady + io::WriteReady,
    PIN: OutputPin,
{
    pub fn new(uart: Uart, mut config_pin: PIN) -> Result<Self, PIN::Error> {
        let res = config_pin.set_high();

        match res {
            Ok(_) => {
                let hc12: HC12<Uart, PIN> = HC12 {
                    uart,
                    config_pin,
                    mode: HC12Mode::Normal,
                };

                Ok(hc12)
            },

            Err(e) => Err(e),
        }
    }

    // Helper function, ok if is in configuration mode, error if not
    pub fn check_configuration_mode(&self) -> Result<(), HC12WrongModeError> {
        if self.mode == HC12Mode::Configuration {
            Ok(())
        } else {
            Err(HC12WrongModeError)
        }
    }

    // Helper function, checks if a sent command was successful by two chars OK
    pub fn check_command_success(&mut self) -> Result<bool, Uart::Error> {
        let mut buffer = [0u8; 2];
        self.read_until(&mut buffer, 2, 100000)?;

        if buffer == [b'O', b'K'] {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    // Helper function, iterativly reads until a set number of characters, or a timeout value is reached
    fn read_until(&mut self, buffer: &mut [u8], until: usize, timeout: u32) -> Result<usize, Uart::Error> {
        let mut read = 0;
        let mut timeout = timeout;

        while read < until && timeout > 0 {
            if self.uart.read_ready()? {
                let bytes_read = self.uart.read(&mut buffer[read..until])?;
                read += bytes_read;
            } else {
                timeout -= 1;
            }
        }

        Ok(read)
    }

    // Flush the output buffer
    pub fn flush(&mut self) -> Result<(), Uart::Error> {
        self.uart.flush()
    }

    // Clear the input buffer
    pub fn clear(&mut self) -> Result<(), Uart::Error> {
        while self.uart.read_ready()? {
            let mut buffer = [0u8; 1];
            self.uart.read(&mut buffer)?;
        }

        Ok(())
    }

    pub fn set_mode(&mut self, mode: HC12Mode) -> Result<(), PIN::Error> {
        let result = match mode {
            HC12Mode::Normal => self.config_pin.set_high(),
            HC12Mode::Configuration => self.config_pin.set_low(),
        };

        if result.is_ok() {
            self.mode = mode;
        }

        result
    }

    // Set baudrate, only in configuration mode naturally
    pub fn set_baudrate(&mut self, baudrate: BaudRate) -> Result<(), ConfigurationError> {
        if self.check_configuration_mode().is_err() {
            return Err(ConfigurationError::WrongMode);
        }

        //let baudrate = baudrate.to_u32();

        // Command is AT+Bxxxx
        // These should be garunteed to be valid, as they're all from hardcodeds
        let mut command_string = String::<12>::try_from("AT+B").unwrap();
        command_string.push_str(&baudrate.to_string()).unwrap();
        command_string.push_str("\n").unwrap();

        // Send command
        let command_bytes = command_string.as_bytes();
        // Fine to send this blocking, as we're in configuration mode and it's only a few bytes
        self.uart.write_all(command_bytes).unwrap();

        // Check if command was successful
        let res = match self.check_command_success() {
            Ok(true) => Ok(()),
            Ok(false) => Err(ConfigurationError::BadBaudrate),
            Err(_) => Err(ConfigurationError::WrongMode),
        };

        res
    }

    // Set power, 1-8 is valid
    pub fn set_power(&mut self, power: u8) -> Result<(), ConfigurationError> {
        if self.check_configuration_mode().is_err() {
            return Err(ConfigurationError::WrongMode);
        }

        if power > 8 {
            return Err(ConfigurationError::BadBaudrate);
        }

        // Command is AT+Px
        let mut command_string = String::<12>::try_from("AT+P").unwrap();
        write!(command_string, "{}", power).unwrap();
        command_string.push_str("\n").unwrap();

        // Send command
        let command_bytes = command_string.as_bytes();
        // Fine to send this blocking, as we're in configuration mode and it's only a few bytes
        self.uart.write_all(command_bytes).unwrap();

        // Check if command was successful
        let res = match self.check_command_success() {
            Ok(true) => Ok(()),
            Ok(false) => Err(ConfigurationError::BadBaudrate),
            Err(_) => Err(ConfigurationError::WrongMode),
        };

        res
    }

    // Set channel, 001-127 are supported
    pub fn set_channel(&mut self, channel: u8) -> Result<(), ConfigurationError> {
        if self.check_configuration_mode().is_err() {
            return Err(ConfigurationError::WrongMode);
        }

        if channel > 127 {
            return Err(ConfigurationError::BadBaudrate);
        }

        // Command is AT+Cxxx
        let mut command_string = String::<12>::try_from("AT+C").unwrap();
        write!(command_string, "{:03}", channel).unwrap();
        command_string.push_str("\n").unwrap();

        // Send command
        let command_bytes = command_string.as_bytes();
        // Fine to send this blocking, as we're in configuration mode and it's only a few bytes
        self.uart.write_all(command_bytes).unwrap();

        // Check if command was successful
        let res = match self.check_command_success() {
            Ok(true) => Ok(()),
            Ok(false) => Err(ConfigurationError::BadBaudrate),
            Err(_) => Err(ConfigurationError::WrongMode),
        };

        res
    }

    pub fn get_mode(&self) -> HC12Mode {
        self.mode
    }
}

// Re-implimentation of embedded_io traits, so we can use it as a drop-in Read+Write device
impl<Uart, Pin> ErrorType for HC12<Uart, Pin>
where Uart: io::Read + io::Write + io::ReadReady + io::WriteReady,
      Pin: OutputPin,
{
    type Error = Uart::Error;
}

// Read
impl<Uart, Pin> io::Read for HC12<Uart, Pin>
where Uart: io::Read + io::Write + io::ReadReady + io::WriteReady,
      Pin: OutputPin,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.uart.read(buf)
    }
}

// Write
impl<Uart, Pin> io::Write for HC12<Uart, Pin>
where Uart: io::Read + io::Write + io::ReadReady + io::WriteReady,
      Pin: OutputPin,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.uart.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.uart.flush()
    }
}

// ReadReady
impl<Uart, Pin> io::ReadReady for HC12<Uart, Pin>
where Uart: io::Read + io::Write + io::ReadReady + io::WriteReady,
      Pin: OutputPin,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        self.uart.read_ready()
    }
}

// WriteReady
impl<Uart, Pin> io::WriteReady for HC12<Uart, Pin>
where Uart: io::Read + io::Write + io::ReadReady + io::WriteReady,
      Pin: OutputPin,
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        self.uart.write_ready()
    }
}