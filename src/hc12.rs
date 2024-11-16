use core::fmt::{self, Write as FmtWrite};
use embedded_io::{self as io, ErrorType};
use embedded_hal::digital::OutputPin;
use heapless::String;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HC12Mode {
    Normal,
    Configuration,
}

pub struct HC12<Uart, ConfigPin> {
    uart: Uart,
    config_pin: ConfigPin,
    mode: HC12Mode,
}

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

impl BaudRate {
    pub fn to_u32(&self) -> u32 {
        *self as u32
    }

    pub fn try_from_u32(value: u32) -> Result<Self, HC12Error<(), ()>> {
        match value {
            1200 => Ok(BaudRate::B1200),
            2400 => Ok(BaudRate::B2400),
            4800 => Ok(BaudRate::B4800),
            9600 => Ok(BaudRate::B9600),
            19200 => Ok(BaudRate::B19200),
            38400 => Ok(BaudRate::B38400),
            57600 => Ok(BaudRate::B57600),
            115200 => Ok(BaudRate::B115200),
            _ => Err(HC12Error::BadBaudrate),
        }
    }
}

impl fmt::Display for BaudRate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let baud_str = match self {
            BaudRate::B1200 => "1200",
            BaudRate::B2400 => "2400",
            BaudRate::B4800 => "4800",
            BaudRate::B9600 => "9600",
            BaudRate::B19200 => "19200",
            BaudRate::B38400 => "38400",
            BaudRate::B57600 => "57600",
            BaudRate::B115200 => "115200",
        };
        f.write_str(baud_str)
    }
}

#[derive(Debug)]
pub enum HC12Error<UartError, PinError> {
    Uart(UartError),
    Pin(PinError),
    WrongMode,
    BadBaudrate,
    InvalidPower,
    InvalidChannel,
    CommandFailed,
    BufferOverflow,
    Timeout,
}

impl<Uart, Pin> HC12<Uart, Pin>
where
    Uart: io::Read + io::Write + io::ReadReady + io::WriteReady,
    Pin: OutputPin,
{
    pub fn new(uart: Uart, mut config_pin: Pin) -> Result<Self, HC12Error<Uart::Error, Pin::Error>> {
        config_pin.set_high().map_err(HC12Error::Pin)?;

        Ok(HC12 {
            uart,
            config_pin,
            mode: HC12Mode::Normal,
        })
    }

    pub fn check_configuration_mode(&self) -> Result<(), HC12Error<Uart::Error, Pin::Error>> {
        if self.mode == HC12Mode::Configuration {
            Ok(())
        } else {
            Err(HC12Error::WrongMode)
        }
    }

    pub fn check_command_success(&mut self) -> Result<bool, HC12Error<Uart::Error, Pin::Error>> {
        let mut buffer = [0u8; 2];
        self.read_until(&mut buffer, 2, 100_000)?;

        Ok(buffer == [b'O', b'K'])
    }

    fn read_until(
        &mut self,
        buffer: &mut [u8],
        until: usize,
        mut timeout: u32,
    ) -> Result<usize, HC12Error<Uart::Error, Pin::Error>> {
        let mut read = 0;

        while read < until && timeout > 0 {
            if self.uart.read_ready().map_err(HC12Error::Uart)? {
                let bytes_read = self.uart.read(&mut buffer[read..until]).map_err(HC12Error::Uart)?;
                read += bytes_read;
            } else {
                timeout -= 1;
            }
        }

        if read == until {
            Ok(read)
        } else {
            Err(HC12Error::Timeout)
        }
    }

    pub fn flush(&mut self) -> Result<(), HC12Error<Uart::Error, Pin::Error>> {
        self.uart.flush().map_err(HC12Error::Uart)
    }

    pub fn clear(&mut self) -> Result<(), HC12Error<Uart::Error, Pin::Error>> {
        while self.uart.read_ready().map_err(HC12Error::Uart)? {
            let mut buffer = [0u8; 1];
            self.uart.read(&mut buffer).map_err(HC12Error::Uart)?;
        }

        Ok(())
    }

    pub fn set_mode(&mut self, mode: HC12Mode) -> Result<(), HC12Error<Uart::Error, Pin::Error>> {
        match mode {
            HC12Mode::Normal => self.config_pin.set_high().map_err(HC12Error::Pin)?,
            HC12Mode::Configuration => self.config_pin.set_low().map_err(HC12Error::Pin)?,
        }

        self.mode = mode;
        Ok(())
    }

    pub fn set_baudrate(&mut self, baudrate: BaudRate) -> Result<(), HC12Error<Uart::Error, Pin::Error>> {
        self.check_configuration_mode()?;

        let mut command_string = String::<12>::new();
        write!(command_string, "AT+B{}\n", baudrate).map_err(|_| HC12Error::BufferOverflow)?;

        self.uart
            .write_all(command_string.as_bytes())
            .map_err(HC12Error::Uart)?;

        if self.check_command_success()? {
            Ok(())
        } else {
            Err(HC12Error::BadBaudrate)
        }
    }

    pub fn set_power(&mut self, power: u8) -> Result<(), HC12Error<Uart::Error, Pin::Error>> {
        self.check_configuration_mode()?;

        if power == 0 || power > 8 {
            return Err(HC12Error::InvalidPower);
        }

        let mut command_string = String::<12>::new();
        write!(command_string, "AT+P{}\n", power).map_err(|_| HC12Error::BufferOverflow)?;

        self.uart
            .write_all(command_string.as_bytes())
            .map_err(HC12Error::Uart)?;

        if self.check_command_success()? {
            Ok(())
        } else {
            Err(HC12Error::CommandFailed)
        }
    }

    pub fn set_channel(&mut self, channel: u8) -> Result<(), HC12Error<Uart::Error, Pin::Error>> {
        self.check_configuration_mode()?;

        if channel == 0 || channel > 127 {
            return Err(HC12Error::InvalidChannel);
        }

        let mut command_string = String::<12>::new();
        write!(command_string, "AT+C{:03}\n", channel).map_err(|_| HC12Error::BufferOverflow)?;

        self.uart
            .write_all(command_string.as_bytes())
            .map_err(HC12Error::Uart)?;

        if self.check_command_success()? {
            Ok(())
        } else {
            Err(HC12Error::CommandFailed)
        }
    }

    pub fn get_mode(&self) -> HC12Mode {
        self.mode
    }
}

// Implement embedded_io::Error for HC12Error
impl<UartError, PinError> io::Error for HC12Error<UartError, PinError>
where
    UartError: fmt::Debug,
    PinError: fmt::Debug,
{
    fn kind(&self) -> io::ErrorKind {
        match self {
            HC12Error::Uart(_) => io::ErrorKind::Other,
            HC12Error::Pin(_) => io::ErrorKind::Other,
            HC12Error::WrongMode => io::ErrorKind::InvalidInput,
            HC12Error::BadBaudrate => io::ErrorKind::InvalidInput,
            HC12Error::InvalidPower => io::ErrorKind::InvalidInput,
            HC12Error::InvalidChannel => io::ErrorKind::InvalidInput,
            HC12Error::CommandFailed => io::ErrorKind::Other,
            HC12Error::BufferOverflow => io::ErrorKind::Other,
            HC12Error::Timeout => io::ErrorKind::TimedOut,
        }
    }
}

// Implement embedded_io traits
impl<Uart, Pin> ErrorType for HC12<Uart, Pin>
where
    Uart: io::Read + io::Write + io::ReadReady + io::WriteReady + ErrorType,
    Pin: OutputPin,
    Uart::Error: fmt::Debug,
    Pin::Error: fmt::Debug,
{
    type Error = HC12Error<Uart::Error, Pin::Error>;
}

impl<Uart, Pin> io::Read for HC12<Uart, Pin>
where
    Uart: io::Read + io::Write + io::ReadReady + io::WriteReady,
    Pin: OutputPin,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.uart.read(buf).map_err(HC12Error::Uart)
    }
}

impl<Uart, Pin> io::Write for HC12<Uart, Pin>
where
    Uart: io::Read + io::Write + io::ReadReady + io::WriteReady,
    Pin: OutputPin,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.uart.write(buf).map_err(HC12Error::Uart)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.uart.flush().map_err(HC12Error::Uart)
    }
}

impl<Uart, Pin> io::ReadReady for HC12<Uart, Pin>
where
    Uart: io::Read + io::Write + io::ReadReady + io::WriteReady,
    Pin: OutputPin,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        self.uart.read_ready().map_err(HC12Error::Uart)
    }
}

impl<Uart, Pin> io::WriteReady for HC12<Uart, Pin>
where
    Uart: io::Read + io::Write + io::ReadReady + io::WriteReady,
    Pin: OutputPin,
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        self.uart.write_ready().map_err(HC12Error::Uart)
    }
}
