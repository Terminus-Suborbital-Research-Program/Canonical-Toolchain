use core::fmt::{self, Write as FmtWrite};
use embedded_hal::digital::OutputPin;
use embedded_io::{Read, ReadReady, Write, WriteReady};
use fugit::{HertzU32, RateExtU32};
use heapless::{Deque, String as HString};
use rp235x_hal::uart::{DataBits, Enabled, StopBits, UartConfig, UartDevice, UartPeripheral, ValidUartPinout};

// Clock frequency of the RP235x is 150_000_000Hz
const CLOCK_FREQ: u32 = 150_000_000;

/// A trait to allow reconfiguration of a UART’s baudrate.
/// Consumes and re-enables the UART if the hardware requires it.
pub trait UartReconfigurable: Sized {
    fn set_baudrate(self, baudrate: BaudRate, frequency: HertzU32) -> Result<Self, HC12Error>;
}

impl<D, P> UartReconfigurable for UartPeripheral<Enabled, D, P>
where
    D: UartDevice,
    P: ValidUartPinout<D>,
{
    fn set_baudrate(self, baudrate: BaudRate, frequency: HertzU32) -> Result<Self, HC12Error> {
        let peripheral = self.disable(); // move out of Enabled state
        let baudrate_u32 = baudrate.to_u32();

        // Re-enable with new baud rate
        let peripheral = peripheral
            .enable(
                UartConfig::new(
                    baudrate_u32.Hz(),
                    DataBits::Eight,
                    None,
                    StopBits::One,
                ),
                frequency,
            )
            .map_err(|_err| HC12Error::UartConfigError)?; // map the driver’s error to our own

        Ok(peripheral)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HC12Mode {
    Normal,
    Configuration,
}

/// Supported baud rates for the HC12 module.
#[derive(Debug, Copy, Clone)]
#[repr(u32)]
#[allow(dead_code)]
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

    pub fn from_u32(baud: u32) -> Result<BaudRate, HC12Error> {
        match baud {
            1200 => Ok(BaudRate::B1200),
            2400 => Ok(BaudRate::B2400),
            4800 => Ok(BaudRate::B4800),
            9600 => Ok(BaudRate::B9600),
            19200 => Ok(BaudRate::B19200),
            38400 => Ok(BaudRate::B38400),
            57600 => Ok(BaudRate::B57600),
            115200 => Ok(BaudRate::B115200),
            _ => Err(HC12Error::InvalidBaudrate),
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

/// Error types for the HC12 driver.
#[derive(Debug)]
pub enum HC12Error {
    /// Error toggling the config pin
    ConfigPinError,

    /// Invalid baud rate supplied (or unsupported by the driver)
    InvalidBaudrate,

    /// Failed reconfiguring UART
    UartConfigError,

    /// UART misconfiguration—e.g., missing peripheral
    UartMisconfigured,

    /// Error writing to UART or buffer
    WriteError,

    /// Error reading from UART or buffer
    ReadError,

    /// Outgoing buffer is full
    BufferFull,

    /// Incoming buffer is empty
    BufferEmpty,

    /// Attempted an operation in the wrong mode
    WrongMode,

    /// Specified RF channel is out of allowable range
    InvalidChannel,

    /// Specified RF power level is out of allowable range
    InvalidPower,
}

#[allow(dead_code)]
pub struct HC12<Uart, ConfigPin> {
    uart: Option<Uart>,
    config_pin: ConfigPin,
    mode: HC12Mode,
    baudrate: BaudRate,

    incoming_buffer: Deque<u8, 128>,
    outgoing_buffer: Deque<u8, 128>,
}

impl<Uart, ConfigPin> HC12<Uart, ConfigPin> {
    pub fn bytes_available(&self) -> usize {
        self.incoming_buffer.len()
    }

    /// Clears the incoming buffer
    pub fn clear(&mut self) {
        self.incoming_buffer.clear();
    }

    /// Checks if the incoming buffer contains "OK"
    pub fn check_ok(&self) -> bool {
        self.contains("OK")
    }

    /// Checks if the buffer contains some &str
    pub fn contains(&self, s: &str) -> bool {
        let cloned_buffer = self.incoming_buffer.clone();
        let mut buffer = HString::<128>::new();

        for c in cloned_buffer {
            let _ = buffer.push(c as char);
        }

        buffer.contains(s)
    }
}

impl<Uart, ConfigPin> HC12<Uart, ConfigPin>
where
    Uart: Read + Write + ReadReady + WriteReady + UartReconfigurable,
    ConfigPin: OutputPin,
{
    fn reconfigure_uart_baudrate(
        &mut self,
        baudrate: BaudRate,
        frequency: HertzU32,
    ) -> Result<(), HC12Error> {
        // Take ownership of the UART peripheral
        let uart = self.uart.take().ok_or(HC12Error::UartMisconfigured)?;

        // Change the baudrate by consuming the UART peripheral
        let uart = uart.set_baudrate(baudrate, frequency)?;

        // Return the UART peripheral to the HC12 struct
        self.uart = Some(uart);

        Ok(())
    }

    fn set_hc12_baudrate(&mut self, baudrate: BaudRate) -> Result<(), HC12Error> {
        // Go into configuration mode
        self.set_mode(HC12Mode::Configuration)?;

        // Set the baudrate via "AT+Bxxxx"
        let mut baud_str = HString::<10>::new();
        write!(baud_str, "B{}", baudrate.to_u32()).map_err(|_| HC12Error::WriteError)?;

        self.send_at(&baud_str)?;

        // Exit configuration mode
        self.set_mode(HC12Mode::Normal)?;

        Ok(())
    }

    /// Change the baudrate of the HC12 module AND the local UART peripheral.
    pub fn set_baudrate(
        &mut self,
        baudrate: BaudRate,
        frequency: HertzU32,
    ) -> Result<(), HC12Error> {
        // 1. Configure the remote HC12 module to the new baud rate
        self.set_hc12_baudrate(baudrate)?;

        // 2. Reconfigure our UART peripheral to match
        self.reconfigure_uart_baudrate(baudrate, frequency)?;

        Ok(())
    }

    /// Creates and initializes a new HC12 driver object.
    pub fn new(uart: Uart, mut config_pin: ConfigPin) -> Result<Self, HC12Error> {
        // Attempt to set the HC12 module to normal mode (config pin = LOW)
        config_pin.set_low().map_err(|_| HC12Error::ConfigPinError)?;

        Ok(Self {
            uart: Some(uart),
            config_pin,
            mode: HC12Mode::Normal,
            baudrate: BaudRate::B9600,
            incoming_buffer: Deque::new(),
            outgoing_buffer: Deque::new(),
        })
    }

    fn write_char(&mut self, c: u8) -> Result<(), HC12Error> {
        self.outgoing_buffer
            .push_back(c)
            .map_err(|_| HC12Error::BufferFull)
    }

    fn read_char(&mut self) -> Result<u8, HC12Error> {
        self.incoming_buffer
            .pop_front()
            .ok_or(HC12Error::BufferEmpty)
    }

    pub fn write_str(&mut self, s: &str) -> Result<(), HC12Error> {
        for c in s.bytes() {
            self.outgoing_buffer
                .push_back(c)
                .map_err(|_| HC12Error::BufferFull)?;
        }
        Ok(())
    }

    pub fn read_str(&mut self) -> Result<HString<128>, HC12Error> {
        let mut s = HString::<128>::new();
        while let Some(c) = self.incoming_buffer.pop_front() {
            s.push(c as char).map_err(|_| HC12Error::BufferFull)?;
        }
        Ok(s)
    }

    /// Flushes the outgoing buffer in a non-blocking way, up to `max_bytes` bytes at a time
    /// Returns `Ok(true)` if the buffer is empty after flushing.
    pub fn flush(&mut self, max_bytes: usize) -> Result<bool, HC12Error> {
        let uart = self.uart.as_mut().ok_or(HC12Error::WriteError)?;

        // Single-byte writes. If the driver returns partial writes for single bytes,
        // it’s essentially 0 or 1. We loop until we can’t write or the buffer is empty.
        while let Some(c) = self.outgoing_buffer.pop_front() {
            if uart.write_ready().unwrap_or(false) {
                uart.write(&[c]).map_err(|_| HC12Error::WriteError)?;
            } else {
                // If the UART isn’t ready, push the byte back onto the buffer
                self.outgoing_buffer
                    .push_front(c)
                    .map_err(|_| HC12Error::BufferFull)?;
                break;
            }

            // If we’ve written the maximum number of bytes, stop
            if self.outgoing_buffer.is_empty() || max_bytes > 0 && self.outgoing_buffer.len() >= max_bytes {
                break;
            }
        }

        Ok(self.outgoing_buffer.is_empty())
    }

    /// Reads any available bytes from the UART into the incoming buffer in a non-blocking way.
    pub fn update(&mut self) -> Result<(), HC12Error> {
        let uart = self.uart.as_mut().ok_or(HC12Error::ReadError)?;

        // Single-byte reads. If partial reads are possible, it’s either 0 or 1 byte for us.
        while uart.read_ready().unwrap_or(false) {
            let mut buff = [0u8; 1];
            uart.read(&mut buff).map_err(|_| HC12Error::ReadError)?;
            self.incoming_buffer
                .push_back(buff[0])
                .map_err(|_| HC12Error::BufferFull)?;
        }

        Ok(())
    }

    /// Sets the HC12 module to a mode (Normal or Configuration).
    pub fn set_mode(&mut self, mode: HC12Mode) -> Result<(), HC12Error> {
        match mode {
            HC12Mode::Normal => {
                self.config_pin.set_low().map_err(|_| HC12Error::ConfigPinError)?;
                self.mode = HC12Mode::Normal;
            }
            HC12Mode::Configuration => {
                self.config_pin
                    .set_high()
                    .map_err(|_| HC12Error::ConfigPinError)?;

                // Per HC12 spec: configuration mode is always 9600 baud
                self.reconfigure_uart_baudrate(BaudRate::B9600, CLOCK_FREQ.Hz())?;
                self.mode = HC12Mode::Configuration;
            }
        }
        Ok(())
    }

    /// Sends an AT command to the HC12 module (must be in Configuration mode).
    pub fn send_at(&mut self, command: &str) -> Result<(), HC12Error> {
        if self.mode == HC12Mode::Normal {
            return Err(HC12Error::WrongMode);
        }

        // Clear buffers to ensure fresh data
        self.outgoing_buffer.clear();
        self.incoming_buffer.clear();

        self.write_str("AT+")?;
        self.write_str(command)?;
        self.write_str("\n")?;

        self.flush(128)?;
        Ok(())
    }

    /// Sends an AT check command ("AT") to the HC12 module (must be in Configuration mode).
    pub fn check_at(&mut self) -> Result<(), HC12Error> {
        if self.mode == HC12Mode::Normal {
            return Err(HC12Error::WrongMode);
        }

        self.outgoing_buffer.clear();
        self.incoming_buffer.clear();

        self.write_str("AT\n")?;
        self.flush(128)?;
        Ok(())
    }

    /// Sets the channel of the HC12 module (0..127).
    pub fn set_channel(&mut self, channel: u8) -> Result<(), HC12Error> {
        if channel > 127 {
            return Err(HC12Error::InvalidChannel);
        }

        let mut channel_str = HString::<10>::new();
        write!(channel_str, "C{}", channel).map_err(|_| HC12Error::WriteError)?;
        self.send_at(&channel_str)
    }

    /// Sets the power of the HC12 module (1..8).
    pub fn set_power(&mut self, power: u8) -> Result<(), HC12Error> {
        if !(1..=8).contains(&power) {
            return Err(HC12Error::InvalidPower);
        }

        let mut power_str = HString::<10>::new();
        write!(power_str, "P{}", power).map_err(|_| HC12Error::WriteError)?;
        self.send_at(&power_str)
    }
}