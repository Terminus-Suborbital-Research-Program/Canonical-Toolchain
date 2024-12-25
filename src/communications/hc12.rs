use core::fmt::{self, Write as FmtWrite};
use embedded_hal::digital::OutputPin;
use embedded_io::{Read, ReadReady, Write, WriteReady};
use heapless::Deque;
use fugit::{HertzU32, RateExtU32};
use rp235x_hal::uart::{DataBits, Enabled, StopBits, UartConfig, UartDevice, UartPeripheral, ValidUartPinout};

// Clock frequency of the RP235x is 150000000Hz
const CLOCK_FREQ: u32 = 150_000_000;

#[allow(dead_code)]
pub trait BaudRateChangeable: Sized {
    fn set_baudrate(self, baudrate: BaudRate, frequency: HertzU32) -> Result<Self, HC12Error>;
}

impl<D: UartDevice, P: ValidUartPinout<D>> BaudRateChangeable for UartPeripheral<Enabled, D, P>
{
    fn set_baudrate(self, baudrate: BaudRate, frequency: HertzU32) -> Result<Self, HC12Error> {
        let peripheral = self.disable();
        let baudrate_u32 = baudrate.to_u32();
        let peripheral = peripheral.enable(
            UartConfig::new(
                baudrate_u32.Hz(),
                DataBits::Eight,
                None,
                StopBits::One,
            ), 
            frequency
        ).unwrap();
        
        Ok(peripheral)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HC12Mode {
    Normal,
    Configuration,
}

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

#[derive(Debug)]
pub enum HC12Error {
    ConfigPinError,
    BadBaudrate,
    WriteError,
    ReadError,
    BufferFull,
    BufferEmpty,
    UartMisconfig,
    WrongMode,
}

#[allow(dead_code)]
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

    // Cleares the incoming buffer
    pub fn clear(&mut self) {
        self.incoming_buffer.clear();
    }

    // Checks if the buffer contains "OK"
    pub fn check_ok(&self) -> bool {
        let cloned_buffer = self.incoming_buffer.clone();
        let mut buffer = heapless::String::<128>::new();
        for c in cloned_buffer {
            buffer.push(c as char).unwrap();
        }
        buffer.contains("OK")
    }
}

// Allows changing the baudrate of the HC12 module, not possible without the BaudRateChangeable trait
// Dunno why that trait isn't part of embedded_hal/embedded_io
#[allow(dead_code)]
impl<Uart, ConfigPin> HC12<Uart, ConfigPin>
where
    Uart: Read + Write + ReadReady + WriteReady + BaudRateChangeable,
    ConfigPin: OutputPin,
{
    fn set_uart_baudrate(&mut self, baudrate: BaudRate, frequency: HertzU32) -> Result<(), HC12Error> {
        // Take ownership of the UART peripheral
        let uart = self.uart.take().ok_or(HC12Error::UartMisconfig)?;

        // Change the baudrate by consuming the UART peripheral
        let uart = uart.set_baudrate(baudrate, frequency)?;

        // Return the UART peripheral to the HC12 struct
        self.uart = Some(uart);

        Ok(())
    }

    // Change the baudrate of the HC12 module
    fn set_hc12_baudrate(&mut self, baudrate: BaudRate) -> Result<(), HC12Error> {
        // Go into configuration mode
        self.set_mode(HC12Mode::Configuration)?;

        // Set the baudrate via AT+Bxxxx
        let mut baud_str = heapless::String::<10>::new();
        write!(baud_str, "B{}", baudrate.to_u32()).unwrap();
        self.send_at(&baud_str)?;

        // Exit configuration mode
        self.set_mode(HC12Mode::Normal)?;

        Ok(())
    }

    // Change the baudrate of the HC12 module and the UART peripheral
    pub fn set_baudrate(&mut self, baudrate: BaudRate, frequency: HertzU32) -> Result<(), HC12Error> {
        // Set the baudrate of the HC12 module
        self.set_hc12_baudrate(baudrate)?;

        // Set the baudrate of the UART peripheral
        self.set_uart_baudrate(baudrate, frequency)?;

        Ok(())
    }

    pub fn new(uart: Uart, mut config_pin: ConfigPin) -> Result<Self, HC12Error> {
        // Set the HC12 module to normal mode
        match config_pin.set_low() {
            Ok(_) => Ok(HC12 {
                uart: Some(uart),
                config_pin,
                mode: HC12Mode::Normal,
                baudrate: BaudRate::B9600,
                incoming_buffer: Deque::new(),
                outgoing_buffer: Deque::new(),
            }),

            Err(_) => Err(HC12Error::ConfigPinError),
        }
    }

    // Write a character to the HC12 module's buffer
    fn write_char(&mut self, c: u8) -> Result<(), HC12Error> {
        match self.outgoing_buffer.push_back(c) {
            Ok(_) => Ok(()),
            Err(_) => Err(HC12Error::BufferFull),
        }
    }

    // Read a character from the HC12 module's buffer
    fn read_char(&mut self) -> Result<u8, HC12Error> {
        match self.incoming_buffer.pop_front() {
            Some(c) => Ok(c),
            None => Err(HC12Error::BufferEmpty),
        }
    }

    // Write a string to the HC12 module's buffer
    pub fn write_str(&mut self, s: &str) -> Result<(), HC12Error> {
        for c in s.bytes() {
            match self.outgoing_buffer.push_back(c) {
                Ok(_) => (),
                Err(_) => return Err(HC12Error::BufferFull),
            }
        }
        Ok(())
    }

    // Read a string from the HC12 module's buffer
    pub fn read_str(&mut self) -> Result<heapless::String<128>, HC12Error> {
        let mut s = heapless::String::new();
        loop {
            match self.incoming_buffer.pop_front() {
                Some(c) => match s.push(c as char) {
                    Ok(_) => (),
                    Err(_) => return Err(HC12Error::BufferFull),
                },
                None => break,
            }
        }
        Ok(s)
    }

    // Flush the outgoing buffer, making sure not to block
    // Returns true if the buffer is empty
    pub fn flush(&mut self) -> Result<bool, HC12Error> {
        match self.uart.as_mut() {
            Some(uart) => {
                while self.outgoing_buffer.len() > 0 && uart.write_ready().unwrap_or(false) {
                    match self.outgoing_buffer.pop_front() {
                        Some(c) => match uart.write(&[c]) {
                            Ok(_) => (),
                            Err(_) => return Err(HC12Error::WriteError),
                        },
                        None => return Err(HC12Error::BufferEmpty),
                    }
                }
            }

            None => return Err(HC12Error::WriteError),
        }

        Ok(self.outgoing_buffer.len() == 0)
    }

    // Update the incoming buffer, making sure not to block
    pub fn update(&mut self) -> Result<(), HC12Error> {
        match self.uart.as_mut() {
            Some(uart) => {
                while uart.read_ready().unwrap_or(false) {
                    let mut buff: [u8; 1] = [0];
                    match uart.read(&mut buff) {
                        Ok(_c) => match self.incoming_buffer.push_back(buff[0]) {
                            Ok(_) => (),
                            Err(_) => return Err(HC12Error::BufferFull),
                        },
                        Err(_) => return Err(HC12Error::ReadError),
                    }
                }
            }

            None => return Err(HC12Error::ReadError),
        }

        Ok(())
    }

    // Set the HC12 module to a mode
    pub fn set_mode(&mut self, mode: HC12Mode) -> Result<(), HC12Error> {
        match mode {
            HC12Mode::Normal => match self.config_pin.set_low() {
                Ok(_) => {
                    self.mode = HC12Mode::Normal;
                    Ok(())
                }
                Err(_) => Err(HC12Error::ConfigPinError),
            },
            HC12Mode::Configuration => match self.config_pin.set_high() {
                Ok(_) => {
                    // If we go into configuration mode, we need to set our uart baudrate to 9600
                    self.set_uart_baudrate(BaudRate::B9600, CLOCK_FREQ.Hz())?;
                    self.mode = HC12Mode::Configuration;
                    Ok(())
                }
                Err(_) => Err(HC12Error::ConfigPinError),
            },
        }
    }

    // Sends an AT command to the HC12 module
    // Errors if the HC12 module is not in configuration mode
    // Prepends the AT+ prefix to the command, and appends a newline
    pub fn send_at(&mut self, command: &str) -> Result<(), HC12Error> {
        if self.mode == HC12Mode::Normal {
            return Err(HC12Error::WrongMode);
        }

        // Clear out the outgoing and incoming buffers
        self.outgoing_buffer.clear();
        self.incoming_buffer.clear();

        self.write_str("AT+")?;
        self.write_str(command)?;
        self.write_str("\n")?;

        self.flush()?;

        Ok(())
    }

    // Sends a raw AT check command to the HC12 module
    // Errors if the HC12 module is not in configuration mode
    pub fn check_at(&mut self) -> Result<(), HC12Error> {
        if self.mode == HC12Mode::Normal {
            return Err(HC12Error::WrongMode);
        }
        self.outgoing_buffer.clear();
        self.incoming_buffer.clear();
        self.write_str("AT\n")?;
        self.flush()?;

        Ok(())
    }

    // Sets the channel of the HC12 module
    pub fn set_channel(&mut self, channel: u8) -> Result<(), HC12Error> {
        if channel > 127 {
            return Err(HC12Error::BadBaudrate);
        }
        let mut channel_str = heapless::String::<10>::new();
        write!(channel_str, "C{}", channel).unwrap();
        self.send_at(&channel_str)
    }

    // Sets the power of the HC12 module
    // Ranges from 1-8, inclusive
    pub fn set_power(&mut self, power: u8) -> Result<(), HC12Error> {
        if power < 1 || power > 8 {
            return Err(HC12Error::BadBaudrate);
        }
        let mut power_str = heapless::String::<10>::new();
        write!(power_str, "P{}", power).unwrap();
        self.send_at(&power_str)
    }
}

// Impliment the Write trait for HC12 so we can use the write! macro
// Uses the write_char method to write to the HC12 module's buffer
// impl<Uart, ConfigPin> FmtWrite for HC12<Uart, ConfigPin>
// where
//     Uart: Read + Write + ReadReady + WriteReady,
//     ConfigPin: OutputPin,
// {
//     fn write_str(&mut self, s: &str) -> fmt::Result {
//         match self.write_str(s) {
//             Ok(_) => Ok(()),
//             Err(_) => Err(fmt::Error),
//         }
//     }
// }
