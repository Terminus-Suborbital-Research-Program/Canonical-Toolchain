use core::fmt::{self, Write as FmtWrite};
use embedded_io::{Read, ReadReady, Write, WriteReady};
use embedded_hal::digital::OutputPin;
use heapless::Deque;

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
}

#[allow(dead_code)]
impl BaudRate {
    pub fn to_u32(&self) -> u32 {
        *self as u32
    }

    pub fn try_from_u32(baud: u32) -> Result<BaudRate, HC12Error> {
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

pub struct HC12<Uart, ConfigPin> {
    uart: Uart,
    config_pin: ConfigPin,
    mode: HC12Mode,
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

impl<Uart, ConfigPin> HC12<Uart, ConfigPin>
where
    Uart: Read + Write + ReadReady + WriteReady,
    ConfigPin: OutputPin
{
    pub fn new(uart: Uart, mut config_pin: ConfigPin) -> Result<Self, HC12Error> {
        // Set the HC12 module to normal mode
        match config_pin.set_low() {
            Ok(_) => Ok(HC12 {
                uart,
                config_pin,
                mode: HC12Mode::Normal,
                incoming_buffer: Deque::new(),
                outgoing_buffer: Deque::new(),
            }),

            Err(_) => Err(HC12Error::ConfigPinError),
        }
    }

    // Write a character to the HC12 module's buffer
    pub fn write_char(&mut self, c: u8) -> Result<(), HC12Error> {
        match self.outgoing_buffer.push_back(c) {
            Ok(_) => Ok(()),
            Err(_) => Err(HC12Error::BufferFull),
        }
    }

    // Read a character from the HC12 module's buffer
    pub fn read_char(&mut self) -> Result<u8, HC12Error> {
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
    pub fn flush(&mut self) -> Result<(), HC12Error> {
        while self.outgoing_buffer.len() > 0 {
            if self.uart.write_ready().unwrap_or(false) {
                match self.outgoing_buffer.pop_front() {
                    Some(c) => match self.uart.write(&[c]) {
                        Ok(_) => (),
                        Err(_) => return Err(HC12Error::WriteError),
                    },
                    None => return Err(HC12Error::BufferEmpty),
                }
            }
        }
        Ok(())
    }

    // Update the incoming buffer, making sure not to block
    pub fn update(&mut self) -> Result<(), HC12Error> {
        while self.uart.read_ready().unwrap_or(false) {
            let mut buf = [0];
            match self.uart.read(&mut buf) {
                Ok(_) => match self.incoming_buffer.push_back(buf[0]) {
                    Ok(_) => (),
                    Err(_) => return Err(HC12Error::BufferFull),
                },
                Err(_) => return Err(HC12Error::ReadError),
            }
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
            return Err(HC12Error::ConfigPinError);
        }
        self.write_str("AT+")?;
        self.write_str(command)?;
        self.write_str("\n")
    }

    // Sends a raw AT check command to the HC12 module
    // Errors if the HC12 module is not in configuration mode
    pub fn check_at(&mut self) -> Result<(), HC12Error> {
        if self.mode == HC12Mode::Normal {
            return Err(HC12Error::ConfigPinError);
        }
        self.write_str("AT\n")
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
impl<Uart, ConfigPin> FmtWrite for HC12<Uart, ConfigPin>
where
    Uart: Read + Write + ReadReady + WriteReady,
    ConfigPin: OutputPin
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        match self.write_str(s) {
            Ok(_) => Ok(()),
            Err(_) => Err(fmt::Error),
        }
    }
}