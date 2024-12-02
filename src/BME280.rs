use embedded_hal::i2c::I2c as I2cTrait;
use core::cell::RefCell;

#[repr(C)]
pub enum BMEMode {
    Sleep = 0b00,
    Forced = 0b01,
    Normal = 0b11,
}

pub struct BME280<I2C> {
    i2c: RefCell<I2C>,
    address: u8,
    mode: BMEMode,
}


#[allow(dead_code)]
impl<I2C> BME280<I2C>
where
    I2C: I2cTrait,
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        BME280 {
            i2c: RefCell::new(i2c),
            address,
            mode: BMEMode::Sleep,
        }
    }
}