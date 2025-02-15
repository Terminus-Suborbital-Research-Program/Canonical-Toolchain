use rp235x_hal::{gpio, pac::I2C1, I2C};

pub type I2CMainBus = I2C<
    I2C1,
    (
        gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionI2c, gpio::PullUp>,
        gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionI2c, gpio::PullUp>,
    ),
>;
