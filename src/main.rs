#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal as hal;



// Some things we need
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

// Some things we need
use hal::clocks::Clock;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};
use hal::uart::{DataBits, StopBits, UartConfig};
use embedded_io::Write;
use bno055::mint;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the rp235x peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);


    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // uart declaration
    let mut uart = hal::uart::UartPeripheral::new(
        // using the first UART channel (pins 0 and 1)
        pac.UART0,
        // pins allocation for UART
        (pins.gpio0.into_function(), pins.gpio1.into_function()),
        &mut pac.RESETS,
    )
    .enable(
        // these configs we'll be using on the serial receiver.
        UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
        clocks.peripheral_clock.freq(),
    )
    .unwrap();

    uart.write_fmt(format_args!("Welcome to Terminus!\n"));

    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio14.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio15.reconfigure();
    let mut i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut imu = bno055::Bno055::new(i2c).with_alternative_address();
    imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut timer).expect("Could not set IMU Mode...");
    imu.set_power_mode(bno055::BNO055PowerMode::NORMAL);
    let mut seconds = 0;
    let mut led_pin = pins.gpio25.into_push_pull_output();
    
    let mut status = imu.get_calibration_status().unwrap();
    uart.write_fmt(format_args!("Calibration status: {:?}\n", status));

    uart.write_fmt(format_args!("Beginning Calibration\n"));
    while !imu.is_fully_calibrated().unwrap() {
        status = imu.get_calibration_status().unwrap();
        timer.delay_ms(1000);
        uart.write_fmt(format_args!("Calibration status: {:?}\n", status));
    }

    let calib = imu.calibration_profile(&mut timer).unwrap();
    imu.set_calibration_profile(calib, &mut timer).unwrap();
    let mut quat: mint::Quaternion<f32>  = mint::Quaternion::<f32>::from([0.0, 0.0, 0.0, 0.0]);;
    loop {
        match imu.quaternion(){
            Ok(val) =>{
                quat=val
            },
            Err(error) => {
                uart.write_fmt(format_args!("Quaternion failed.\n")).unwrap();
                panic!("Problem...")}
        };

        // let euler_angle = match imu.euler_angles(){
        //     Ok(quat) =>{quat},
        //     Err(error) => {
        //         uart.write_fmt(format_args!("Eulers failed.\n")).unwrap();
        //         panic!("Problem...")}
        // };


        uart.write_fmt(format_args!("Quat: {:},{:},{:},{:},{:.2}\n", seconds, quat.v.x, quat.v.y, quat.v.z, quat.s)).unwrap();

        led_pin.set_high().unwrap();

        // timer.delay_ms(1000);
        seconds+=1;
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Terminus"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
