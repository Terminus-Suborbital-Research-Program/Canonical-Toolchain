// Motor Configuartion
pub type PWM2a = rp235x_hal::pwm::Channel<rp235x_hal::pwm::Slice<rp235x_hal::pwm::Pwm2, rp235x_hal::pwm::FreeRunning>, rp235x_hal::pwm::A>;

pub mod motor;
pub mod servo;