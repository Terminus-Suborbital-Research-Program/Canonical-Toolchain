use embedded_hal::pwm::SetDutyCycle;

static MAX_DUTY: u32 = 8200;
static MIN_DUTY: u32 = 2200;

static EJECTION_ANGLE: u16 = 145;

pub struct Servo<C, P> {
    channel: C,
    _pin: P, // Consume this pin please
}

#[allow(dead_code)]
impl<C, P> Servo<C, P> {
    pub fn new(channel: C, pin: P) -> Self {
        Self { channel, _pin: pin }
    }
}

#[allow(dead_code)]
impl<C, P> Servo<C, P>
where
    C: SetDutyCycle,
{
    pub fn set_angle(&mut self, angle: u16) {
        let duty = ((angle as f32 / 180.0) * (MAX_DUTY - MIN_DUTY) as f32 + MIN_DUTY as f32) as u16;
        self.channel.set_duty_cycle(duty).unwrap();
    }
}
