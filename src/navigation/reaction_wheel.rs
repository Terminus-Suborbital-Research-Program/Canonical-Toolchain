use crate::utils::type_conversion::f32_to_u32;
#[derive(Clone)]
pub struct ReactionWheel{
    torque_constant: f32,    // "K_T"
    velocity_constant: f32,   // "K_V"
    voltage: f32,
    resistance: f32,
    max_velocity: f32
}

impl ReactionWheel{
    pub fn initialize(kv_rpm: f32, voltage: f32, resistance: f32)->Self{
        ReactionWheel{
            torque_constant: 1.0/kv_rpm,
            velocity_constant: kv_rpm,
            voltage: voltage,
            resistance: resistance, // Milliohms
            max_velocity: kv_rpm * voltage,
        }
    }

    // Get desired duty from desired velocity (Based on Max Velocity)
    // If velocity is negative, should specify to swap directions
    pub fn get_pwm(&self, velocity_desired: f32)->u32{
        let duty = (velocity_desired / self.max_velocity);
        let duty_rounded = f32_to_u32(duty).0;
        return duty_rounded;
    }
}