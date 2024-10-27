
pub mod dynamics{
    use heapless;
    use serde::{Serialize,Deserialize};
    #[derive(Default, Clone, Serialize, Deserialize)]
    pub struct State{
        pub ab:     heapless::Vec<f32, 3>,// Body Acceleration
        pub vl:     heapless::Vec<f32, 3>,// Local Velocity
        pub rl:     heapless::Vec<f32, 3>,// Local Position
        pub dw:     heapless::Vec<f32, 3>,// Body Angular Acceleration
        pub w:      heapless::Vec<f32, 3>,// Body Relative Velocity
        pub e:      heapless::Vec<f32, 3>,// Body Relative Angular Position
        pub rr:     heapless::Vec<f32, 3>,// ECI Position
        pub re:     heapless::Vec<f32, 3>,// ECEF Position
        pub tl2r:   heapless::Vec<f32, 9>,// Local Level to ECI DCM
        pub tr2e:   heapless::Vec<f32, 9>,// ECI to ECEF DCM
        pub lq:     heapless::Vec<f32, 4>,// Body Relative Quaternion 
    }
    impl State{
    }
}
