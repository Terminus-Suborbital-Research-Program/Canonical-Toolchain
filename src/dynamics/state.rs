use heapless;
use serde::{Serialize,Deserialize};
// May want to utilize the mint versions of vectors. Not sure yet.
#[derive(Clone, Serialize, Deserialize)]
pub struct State{
    // pub t:          u32,
    pub ab:     [f32; 3],// Body Acceleration
    // pub vl:     [f32; 3],// Local Velocity
    // pub rl:     [f32; 3],// Local Position
    pub dwb:    [f32; 3],// Body Angular Acceleration
    pub wb:     [f32; 3],// Body Relative Velocity
    pub e:      [f32; 3],// Body Relative Angular Position
    // pub rr:     [f32; 3],// ECI Position
    // pub re:     [f32; 3],// ECEF Position
    // pub tl2r:   [f32; 9],// Local Level to ECI DCM
    // pub tr2e:   [f32; 9],// ECI to ECEF DCM
    pub qb:     [f32; 4],// Body Relative Quaternion
    pub m:      [f32; 3] // Magnetometer State
}
impl State{
    pub fn initialize()->Self{
        State{
            // t: 0,
            ab:     [0.0, 0.0, 0.0],
            // vl:     [0.0, 0.0, 0.0],
            // rl:     [0.0, 0.0, 0.0],
            dwb:    [0.0, 0.0, 0.0], 
            wb:     [0.0, 0.0, 0.0], 
            e:      [0.0, 0.0, 0.0], 
            // rr:     [0.0, 0.0, 0.0],
            // re:     [0.0, 0.0, 0.0],
            // tl2r:   [0.0, 0.0, 0.0, 
            //             0.0, 0.0, 0.0,
            //             0.0, 0.0, 0.0], 
            // tr2e:   [0.0, 0.0, 0.0, 
            //             0.0, 0.0, 0.0,
            //             0.0, 0.0, 0.0], 
            qb:     [0.0, 0.0, 0.0, 0.0], 
            m:      [0.0, 0.0, 0.0]
        }
    }
}