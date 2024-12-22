pub fn f32_to_u32(value: f32)->(u32, bool){
    let value_rounded = value as u32;    
    if value > 0.0 {
        return (value_rounded as u32, false);
    }
    else{
        return (value_rounded as u32, true);
    }
}