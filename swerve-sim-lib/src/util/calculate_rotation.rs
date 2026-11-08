use std::f32::consts::PI;
use whippyunits::{quantity, unit};

pub fn calculate_rotation(
    last_rotation: unit!(radian, f32),
    current_rotation: unit!(radian, f32),
    current_speed: unit!(radian / s, f32),
    dt: unit!(s, f32),
) -> unit!(radian, f32) {
    let perdicted_rotation = last_rotation + current_speed * dt;
    let perdicted_rotation_wrapped = perdicted_rotation % quantity!(2.0 * PI, radian, f32);
    let perdicted_rotation_wrapped = if perdicted_rotation_wrapped < quantity!(0.0, radian, f32) {
        perdicted_rotation_wrapped + quantity!(2.0 * PI, radian, f32)
    } else {
        perdicted_rotation_wrapped
    };
    let perdicted_rotation_wrapped = if perdicted_rotation_wrapped > quantity!(PI, radian, f32) {
        perdicted_rotation_wrapped - quantity!(2.0 * PI, radian, f32)
    } else {
        perdicted_rotation_wrapped
    };
    println!("Predicted rotation: {}", perdicted_rotation_wrapped);
    println!("Actual rotation: {}", current_rotation);

    return quantity!(0.0, radian, f32);
}
