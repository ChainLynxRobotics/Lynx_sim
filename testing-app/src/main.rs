use std::f32::consts::PI;

use rapier3d::{
    dynamics::{LockedAxes, RigidBodyBuilder},
    math::{Pose3, Vec3},
    na::Translation3,
};
use std::time::Duration;
use swerve_sim_3d::{
    physics_world,
    swerve_module::{
        SwerveModule,
        default_configs::{Mk4iGearRatio, Mk4iWheel, generate_mk4i_swerve_config},
    },
    util::debug_render::{Camera, DebugWindow},
};

fn main() {
    let (window, context) = swerve_sim_3d::util::debug_render::spawn_debug_window();
    let mut physics_world = physics_world::PhysicsWorld::default();
    let mut drive_base = RigidBodyBuilder::dynamic().gravity_scale(0.0).build();
    drive_base.set_locked_axes(LockedAxes::all(), false);
    let drive_base = physics_world.rigid_body_set.insert(drive_base);
    let swerve_module = SwerveModule::new(
        generate_mk4i_swerve_config(Mk4iGearRatio::L2Plus, Mk4iWheel::Billet),
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        drive_base,
        &mut physics_world.rigid_body_set,
        &mut physics_world.collider_set,
        &mut physics_world.multibody_joint_set,
    );
    let (size_x, size_y) = window.output_size().unwrap();
    let mut window = DebugWindow {
        canvas: window,
        event_pump: context.event_pump().unwrap(),
        camera: Camera {
            fov: PI / 2.0,
            pose: Pose3::from_translation(Vec3::new(-0.5, 0.0, 0.0)),
            aspect_ratio: size_x as f32 / size_y as f32,
            x_pixels: size_x,
            y_pixels: size_y,
        },
    };
    loop {
        swerve_sim_3d::util::debug_render::draw_debug_window(
            Some(&mut window),
            &physics_world.rigid_body_set,
            &physics_world.collider_set,
            &physics_world.impulse_joint_set,
            &physics_world.multibody_joint_set,
            &physics_world.narrow_phase,
        );
        ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));
    }
}
