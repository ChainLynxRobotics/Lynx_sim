use std::f32::consts::PI;

use rapier3d::{
    dynamics::{LockedAxes, RigidBodyBuilder},
    geometry::ColliderBuilder,
    math::{Pose3, Vec3, Vector},
};
use sdl3::{event::Event, keyboard::Keycode};
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
    let mut drive_base = RigidBodyBuilder::dynamic().build();
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
    let ground = RigidBodyBuilder::fixed()
        .translation(Vector::new(0.0, 0.0, -1.0))
        .build();
    let ground_colider = ColliderBuilder::cuboid(1.0, 1.0, 0.1).build();
    let ground = physics_world.rigid_body_set.insert(ground);
    physics_world.collider_set.insert_with_parent(
        ground_colider,
        ground,
        &mut physics_world.rigid_body_set,
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
        for event in window.event_pump.poll_iter() {
            match event {
                Event::Quit { .. }
                | Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => panic!("uhh close the app i guess"),
                Event::KeyDown {
                    keycode: Some(Keycode::W),
                    ..
                } => {
                    window.camera.pose = window.camera.pose.append_translation(Vec3 {
                        x: 0.01,
                        y: 0.0,
                        z: 0.0,
                    })
                }
                Event::KeyDown {
                    keycode: Some(Keycode::S),
                    ..
                } => {
                    window.camera.pose = window.camera.pose.append_translation(Vec3 {
                        x: -0.01,
                        y: 0.0,
                        z: 0.0,
                    })
                }
                Event::KeyDown {
                    keycode: Some(Keycode::A),
                    ..
                } => {
                    window.camera.pose = window.camera.pose.append_translation(Vec3 {
                        x: 0.0,
                        y: 0.01,
                        z: 0.0,
                    })
                }
                Event::KeyDown {
                    keycode: Some(Keycode::D),
                    ..
                } => {
                    window.camera.pose = window.camera.pose.append_translation(Vec3 {
                        x: 0.0,
                        y: -0.01,
                        z: 0.0,
                    })
                }
                Event::KeyDown {
                    keycode: Some(Keycode::Q),
                    ..
                } => {
                    window.camera.pose = window.camera.pose.append_translation(Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: -0.01,
                    })
                }
                Event::KeyDown {
                    keycode: Some(Keycode::E),
                    ..
                } => {
                    window.camera.pose = window.camera.pose.append_translation(Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.01,
                    })
                }
                Event::KeyDown {
                    keycode: Some(Keycode::Left),
                    ..
                } => {
                    let current_rot = window
                        .camera
                        .pose
                        .rotation
                        .to_euler(rapier3d::glamx::EulerRot::XYZ);
                    window.camera.pose = Pose3::new(
                        window.camera.pose.translation,
                        Vec3 {
                            x: current_rot.0,
                            y: current_rot.1,
                            z: current_rot.2 + 0.05,
                        },
                    )
                }
                Event::KeyDown {
                    keycode: Some(Keycode::Right),
                    ..
                } => {
                    let current_rot = window
                        .camera
                        .pose
                        .rotation
                        .to_euler(rapier3d::glamx::EulerRot::XYZ);
                    window.camera.pose = Pose3::new(
                        window.camera.pose.translation,
                        Vec3 {
                            x: current_rot.0,
                            y: current_rot.1,
                            z: current_rot.2 - 0.05,
                        },
                    )
                }
                Event::KeyDown {
                    keycode: Some(Keycode::Up),
                    ..
                } => {
                    let current_rot = window
                        .camera
                        .pose
                        .rotation
                        .to_euler(rapier3d::glamx::EulerRot::XYZ);
                    window.camera.pose = Pose3::new(
                        window.camera.pose.translation,
                        Vec3 {
                            x: current_rot.0,
                            y: current_rot.1 + 0.05,
                            z: current_rot.2,
                        },
                    )
                }
                Event::KeyDown {
                    keycode: Some(Keycode::Down),
                    ..
                } => {
                    let current_rot = window
                        .camera
                        .pose
                        .rotation
                        .to_euler(rapier3d::glamx::EulerRot::XYZ);
                    window.camera.pose = Pose3::new(
                        window.camera.pose.translation,
                        Vec3 {
                            x: current_rot.0,
                            y: current_rot.1 - 0.05,
                            z: current_rot.2,
                        },
                    )
                }
                Event::KeyDown {
                    keycode: Some(Keycode::P),
                    ..
                } => {
                    physics_world.step();
                }
                _ => {}
            }
        }
        ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));
    }
}
