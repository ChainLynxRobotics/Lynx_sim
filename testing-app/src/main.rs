use std::{
    thread,
    time::{Duration, Instant},
};

use whippyunits::{quantity, unit, value};

use rapier3d::{
    dynamics::RigidBodyBuilder,
    geometry::ColliderBuilder,
    math::{Vec3, Vector},
};
use swerve_sim_3d::{
    FIELD_INTERACTION_GROUPS, ROBOT_INTERACTION_GROUPS,
    physics_world::PhysicsWorld,
    swerve_module::{
        SwerveModule,
        default_configs::{Mk4iGearRatio, Mk4iWheel, generate_mk4i_swerve_config},
    },
    util::debug_render::DebugWindow,
};
pub const SIMULATION_FREQUENCY: unit!(Hz, f32) = quantity!(250.0, Hz, f32);
pub const SIMULATION_TIMESTEP: unit!(s, f32) =
    quantity!(1.0 / value!(SIMULATION_FREQUENCY, Hz, f32), s, f32);

fn main() {
    let mut window = DebugWindow::spawn_debug_window();
    let mut physics_world = PhysicsWorld::new(SIMULATION_TIMESTEP);
    let drive_base = RigidBodyBuilder::dynamic().build();
    let drive_base = physics_world.rigid_body_set.insert(drive_base);
    // 17.25 in
    let drive_base_colider = ColliderBuilder::cuboid(0.44, 0.44, 0.055)
        .collision_groups(ROBOT_INTERACTION_GROUPS)
        .restitution(0.0)
        .mass(50.0)
        .build();
    physics_world.collider_set.insert_with_parent(
        drive_base_colider,
        drive_base,
        &mut physics_world.rigid_body_set,
    );
    // 6.375 in from edge
    let swerve_module1 = SwerveModule::new(
        generate_mk4i_swerve_config(Mk4iGearRatio::L2Plus, Mk4iWheel::Billet),
        Vec3 {
            x: 0.28,
            y: 0.28,
            z: -0.055,
        },
        drive_base,
        &mut physics_world.rigid_body_set,
        &mut physics_world.collider_set,
        &mut physics_world.impulse_joint_set,
    );
    physics_world
        .rigid_body_set
        .get_mut(swerve_module1.azumith)
        .unwrap()
        .add_torque(
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.1,
            },
            true,
        );
    let swerve_module2 = SwerveModule::new(
        generate_mk4i_swerve_config(Mk4iGearRatio::L2Plus, Mk4iWheel::Billet),
        Vec3 {
            x: -0.28,
            y: 0.28,
            z: -0.055,
        },
        drive_base,
        &mut physics_world.rigid_body_set,
        &mut physics_world.collider_set,
        &mut physics_world.impulse_joint_set,
    );
    let swerve_module3 = SwerveModule::new(
        generate_mk4i_swerve_config(Mk4iGearRatio::L2Plus, Mk4iWheel::Billet),
        Vec3 {
            x: 0.28,
            y: -0.28,
            z: -0.055,
        },
        drive_base,
        &mut physics_world.rigid_body_set,
        &mut physics_world.collider_set,
        &mut physics_world.impulse_joint_set,
    );
    let swerve_module4 = SwerveModule::new(
        generate_mk4i_swerve_config(Mk4iGearRatio::L2Plus, Mk4iWheel::Billet),
        Vec3 {
            x: -0.28,
            y: -0.28,
            z: -0.055,
        },
        drive_base,
        &mut physics_world.rigid_body_set,
        &mut physics_world.collider_set,
        &mut physics_world.impulse_joint_set,
    );
    let ground = RigidBodyBuilder::fixed()
        .translation(Vector::new(0.0, 0.0, -2.0))
        .build();
    let ground_colider = ColliderBuilder::cuboid(10.0, 10.0, 1.0)
        .collision_groups(FIELD_INTERACTION_GROUPS)
        .restitution(0.0)
        .build();
    let ground = physics_world.rigid_body_set.insert(ground);
    physics_world.collider_set.insert_with_parent(
        ground_colider,
        ground,
        &mut physics_world.rigid_body_set,
    );
    let mut tracking = 0;
    let mut loop_overuns = 0;
    loop {
        let start_time = Instant::now();
        physics_world.step();
        if tracking % (value!(SIMULATION_FREQUENCY, Hz, f32) as u32 / 30) == 0 {
            window.render(&physics_world);
        }
        let processing_time = start_time.elapsed();
        if tracking % 50 == 0 {
            println!("processing time: {:?}", processing_time);
            println!("loop overuns: {}", loop_overuns);
        }
        tracking += 1;
        if processing_time <= Duration::from_secs_f32(value!(SIMULATION_TIMESTEP, s, f32)) {
            thread::sleep(
                Duration::from_secs_f32(value!(SIMULATION_TIMESTEP, s, f32)) - processing_time,
            );
        } else {
            loop_overuns += 1;
        }
    }
}
