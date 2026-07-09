use std::{
    thread,
    time::{Duration, Instant},
};

use whippyunits::{quantity, unit, value};

use rapier3d::{
    dynamics::{FixedJointBuilder, RigidBodyBuilder},
    geometry::ColliderBuilder,
    math::{Vec3, Vector},
};
use swerve_sim_3d::{physics_world::PhysicsWorld, util::debug_render::DebugWindow};
pub const SIMULATION_FREQUENCY: unit!(Hz, f32) = quantity!(50.0, Hz, f32);
pub const SIMULATION_TIMESTEP: unit!(s, f32) =
    quantity!(1.0 / value!(SIMULATION_FREQUENCY, Hz, f32), s, f32);

fn main() {
    let mut window = DebugWindow::spawn_debug_window();
    let mut physics_world = PhysicsWorld::new(SIMULATION_TIMESTEP);
    let drive_base = RigidBodyBuilder::dynamic().build();
    let drive_base = physics_world.rigid_body_set.insert(drive_base);
    let ground = RigidBodyBuilder::fixed()
        .translation(Vector::new(0.0, 0.0, -2.0))
        .build();
    let ground_colider = ColliderBuilder::cuboid(10.0, 10.0, 1.0).build();
    let ground = physics_world.rigid_body_set.insert(ground);
    physics_world.collider_set.insert_with_parent(
        ground_colider,
        ground,
        &mut physics_world.rigid_body_set,
    );
    let rb = RigidBodyBuilder::dynamic().build();
    let collider = ColliderBuilder::cylinder(1.0, 1.0).build();
    let rb = physics_world.rigid_body_set.insert(rb);
    physics_world
        .collider_set
        .insert_with_parent(collider, rb, &mut physics_world.rigid_body_set);

    let joint = FixedJointBuilder::new()
        .local_anchor1(Vec3::ZERO)
        .local_anchor2(Vec3::ZERO)
        .build();
    physics_world
        .multibody_joint_set
        .insert(drive_base, rb, joint, true);

    loop {
        let start_time = Instant::now();
        physics_world.step();
        window.render(&physics_world);
        let processing_time = start_time.elapsed();
        thread::sleep(Duration::from_secs_f32(0.1));
        if processing_time <= Duration::from_secs_f32(value!(SIMULATION_TIMESTEP, s, f32)) {
            thread::sleep(
                Duration::from_secs_f32(value!(SIMULATION_TIMESTEP, s, f32)) - processing_time,
            );
        }
    }
}
