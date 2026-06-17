use std::thread;

use rapier3d::{
    dynamics::{LockedAxes, RigidBodyBuilder},
    geometry::ColliderBuilder,
    math::{Vec3, Vector},
};
use std::time::Duration;
use swerve_sim_3d::{
    physics_world,
    swerve_module::{
        SwerveModule,
        default_configs::{Mk4iGearRatio, Mk4iWheel, generate_mk4i_swerve_config},
    },
};

fn main() {
    let window = swerve_sim_3d::util::debug_render::DebugWindow::spawn_debug_window();
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
    loop {
        thread::sleep(Duration::from_secs(1));
    }
}
