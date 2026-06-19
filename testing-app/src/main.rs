use std::thread;

use rapier3d::{
    dynamics::{LockedAxes, RigidBodyBuilder},
    geometry::ColliderBuilder,
    math::{Vec3, Vector},
};
use std::time::Duration;
use swerve_sim_3d::{
    physics_world::PhysicsWorld,
    swerve_module::{
        SwerveModule,
        default_configs::{Mk4iGearRatio, Mk4iWheel, generate_mk4i_swerve_config},
    },
    util::debug_render::DebugWindow,
};

fn main() {
    let mut window = DebugWindow::spawn_debug_window();
    let mut physics_world = PhysicsWorld::default();
    let drive_base = RigidBodyBuilder::dynamic().build();
    let drive_base = physics_world.rigid_body_set.insert(drive_base);
    let _swerve_module = SwerveModule::new(
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
        window.render(&physics_world);
        thread::sleep(Duration::from_secs_f32(1.0 / 50.0));
        physics_world.step();
    }
}
