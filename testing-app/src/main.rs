use whippyunits::quantity;

use rapier3d::{
    dynamics::RigidBodyBuilder,
    geometry::{ColliderBuilder, Group, InteractionGroups, InteractionTestMode::Or},
    math::{Vec3, Vector},
};
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
    let mut physics_world = PhysicsWorld::new(quantity!(1.0, s, f32) / 100.0);
    let drive_base = RigidBodyBuilder::dynamic().build();
    let drive_base = physics_world.rigid_body_set.insert(drive_base);
    // let drive_base_colider = ColliderBuilder::cuboid(1.0, 1.0, 0.11)
    //     .collision_groups(InteractionGroups::new(Group::GROUP_2, Group::GROUP_1, Or))
    //     .restitution(0.0)
    //     .build();
    // physics_world.collider_set.insert_with_parent(
    //     drive_base_colider,
    //     drive_base,
    //     &mut physics_world.rigid_body_set,
    // );
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
        .translation(Vector::new(0.0, 0.0, -2.0))
        .build();
    let ground_colider = ColliderBuilder::cuboid(10.0, 10.0, 1.0)
        .collision_groups(InteractionGroups::new(
            Group::GROUP_1,
            Group::GROUP_1 | Group::GROUP_2,
            Or,
        ))
        .restitution(0.0)
        .build();
    let ground = physics_world.rigid_body_set.insert(ground);
    physics_world.collider_set.insert_with_parent(
        ground_colider,
        ground,
        &mut physics_world.rigid_body_set,
    );
    loop {
        window.render(&physics_world);
        thread::sleep(Duration::from_secs_f32(1.0 / 1.0));
        physics_world.step();
    }
}
