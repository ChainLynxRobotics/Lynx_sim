use crate::{
    BUMPER_INTERACTION_GROUPS, physics_world,
    swerve_module::{SwerveModule, config::SwerveModuleConfig},
};
use rapier3d::{
    dynamics::{MassProperties, RigidBodyBuilder},
    geometry::ColliderBuilder,
    math::{Pose, Vector},
    prelude::RigidBodyHandle,
};
use whippyunits::{quantity, unit, value};

pub struct Robot<const NUMBER_OF_SWERVE_MODULES: usize> {
    pub drive_base: RigidBodyHandle,
    pub modules: [SwerveModule; NUMBER_OF_SWERVE_MODULES],
}
impl<const NUMBER_OF_SWERVE_MODULES: usize> Robot<NUMBER_OF_SWERVE_MODULES> {
    pub fn new(
        starting_position: Pose,
        width: unit!(m, f32),
        height: unit!(m, f32),
        bumper_height: unit!(m, f32),
        cornner_radius: unit!(m, f32),
        mass: unit!(kg, f32),
        center_of_mass: Vector,
        moments_of_inertia: Vector,
        module_config: SwerveModuleConfig,
        module_locations: [(unit!(m, f32), unit!(m, f32)); NUMBER_OF_SWERVE_MODULES],
        physics_world: &mut physics_world::PhysicsWorld,
    ) -> Robot<NUMBER_OF_SWERVE_MODULES> {
        let drive_base_height = value!(
            (bumper_height / 2.0) - module_config.wheel_center_height + module_config.wheel_radius,
            m,
            f32
        );
        let drive_base_pose = starting_position.append_translation(Vector::Z * drive_base_height);
        let drive_base = RigidBodyBuilder::dynamic()
            .pose(drive_base_pose)
            .gyroscopic_forces_enabled(false)
            .build();
        let drive_base = physics_world.rigid_body_set.insert(drive_base);
        let drive_base_collider = if cornner_radius > quantity!(0.0, m, f32) {
            // TODO: Check if the slight performance hit of the round cuboid collider acctualy makes any difference
            // see https://rapier.rs/docs/user_guides/rust/colliders#round-shapes
            ColliderBuilder::round_cuboid(
                value!((height / 2.0) - cornner_radius, m, f32),
                value!((width / 2.0) - cornner_radius, m, f32),
                value!(bumper_height / 2.0, m, f32),
                value!(cornner_radius, m, f32),
            )
        } else {
            // https://github.com/dimforge/rapier/issues/969
            ColliderBuilder::cuboid(
                value!((height / 2.0) - cornner_radius, m, f32),
                value!((width / 2.0) - cornner_radius, m, f32),
                value!(bumper_height / 2.0, m, f32),
            )
        };
        let drive_base_collider = drive_base_collider
            .collision_groups(BUMPER_INTERACTION_GROUPS)
            .mass_properties(MassProperties::new(
                center_of_mass - Vector::Z * drive_base_height,
                value!(
                    mass - (module_config.azumith_mass + module_config.wheel_mass),
                    kg,
                    f32
                ),
                moments_of_inertia,
            ))
            .restitution(0.0)
            .build();
        physics_world.collider_set.insert_with_parent(
            drive_base_collider,
            drive_base,
            &mut physics_world.rigid_body_set,
        );

        let modules = module_locations.map(|location| {
            SwerveModule::new(
                module_config,
                Vector::new(
                    value!(location.0, m, f32),
                    value!(location.1, m, f32),
                    value!(-bumper_height / 2.0, m, f32),
                ),
                drive_base_pose,
                drive_base,
                &mut physics_world.rigid_body_set,
                &mut physics_world.collider_set,
                &mut physics_world.impulse_joint_set,
            )
        });
        Robot {
            drive_base,
            modules,
        }
    }
}
