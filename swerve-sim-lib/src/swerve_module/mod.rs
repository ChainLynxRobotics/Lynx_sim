use std::f32::consts::PI;

use rapier3d::dynamics::ImpulseJointSet;
use rapier3d::math::AngVector;
use rapier3d::{
    math::{Vec3, Vector},
    prelude::{
        ColliderBuilder, ColliderSet, MassProperties, RevoluteJointBuilder, RigidBodyBuilder,
        RigidBodyHandle, RigidBodySet,
    },
};
use whippyunits::{quantity, unit, value};

use crate::ROBOT_INTERACTION_GROUPS;
use crate::physics_world::PhysicsWorld;

use self::config::SwerveModuleConfig;

pub mod config;
pub mod default_configs;
mod test;

#[derive(Debug, PartialEq)]
pub struct SwerveModule {
    pub config: SwerveModuleConfig,
    pub wheel: RigidBodyHandle,
    pub azumith: RigidBodyHandle,
}
impl SwerveModule {
    pub fn new(
        config: SwerveModuleConfig,
        module_center: Vector,
        drive_base: RigidBodyHandle,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        joint_set: &mut ImpulseJointSet,
    ) -> Self {
        let azumith = RigidBodyBuilder::dynamic()
            .translation(
                module_center + Vec3::new(0.0, 0.0, value!(config.azumith_center_height, m, f32)),
            )
            .build();
        let azumith_colider = ColliderBuilder::cylinder(
            value!(config.azumith_thickness, m, f32) / 2.0,
            value!(config.azumith_radius, m, f32),
        )
        .mass_properties(MassProperties::new(
            Vec3::ZERO,
            value!(config.azumith_mass, kg, f32),
            Vec3::new(
                value!(config.azumith_secondary_moi, kg * m ^ 2, f32),
                value!(config.azumith_moi, kg * m ^ 2, f32),
                value!(config.azumith_secondary_moi, kg * m ^ 2, f32),
            ),
        ))
        .collision_groups(ROBOT_INTERACTION_GROUPS)
        .rotation(Vec3::new(PI / 2.0, 0.0, 0.0))
        .restitution(0.0)
        .build();
        let azumith = rigid_body_set.insert(azumith);
        collider_set.insert_with_parent(azumith_colider, azumith, rigid_body_set);

        let wheel = RigidBodyBuilder::dynamic()
            .translation(
                module_center + Vec3::new(0.0, 0.0, value!(config.wheel_center_height, m, f32)),
            )
            .clone()
            .ccd_enabled(true)
            .soft_ccd_prediction(0.05)
            // .angular_damping(1.5)
            .build();
        let wheel_colider = ColliderBuilder::cylinder(
            value!(config.wheel_width, m, f32) / 2.0,
            value!(config.wheel_radius, m, f32),
        )
        .friction(config.wheel_cof)
        .restitution(config.wheel_coefficient_of_restetution)
        .mass_properties(MassProperties::new(
            Vec3::ZERO,
            value!(config.wheel_mass, kg, f32),
            Vec3::new(
                value!(config.wheel_secondary_moi, kg * m ^ 2, f32),
                value!(config.drive_moi, kg * m ^ 2, f32),
                value!(config.wheel_secondary_moi, kg * m ^ 2, f32),
            ),
        ))
        .collision_groups(ROBOT_INTERACTION_GROUPS)
        .build();
        let wheel = rigid_body_set.insert(wheel);
        collider_set.insert_with_parent(wheel_colider, wheel, rigid_body_set);

        let azumith_joint = RevoluteJointBuilder::new(Vec3::Z)
            .local_anchor1(Vec3::ZERO)
            .local_anchor2(
                module_center + Vec3::new(0.0, 0.0, value!(config.azumith_center_height, m, f32)),
            )
            .build();
        joint_set.insert(azumith, drive_base, azumith_joint, true);

        let wheel_joint = RevoluteJointBuilder::new(Vec3::Y)
            .local_anchor1(Vec3::ZERO)
            .local_anchor2(Vec3::new(
                0.0,
                0.0,
                value!(config.wheel_center_height, m, f32),
            ))
            .build();
        joint_set.insert(wheel, azumith, wheel_joint, true);

        return SwerveModule {
            config,
            wheel,
            azumith,
        };
    }

    pub fn apply_voltages(
        &self,
        drive: unit!(volt, f32),
        turn: unit!(volt, f32),
        physics_world: &mut PhysicsWorld,
    ) {
        let drive_motor = self.config.drive_motor;
        let turn_motor = self.config.turn_motor;
        {
            let wheel = physics_world
                .rigid_body_set
                .get_mut(self.wheel)
                .expect("Rigid body set didnt have wheel");
            wheel.reset_torques(false);
            let wheel_speeds: AngVector = wheel.position().inverse() * wheel.angvel();

            wheel.add_torque(
                wheel.position()
                    * (Vec3::Y
                        * value!(
                            drive_motor.get_torque_from_voltage(
                                drive,
                                quantity!(
                                    wheel_speeds.y * self.config.drive_gear_ratio,
                                    radians / s,
                                    f32
                                ),
                            ),
                            Nm,
                            f32
                        )
                        * self.config.drive_gear_ratio),
                true,
            );
        }

        let azumith = physics_world
            .rigid_body_set
            .get_mut(self.azumith)
            .expect("Rigid body set didnt have azumith");
        azumith.reset_torques(false);
        let azumith_speeds = azumith.position().inverse() * azumith.angvel();

        azumith.add_torque(
            azumith.position()
                * (Vec3::Z
                    * value!(
                        turn_motor.get_torque_from_voltage(
                            turn,
                            quantity!(
                                azumith_speeds.z * self.config.turn_gear_ratio,
                                radians / s,
                                f32
                            ),
                        ),
                        Nm,
                        f32
                    )
                    * self.config.turn_gear_ratio),
            true,
        );
    }
}
