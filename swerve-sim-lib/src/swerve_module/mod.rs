use std::f32::consts::PI;

use rapier3d::dynamics::{ImpulseJointSet, RigidBody};
use rapier3d::math::{AngVector, Pose3};
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
use crate::util::motor::Motor;

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
                value!(config.azumith_secondary_moi, kg * m ^ 2, f32),
                value!(config.azumith_moi, kg * m ^ 2, f32),
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
        let wheel = physics_world
            .rigid_body_set
            .get_mut(self.wheel)
            .expect("Rigid body set didnt have wheel");
        Self::apply_voltage_to_motor(
            drive_motor,
            drive,
            wheel,
            Vec3::Y,
            self.config.drive_gear_ratio,
        );

        let azumith = physics_world
            .rigid_body_set
            .get_mut(self.azumith)
            .expect("Rigid body set didnt have azumith");
        Self::apply_voltage_to_motor(
            turn_motor,
            turn,
            azumith,
            Vec3::Z,
            self.config.turn_gear_ratio,
        );
    }

    fn apply_voltage_to_motor(
        motor: Motor,
        voltage: unit!(volt, f32),
        rb: &mut RigidBody,
        rotation_axis: Vec3,
        gear_ratio: f32,
    ) {
        let rotation_axis = rotation_axis.normalize();
        rb.reset_torques(false);

        let speeds: AngVector =
            Pose3::from_parts(Vec3::ZERO, rb.position().rotation).inverse() * rb.angvel();

        let speed = speeds * rotation_axis;
        let speed_sign = 1.0f32.copysign(speed.x + speed.y + speed.z);
        let speed = speed.length() * speed_sign * gear_ratio;

        let calculated_torque = value!(
            motor.get_torque_from_voltage(voltage, quantity!(speed, radians / s, f32),),
            Nm,
            f32
        );

        let calculated_torque = rotation_axis * calculated_torque * gear_ratio;
        let calculated_torque =
            Pose3::from_parts(Vec3::ZERO, rb.position().rotation) * calculated_torque;

        rb.add_torque(calculated_torque, true);
    }
}
