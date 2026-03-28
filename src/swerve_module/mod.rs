use std::f32::consts::PI;

use rapier3d::{
    math::{Vec3, Vector},
    prelude::{
        ColliderBuilder, ColliderSet, MassProperties, MultibodyJointSet, RevoluteJointBuilder,
        RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
    },
};
use whippyunits::{unit, value};

#[derive(Debug, PartialEq)]
struct SwerveModule {
    pub config: SwerveModuleConfig,
    pub wheel_handle: RigidBodyHandle,
    pub azumith_handle: RigidBodyHandle,
}
impl SwerveModule {
    pub fn new(
        config: SwerveModuleConfig,
        module_top_center: Vector,
        drive_base_handle: RigidBodyHandle,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        joint_set: &mut MultibodyJointSet,
    ) -> Self {
        let wheel = RigidBodyBuilder::dynamic()
            .translation(
                module_top_center - Vec3::new(0.0, 0.0, value!(config.wheel_center_height, m, f32)),
            )
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
        .build();
        let azumith = RigidBodyBuilder::dynamic()
            .translation(
                module_top_center
                    - Vec3::new(0.0, 0.0, value!(config.azumith_center_height, m, f32)),
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
        .rotation(Vec3::new(PI, 0.0, 0.0));

        let wheel = rigid_body_set.insert(wheel);
        let azumith = rigid_body_set.insert(azumith);

        collider_set.insert_with_parent(wheel_colider, wheel, rigid_body_set);
        collider_set.insert_with_parent(azumith_colider, azumith, rigid_body_set);

        let wheel_joint = RevoluteJointBuilder::new(Vec3::Y)
            .local_anchor1(Vec3::ZERO)
            .local_anchor2(
                module_top_center - Vec3::new(0.0, 0.0, value!(config.wheel_center_height, m, f32)),
            )
            .build();
        let azumith_joint = RevoluteJointBuilder::new(Vec3::Z)
            .local_anchor1(Vec3::ZERO)
            .local_anchor2(
                module_top_center
                    - Vec3::new(0.0, 0.0, value!(config.azumith_center_height, m, f32)),
            )
            .build();

        joint_set.insert(wheel, drive_base_handle, wheel_joint, true);
        joint_set.insert(azumith, drive_base_handle, azumith_joint, true);
        return SwerveModule {
            config,
            wheel_handle: wheel,
            azumith_handle: azumith,
        };
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct SwerveModuleConfig {
    pub wheel_radius: unit!(m, f32),
    pub wheel_width: unit!(m, f32),
    pub wheel_mass: unit!(kg, f32),
    pub drive_moi: unit!(kg * m ^ 2, f32),
    pub wheel_secondary_moi: unit!(kg * m ^ 2, f32),
    pub azumith_moi: unit!(kg * m ^ 2, f32),
    pub azumith_secondary_moi: unit!(kg * m ^ 2, f32),
    pub azumith_mass: unit!(kg, f32),
    pub azumith_center_height: unit!(m, f32),
    pub azumith_thickness: unit!(m, f32),
    pub azumith_radius: unit!(m, f32),
    pub wheel_center_height: unit!(m, f32),
    pub wheel_cof: f32,
    pub wheel_coefficient_of_restetution: f32,
}
impl SwerveModuleConfig {
    pub fn new(
        wheel_radius: unit!(m, f32),
        wheel_width: unit!(m, f32),
        wheel_mass: unit!(kg, f32),
        drive_moi: unit!(kg * m ^ 2, f32),
        wheel_secondary_moi: unit!(kg * m ^ 2, f32),
        azumith_moi: unit!(kg * m ^ 2, f32),
        azumith_secondary_moi: unit!(kg * m ^ 2, f32),
        azumith_mass: unit!(kg, f32),
        azumith_center_height: unit!(m, f32),
        azumith_thickness: unit!(m, f32),
        azumith_radius: unit!(m, f32),
        wheel_center_height: unit!(m, f32),
        wheel_cof: f32,
        wheel_coefficient_of_restetution: f32,
    ) -> Self {
        return SwerveModuleConfig {
            wheel_radius,
            wheel_width,
            wheel_mass,
            drive_moi,
            wheel_secondary_moi,
            azumith_moi,
            azumith_secondary_moi,
            azumith_mass,
            azumith_center_height,
            azumith_thickness,
            azumith_radius,
            wheel_center_height,
            wheel_cof,
            wheel_coefficient_of_restetution,
        };
    }
}
