use rapier3d::prelude::RigidBodyHandle;
use whippyunits::unit;

struct SwerveModule {
    pub config: SwerveModuleConfig,
    pub wheel_handle: RigidBodyHandle,
    pub azumith_handle: RigidBodyHandle,
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
