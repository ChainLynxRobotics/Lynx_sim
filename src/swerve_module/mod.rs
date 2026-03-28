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
    pub drive_moi: unit!(kg * m ^ 2, f32),
    pub azumith_moi: unit!(kg * m ^ 2, f32),
    pub wheel_center_height: unit!(m, f32),
    pub wheel_cof: f32,
}
