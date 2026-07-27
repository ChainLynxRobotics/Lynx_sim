use crate::{physics_world, swerve_module::SwerveModule};
use rapier3d::{math::Vec2, pipeline::PhysicsWorld, prelude::RigidBodyHandle};
use whippyunits::unit;

pub struct Robot {
    drive_base: RigidBodyHandle,
    front_left: SwerveModule,
    front_right: SwerveModule,
    back_left: SwerveModule,
    back_right: SwerveModule,
}
impl Robot {
    pub fn new(
        width: unit!(m, f32),
        height: unit!(m, f32),
        cornner_radius: unit!(m, f32),
        front_left_location: Vec2,
        front_right_location: Vec2,
        back_left_location: Vec2,
        back_right_location: Vec2,
        physics_world: PhysicsWorld,
    ) -> Robot {
        todo!()
    }
}
