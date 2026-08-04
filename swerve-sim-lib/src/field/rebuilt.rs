use rapier3d::dynamics::{RigidBodyBuilder, RigidBodyHandle};
use rapier3d::geometry::ColliderBuilder;
use rapier3d::math::{Pose, Vector};
use whippyunits::{quantity, rescale, unit, value};

use crate::field::Field;
use crate::physics_world::PhysicsWorld;

pub struct RebuiltField {
    fuel: Vec<RigidBodyHandle>,
}

const WIDTH: unit!(m, f32) = rescale!(quantity!(651.22, inch, f32), m, f32);
const HEIGHT: unit!(m, f32) = rescale!(quantity!(317.69, inch, f32), m, f32);

impl Field for RebuiltField {
    fn new(physics_world: &mut PhysicsWorld) -> Box<Self> {
        let field = RigidBodyBuilder::fixed();
        let field = physics_world.rigid_body_set.insert(field);
        // TODO: Use halfspaces here
        let ground_collider = ColliderBuilder::cuboid(
            value!(WIDTH / 2.0, m, f32),
            value!(HEIGHT / 2.0, m, f32),
            1.0,
        )
        .position(Pose::from_translation(Vector::NEG_Z));
        physics_world.collider_set.insert_with_parent(
            ground_collider,
            field,
            &mut physics_world.rigid_body_set,
        );

        Box::new(RebuiltField {})
    }

    fn update(&mut self, physics_world: &mut PhysicsWorld) {
        todo!()
    }

    fn get_blue_score(&self, physics_world: &PhysicsWorld) -> u64 {
        todo!()
    }

    fn get_red_score(&self, physics_world: &PhysicsWorld) -> u64 {
        todo!()
    }
}
