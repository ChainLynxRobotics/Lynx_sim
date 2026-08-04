use crate::physics_world::PhysicsWorld;

pub mod rebuilt;

pub trait Field {
    fn new(physics_world: &mut PhysicsWorld) -> Box<Self>;
    fn update(&mut self, physics_world: &mut PhysicsWorld);
    fn get_blue_score(&self, physics_world: &PhysicsWorld) -> u64;
    fn get_red_score(&self, physics_world: &PhysicsWorld) -> u64;
}
