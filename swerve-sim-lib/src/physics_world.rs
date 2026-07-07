use rapier3d::{
    math::Vec3,
    prelude::{
        BroadPhaseBvh, CCDSolver, ColliderSet, DefaultBroadPhase, ImpulseJointSet,
        IntegrationParameters, IslandManager, MultibodyJointSet, NarrowPhase, PhysicsPipeline,
        RigidBodySet,
    },
};
use whippyunits::{quantity, unit, value};

pub struct PhysicsWorld {
    pub physics_pipeline: PhysicsPipeline,
    pub gravity: Vec3,
    pub integration_parameters: IntegrationParameters,
    pub island_manager: IslandManager,
    pub broad_phase: BroadPhaseBvh,
    pub narrow_phase: NarrowPhase,
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub physics_hooks: (),
    pub event_handlers: (),
}
impl PhysicsWorld {
    pub fn new(dt: unit!(s, f32)) -> PhysicsWorld {
        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.dt = value!(dt, s, f32);
        // integration_parameters.num_solver_iterations = 64;
        // integration_parameters.num_internal_stabilization_iterations = 16;
        // integration_parameters.normalized_max_corrective_velocity = 1.0;

        return PhysicsWorld {
            physics_pipeline: PhysicsPipeline::new(),
            gravity: Vec3::new(0.0, 0.0, -9.80665),
            integration_parameters: integration_parameters,
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            physics_hooks: (),
            event_handlers: (),
        };
    }
    pub fn step(&mut self) {
        self.physics_pipeline.step(
            self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            &mut self.physics_hooks,
            &mut self.event_handlers,
        );
    }
}
impl Default for PhysicsWorld {
    fn default() -> Self {
        return PhysicsWorld::new(quantity!(0.02, s, f32));
    }
}
