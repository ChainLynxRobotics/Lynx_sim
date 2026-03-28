use rapier3d::{
    math::Vec3,
    prelude::{
        BroadPhaseBvh, CCDSolver, ColliderSet, DefaultBroadPhase, ImpulseJointSet,
        IntegrationParameters, IslandManager, MultibodyJointSet, NarrowPhase, RigidBodySet,
    },
};
use whippyunits::{unit, value};

pub struct PhysicsWorld {
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

        return PhysicsWorld {
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
}
