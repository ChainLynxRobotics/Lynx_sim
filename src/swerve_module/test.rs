#[cfg(test)]
mod test {
    use crate::physics_world;

    use super::super::default_configs::*;
    use super::super::*;

    #[test]
    fn test_creation() {
        let mut physics_world = physics_world::PhysicsWorld::default();
        let drive_base = RigidBodyBuilder::fixed().build();
        let drive_base = physics_world.rigid_body_set.insert(drive_base);
        _ = SwerveModule::new(
            generate_mk4i_swerve_config(Mk4iGearRatio::L2Plus, Mk4iWheel::Billet),
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            drive_base,
            &mut physics_world.rigid_body_set,
            &mut physics_world.collider_set,
            &mut physics_world.multibody_joint_set,
        );
    }
}
