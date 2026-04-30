#[cfg(test)]
mod test {
    use rapier3d::prelude::LockedAxes;

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

    #[test]
    fn test_azumith() {
        let mut physics_world = physics_world::PhysicsWorld::default();
        let mut drive_base = RigidBodyBuilder::dynamic().gravity_scale(0.0).build();
        drive_base.set_locked_axes(LockedAxes::all(), false);
        let drive_base = physics_world.rigid_body_set.insert(drive_base);
        let swerve_module = SwerveModule::new(
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
        println!("{}", swerve_module.config.drive_moi);
        println!("{}", swerve_module.config.azumith_moi);
        println!("{}", swerve_module.config.wheel_secondary_moi);
        let azumith = physics_world
            .rigid_body_set
            .get_mut(swerve_module.azumith_handle)
            .unwrap();
        println!("{}", azumith.angvel());
        println!("{:?}", azumith.position());
        physics_world.step();
        let azumith = physics_world
            .rigid_body_set
            .get_mut(swerve_module.azumith_handle)
            .unwrap();
        println!("{}", azumith.angvel());
        println!("{:?}", azumith.position());
        physics_world.step();
        let azumith = physics_world
            .rigid_body_set
            .get_mut(swerve_module.azumith_handle)
            .unwrap();
        println!("{}", azumith.angvel());
        println!("{:?}", azumith.position());
        physics_world.step();
        let azumith = physics_world
            .rigid_body_set
            .get_mut(swerve_module.azumith_handle)
            .unwrap();
        println!("{}", azumith.angvel());
        println!("{:?}", azumith.position());
        azumith.add_torque(
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            true,
        );
        physics_world.step();
        let azumith = physics_world
            .rigid_body_set
            .get_mut(swerve_module.azumith_handle)
            .unwrap();
        println!("{}", azumith.angvel());
        println!("{:?}", azumith.position());
        panic!()
    }
    #[test]
    fn test_window() {
        crate::util::debug_render::spawn_debug_window();
    }
}
