#[cfg(test)]
mod test {
    use std::random::random;

    use rapier3d::math::{DEFAULT_EPSILON, Pose, Pose3};
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
            &mut physics_world.impulse_joint_set,
        );
    }

    // #[test]
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
            &mut physics_world.impulse_joint_set,
        );
        println!("{}", swerve_module.config.drive_moi);
        println!("{}", swerve_module.config.azumith_moi);
        println!("{}", swerve_module.config.wheel_secondary_moi);
        let azumith = physics_world
            .rigid_body_set
            .get_mut(swerve_module.azumith)
            .unwrap();
        println!("{}", azumith.angvel());
        println!("{:?}", azumith.position());
        physics_world.step();
        let azumith = physics_world
            .rigid_body_set
            .get_mut(swerve_module.azumith)
            .unwrap();
        println!("{}", azumith.angvel());
        println!("{:?}", azumith.position());
        physics_world.step();
        let azumith = physics_world
            .rigid_body_set
            .get_mut(swerve_module.azumith)
            .unwrap();
        println!("{}", azumith.angvel());
        println!("{:?}", azumith.position());
        physics_world.step();
        let azumith = physics_world
            .rigid_body_set
            .get_mut(swerve_module.azumith)
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
            .get_mut(swerve_module.azumith)
            .unwrap();
        println!("{}", azumith.angvel());
        println!("{:?}", azumith.position());
        panic!()
    }

    #[test]
    fn test_inverse() {
        let pose = Pose::new(Vec3::ZERO, AngVector::ZERO);
        let inverse = pose.inverse();
        for _ in 0..10000 {
            let x: i32 = random(..);
            let y: i32 = random(..);
            let z: i32 = random(..);
            assert_eq!(
                inverse
                    * Vec3 {
                        x: x as f32,
                        y: y as f32,
                        z: z as f32
                    },
                Vec3 {
                    x: x as f32,
                    y: y as f32,
                    z: z as f32
                }
            );
        }
        let pose = Pose::new(Vec3::X, AngVector::ZERO);
        let inverse = pose.inverse();
        assert_eq!(
            inverse
                * Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0
                },
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        );
        assert_eq!(
            inverse
                * Vec3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0
                },
            Vec3 {
                x: -1.0,
                y: 1.0,
                z: 0.0
            }
        );
        assert_eq!(
            inverse
                * Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 1.0
                },
            Vec3 {
                x: -1.0,
                y: 0.0,
                z: 1.0
            }
        );
        let pose = Pose3::new(Vec3::ZERO, Vec3::Z * PI * 0.5);
        let inverse = pose.inverse();
        assert_eq!(
            (inverse * Pose3::IDENTITY).translation,
            (Pose3::IDENTITY).translation
        );
        assert!(are_poses_approx_eq(
            inverse * Pose3::new(Vec3::X, Vec3::ZERO),
            Pose3::new(Vec3::Y * -1.0, Vec3::Z * PI * -0.5)
        ));
    }

    fn are_poses_approx_eq(pose1: Pose3, pose2: Pose3) -> bool {
        return approx_eq(pose1.translation.x, pose2.translation.x)
            && approx_eq(pose1.translation.y, pose2.translation.y)
            && approx_eq(pose1.translation.z, pose2.translation.z)
            && approx_eq(pose1.rotation.w, pose2.rotation.w)
            && approx_eq(pose1.rotation.x, pose2.rotation.x)
            && approx_eq(pose1.rotation.y, pose2.rotation.y)
            && approx_eq(pose1.rotation.z, pose2.rotation.z);
    }
    fn approx_eq(a: f32, b: f32) -> bool {
        return (a - DEFAULT_EPSILON) <= b && (a + DEFAULT_EPSILON) >= b;
    }
}
