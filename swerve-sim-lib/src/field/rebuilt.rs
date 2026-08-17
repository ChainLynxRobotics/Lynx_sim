use std::f32::consts::PI;

use rapier3d::dynamics::{RigidBodyBuilder, RigidBodyHandle};
use rapier3d::geometry::{ColliderBuilder, SharedShape};
use rapier3d::math::{Pose, Vec3, Vector};
use rapier3d_meshloader::load_from_path;
use whippyunits::{quantity, rescale, unit, value};

use crate::field::Field;
use crate::physics_world::PhysicsWorld;
use crate::{SLEEP_THRESHOLD, inch_to_m};

pub struct RebuiltField {
    fuel: Vec<RigidBodyHandle>,
}

const WIDTH: unit!(m, f32) = rescale!(quantity!(651.22, inch, f32), m, f32);
const HEIGHT: unit!(m, f32) = rescale!(quantity!(317.69, inch, f32), m, f32);
const TOTAL_FUEL: usize = 504;
const FUEL_OFFSET: unit!(m, f32) = rescale!(quantity!(72.0 / 12.0, inch, f32), m, f32);
const FUEL_RADIUS: unit!(m, f32) = quantity!(0.075, m, f32);

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
        // TODO: account for preloaded fuel
        // see 6.3.4 SCORING ELEMENTS https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
        // TODO: fix fuel sliding along ground rather than rolling

        let fuel_in_neutral_zone = TOTAL_FUEL - (24 * 4); // 24 fuel in the depot and in the outpost for each alliance
        let mut fuel: Vec<RigidBodyHandle> = Vec::with_capacity(TOTAL_FUEL);
        let mut translations: Vec<Vector> = Vec::with_capacity(fuel_in_neutral_zone / 4);
        let mut i = 0;
        'outer: loop {
            for j in 0..=5 {
                translations.push(Vector::new(
                    value!(((j as f32) * FUEL_OFFSET) + FUEL_OFFSET / 2.0, m, f32),
                    value!(
                        ((i as f32) * FUEL_OFFSET)
                            + rescale!(quantity!(1.0, inch, f32), m, f32)
                            + FUEL_RADIUS,
                        m,
                        f32
                    ),
                    value!(FUEL_RADIUS, m, f32),
                ));
                if (i * 6) + j >= fuel_in_neutral_zone / 4 {
                    break 'outer;
                }
            }
            i += 1;
        }
        translations
            .iter()
            .flat_map(|t| {
                [
                    t.clone(),
                    Vector::new(t.x, -t.y, t.z),
                    Vector::new(-t.x, t.y, t.z),
                    Vector::new(-t.x, -t.y, t.z),
                ]
            })
            .for_each(|t| {
                let rb = physics_world.rigid_body_set.insert(
                    RigidBodyBuilder::dynamic()
                        .pose(Pose::from_translation(t))
                        .gyroscopic_forces_enabled(false),
                );
                fuel.push(rb);
                let rb_real = physics_world.rigid_body_set.get_mut(rb).unwrap();
                let activation = rb_real.activation_mut();
                activation.normalized_linear_threshold = SLEEP_THRESHOLD;
                physics_world.collider_set.insert_with_parent(
                    ColliderBuilder::ball(0.075),
                    rb,
                    &mut physics_world.rigid_body_set,
                );
            });
        let bump_center = Vector::new(
            inch_to_m!(120.0 + 23.5),
            inch_to_m!((73.0 + 47.0) / 2.0),
            0.0,
        );
        let bump = [
            Vector::new(inch_to_m!(44.4 / 2.0), inch_to_m!(73.0 / 2.0), 0.0),
            Vector::new(-inch_to_m!(44.4 / 2.0), inch_to_m!(73.0 / 2.0), 0.0),
            Vector::new(inch_to_m!(44.4 / 2.0), -inch_to_m!(73.0 / 2.0), 0.0),
            Vector::new(-inch_to_m!(44.4 / 2.0), -inch_to_m!(73.0 / 2.0), 0.0),
            Vector::new(0.0, inch_to_m!(73.0 / 2.0), inch_to_m!(6.531)),
            Vector::new(0.0, -inch_to_m!(73.0 / 2.0), inch_to_m!(6.531)),
        ]
        .map(|t| t + bump_center);
        [
            bump.clone(),
            bump.map(|b| Vector::new(-b.x, b.y, b.z)),
            bump.map(|b| Vector::new(b.x, -b.y, b.z)),
            bump.map(|b| Vector::new(-b.x, -b.y, b.z)),
        ]
        .iter()
        .for_each(|bump| {
            let collider = ColliderBuilder::convex_hull(bump)
                .expect("Failed to convert the bump into a convex hull");
            physics_world.collider_set.insert_with_parent(
                collider,
                field,
                &mut physics_world.rigid_body_set,
            );
        });

        let trench_position = Vector::new(
            inch_to_m!(325.61 - 182.11),
            inch_to_m!((47.0 / 2.0) + 73.0 + (12.0 / 2.0)),
            inch_to_m!(29.25 / 2.0),
        );
        [
            trench_position,
            Vector::new(-trench_position.x, trench_position.y, trench_position.z),
            Vector::new(trench_position.x, -trench_position.y, trench_position.z),
            Vector::new(-trench_position.x, -trench_position.y, trench_position.z),
        ]
        .iter()
        .for_each(|p| {
            let collider = ColliderBuilder::cuboid(
                inch_to_m!(47.0 / 2.0),
                inch_to_m!(12.0 / 2.0),
                inch_to_m!(29.25 / 2.0),
            )
            .translation(*p);
            physics_world.collider_set.insert_with_parent(
                collider,
                field,
                &mut physics_world.rigid_body_set,
            );
        });

        let mut path = std::env::current_exe().expect("coulent get execution path");
        path.pop();
        path.pop();
        path.pop();
        path.push("swerve-sim-lib");
        path.push("assets");
        path.push("Hub.stl");

        let hub: Vec<SharedShape> = load_from_path(
            path,
            &rapier3d::geometry::MeshConverter::TriMesh,
            Vector::ONE,
        )
        .expect("Failed to load mesh")
        .iter()
        .map(|s| s.as_ref().expect("Failed to load shape"))
        .map(|s| s.shape.clone())
        .collect();
        assert!(hub.len() == 1, "multiple meshes in the hub file");
        let hub = hub[0].clone();
        let hub = ColliderBuilder::new(hub);
        let hub_positions = [
            Pose::new(
                Vec3::new(inch_to_m!(120.0 + 47.0 / 2.0), 0.0, 0.0),
                Vec3::new(0.0, 0.0, PI / 2.0),
            ),
            Pose::new(
                Vec3::new(-inch_to_m!(120.0 + 47.0 / 2.0), 0.0, 0.0),
                Vec3::new(0.0, 0.0, PI * 1.5),
            ),
        ];
        hub_positions.iter().for_each(|p| {
            let hub = hub.clone().position(*p);
            physics_world.collider_set.insert_with_parent(
                hub,
                field,
                &mut physics_world.rigid_body_set,
            );
        });

        let depot_position = Pose::new(
            Vector::new(
                inch_to_m!(325.61),
                inch_to_m!(-((317.69 / 2.0) - 82.84)),
                0.0,
            ),
            Vector::ZERO,
        );

        [
            depot_position,
            Pose::new(Vector::ZERO, Vector::Z * PI) * depot_position,
        ]
        .iter()
        .for_each(|p| {
            const HEIGHT: f32 = inch_to_m!(1.125);
            let sizes_and_positions = [
                (
                    Vector::new(
                        inch_to_m!((27.0 - 3.0) / 2.0),
                        inch_to_m!(3.0 / 2.0),
                        HEIGHT / 2.0,
                    ),
                    Vector::new(
                        -inch_to_m!((27.0 - 3.0) / 2.0),
                        inch_to_m!((42.0 - 3.0) / 2.0),
                        HEIGHT / 2.0,
                    ),
                ),
                (
                    Vector::new(
                        inch_to_m!((27.0 - 3.0) / 2.0),
                        inch_to_m!(3.0 / 2.0),
                        HEIGHT / 2.0,
                    ),
                    Vector::new(
                        -inch_to_m!((27.0 - 3.0) / 2.0),
                        inch_to_m!(-((42.0 - 3.0) / 2.0)),
                        HEIGHT / 2.0,
                    ),
                ),
                (
                    Vector::new(inch_to_m!(3.0 / 2.0), inch_to_m!(42.0 / 2.0), HEIGHT / 2.0),
                    Vector::new(-inch_to_m!(27.0 - (3.0 / 2.0)), 0.0, HEIGHT / 2.0),
                ),
            ]
            .iter()
            .for_each(|position_and_size| {
                let collider = ColliderBuilder::cuboid(
                    position_and_size.0.x,
                    position_and_size.0.y,
                    position_and_size.0.z,
                )
                .position(p.prepend_translation(position_and_size.1));

                physics_world.collider_set.insert_with_parent(
                    collider,
                    field,
                    &mut physics_world.rigid_body_set,
                );
            });
        });

        Box::new(RebuiltField { fuel })
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
