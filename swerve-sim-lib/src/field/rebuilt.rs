use std::f32::consts::PI;

use rapier3d::dynamics::{RigidBodyBuilder, RigidBodyHandle};
use rapier3d::geometry::{ColliderBuilder, SharedShape};
use rapier3d::math::{Pose, Vector};
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
        translations.iter().for_each(|t| {
            mirror_to_the_four_sectors(*t, |translation| {
                spawn_fuel(physics_world, *translation, &mut fuel)
            });
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
        ];
        mirror_to_the_four_sectors(bump_center, |center| {
            let points = bump.map(|t| t + center);
            let collider = ColliderBuilder::convex_hull(&points)
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
        mirror_to_the_four_sectors(trench_position, |p| {
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

        mirror_for_alliances(
            Pose::new(
                Vector::new(inch_to_m!(120.0 + 47.0 / 2.0), 0.0, 0.0),
                Vector::new(0.0, 0.0, PI / 2.0),
            ),
            |p| {
                let hub = hub.clone().position(*p);
                physics_world.collider_set.insert_with_parent(
                    hub,
                    field,
                    &mut physics_world.rigid_body_set,
                );
            },
        );

        let depot_position = Pose::new(
            Vector::new(
                inch_to_m!(325.61),
                inch_to_m!(-((317.69 / 2.0) - 82.84)),
                0.0,
            ),
            Vector::ZERO,
        );
        mirror_for_alliances(depot_position, |p| {
            for i in 0..=3 {
                for j in 0..=2 {
                    // from the perspective of the field dimension drawing
                    let top_fuel = p.prepend_translation(Vector::new(
                        -value!((FUEL_OFFSET * (1 + i) as f32) - FUEL_RADIUS, m, f32),
                        value!((FUEL_OFFSET * (1 + j) as f32) - FUEL_RADIUS, m, f32),
                        value!(FUEL_RADIUS, m, f32),
                    ));
                    let bottom_fuel = p.prepend_translation(Vector::new(
                        -value!((FUEL_OFFSET * (1 + i) as f32) - FUEL_RADIUS, m, f32),
                        -value!((FUEL_OFFSET * (1 + j) as f32) - FUEL_RADIUS, m, f32),
                        value!(FUEL_RADIUS, m, f32),
                    ));

                    spawn_fuel(physics_world, top_fuel.translation, &mut fuel);
                    spawn_fuel(physics_world, bottom_fuel.translation, &mut fuel);
                }
            }
            const HEIGHT: f32 = inch_to_m!(1.125);
            [
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
        let tower_center = Vector::new(
            inch_to_m!(325.61 - (40.0725 + 3.5 / 2.0)),
            inch_to_m!(-(26.22 + 121.25) + (87.38 + 82.84)),
            inch_to_m!(72.125 / 2.0),
        );
        mirror_for_alliances(Pose::from_translation(tower_center), |p| {
            [
                Vector::new(0.0, inch_to_m!(32.25 / 2.0), 0.0),
                Vector::new(0.0, inch_to_m!(-32.25 / 2.0), 0.0),
            ]
            .map(|t| t + p.translation)
            .iter()
            .for_each(|p| {
                let collider = ColliderBuilder::cuboid(
                    inch_to_m!(3.5 / 2.0),
                    inch_to_m!(1.5 / 2.0),
                    inch_to_m!(72.125 / 2.0),
                )
                .translation(*p);
                physics_world.collider_set.insert(collider);
            });
        });

        let top_wall = ColliderBuilder::cuboid(inch_to_m!(325.61), 0.5, inch_to_m!(19.2675 / 2.0))
            .translation(Vector::new(
                0.0,
                inch_to_m!(158.84) + 0.5,
                inch_to_m!(19.2675 / 2.0),
            ));
        let bottom_wall =
            ColliderBuilder::cuboid(inch_to_m!(325.61), 0.5, inch_to_m!(19.2675 / 2.0))
                .translation(Vector::new(
                    0.0,
                    -(inch_to_m!(158.84) + 0.5),
                    inch_to_m!(19.2675 / 2.0),
                ));

        physics_world.collider_set.insert_with_parent(
            top_wall,
            field,
            &mut physics_world.rigid_body_set,
        );
        physics_world.collider_set.insert_with_parent(
            bottom_wall,
            field,
            &mut physics_world.rigid_body_set,
        );

        let right_wall =
            ColliderBuilder::cuboid(0.5, inch_to_m!(158.84), inch_to_m!(19.2675 / 2.0))
                .translation(Vector::new(
                    inch_to_m!(325.61) + 0.5,
                    0.0,
                    inch_to_m!(19.2675 / 2.0),
                ));
        let left_wall = ColliderBuilder::cuboid(0.5, inch_to_m!(158.84), inch_to_m!(19.2675 / 2.0))
            .translation(Vector::new(
                -(inch_to_m!(325.61) + 0.5),
                0.0,
                inch_to_m!(19.2675 / 2.0),
            ));

        physics_world.collider_set.insert_with_parent(
            right_wall,
            field,
            &mut physics_world.rigid_body_set,
        );
        physics_world.collider_set.insert_with_parent(
            left_wall,
            field,
            &mut physics_world.rigid_body_set,
        );

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

fn spawn_fuel(physics_world: &mut PhysicsWorld, position: Vector, fuel: &mut Vec<RigidBodyHandle>) {
    let rb = physics_world.rigid_body_set.insert(
        RigidBodyBuilder::dynamic()
            .translation(position)
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
}
fn mirror_to_the_four_sectors<T: FnMut(&Vector)>(position: Vector, mut function: T) {
    [
        position,
        Vector::new(-position.x, position.y, position.z),
        Vector::new(position.x, -position.y, position.z),
        Vector::new(-position.x, -position.y, position.z),
    ]
    .iter()
    .for_each(|p| {
        function(p);
    });
}

fn mirror_for_alliances<T: FnMut(&Pose)>(position: Pose, mut function: T) {
    [position, Pose::new(Vector::ZERO, Vector::Z * PI) * position]
        .iter()
        .for_each(|p| {
            function(p);
        });
}
