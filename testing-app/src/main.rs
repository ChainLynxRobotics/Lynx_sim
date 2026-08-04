use std::{
    thread,
    time::{Duration, Instant},
};

use ipc_types::FRAME_RATE;
use whippyunits::{quantity, rescale, unit, value};

use rapier3d::{
    dynamics::RigidBodyBuilder, geometry::ColliderBuilder, math::Vector, prelude::Pose,
};
use swerve_sim_3d::{
    FIELD_INTERACTION_GROUPS,
    physics_world::PhysicsWorld,
    robot::Robot,
    swerve_module::default_configs::{
        Mk4iGearRatio::L2Plus, Mk4iWheel::Billet, generate_mk4i_swerve_config,
    },
    util::debug_render::DebugWindow,
};
pub const SIMULATION_FREQUENCY: unit!(Hz, f32) = quantity!(500.0, Hz, f32);
pub const SIMULATION_TIMESTEP: unit!(s, f32) =
    quantity!(1.0 / value!(SIMULATION_FREQUENCY, Hz, f32), s, f32);
pub const SUB_STEPS: u32 = 10;

fn main() {
    let mut window = DebugWindow::spawn_debug_window();
    let mut physics_world = PhysicsWorld::new(SIMULATION_TIMESTEP);

    let robots: Vec<Robot<_>> = (0..1)
        .map(|_| {
            Robot::new(
                Pose::IDENTITY.append_translation(Vector::new(0.0, 0.0, 0.0)),
                rescale!(quantity!(34.5, inch, f32), m, f32),
                rescale!(quantity!(34.5, inch, f32), m, f32),
                quantity!(0.11, m, f32),
                quantity!(0.01, m, f32),
                quantity!(50.0, kg, f32),
                Vector::ZERO,
                Vector::ONE,
                generate_mk4i_swerve_config(L2Plus, Billet),
                [
                    (quantity!(0.28, m, f32), (quantity!(0.28, m, f32))),
                    (quantity!(0.28, m, f32), (quantity!(-0.28, m, f32))),
                    (quantity!(-0.28, m, f32), (quantity!(0.28, m, f32))),
                    (quantity!(-0.28, m, f32), (quantity!(-0.28, m, f32))),
                ],
                &mut physics_world,
            )
        })
        .collect();

    for i in 0..=21 {
        for j in 0..=21 {
            let rb = physics_world.rigid_body_set.insert(
                RigidBodyBuilder::dynamic()
                    .pose(Pose::from_translation(Vector::new(
                        (((i as f32) - 10.5) / 10.5) * 4.0,
                        (((j as f32) - 10.5) / 10.5) * 4.0,
                        0.1,
                    )))
                    .gyroscopic_forces_enabled(false),
            );
            let rb_real = physics_world.rigid_body_set.get_mut(rb).unwrap();
            let activation = rb_real.activation_mut();
            activation.normalized_linear_threshold = 0.4;
            physics_world.collider_set.insert_with_parent(
                ColliderBuilder::ball(0.075),
                rb,
                &mut physics_world.rigid_body_set,
            );
        }
    }

    let ground = RigidBodyBuilder::fixed()
        .translation(Vector::new(0.0, 0.0, -1.0))
        .build();
    let ground_colider = ColliderBuilder::cuboid(10.0, 10.0, 1.0)
        .collision_groups(FIELD_INTERACTION_GROUPS)
        .restitution(0.0)
        .build();
    let ground = physics_world.rigid_body_set.insert(ground);
    physics_world.collider_set.insert_with_parent(
        ground_colider,
        ground,
        &mut physics_world.rigid_body_set,
    );
    let mut tracking = 0;
    let mut loop_overuns = 0;
    let mut last_draw = Instant::now();
    for _ in 0..1000 {
        let start_time = Instant::now();
        for _ in 0..SUB_STEPS {
            robots.iter().for_each(|r| {
                r.modules.iter().for_each(|module| {
                    module.apply_voltages(
                        quantity!(5.0, volt, f32),
                        quantity!(5.0, volt, f32),
                        SIMULATION_TIMESTEP,
                        &mut physics_world,
                    );
                })
            });
            physics_world.step();
        }
        if (last_draw + Duration::from_secs_f32(1.0 / FRAME_RATE)) <= Instant::now() {
            window.render(&physics_world);
            last_draw = Instant::now();
        }
        let processing_time = start_time.elapsed();
        if tracking % 50 == 0 {
            // println!("processing time: {:?}", processing_time);
            // println!("loop overuns: {}", loop_overuns);
        }
        tracking += 1;
        if processing_time
            <= Duration::from_secs_f32(value!(SIMULATION_TIMESTEP * SUB_STEPS as f32, s, f32))
        {
            thread::sleep(
                Duration::from_secs_f32(value!(SIMULATION_TIMESTEP * SUB_STEPS as f32, s, f32))
                    - processing_time,
            );
        } else {
            loop_overuns += 1;
        }
    }
}
