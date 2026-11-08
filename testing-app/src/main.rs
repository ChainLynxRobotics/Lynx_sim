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
    field::{Field, rebuilt::RebuiltField},
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
pub const SUB_STEPS: u32 = 1;

fn main() {
    let mut window = DebugWindow::spawn_debug_window();
    let mut physics_world = PhysicsWorld::new(SIMULATION_TIMESTEP);

    let robots: Vec<Robot<_>> = (0..1)
        .map(|_| {
            Robot::new(
                Pose::IDENTITY.append_translation(Vector::new(7.0, 2.0, 0.0)),
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

    RebuiltField::new(&mut physics_world);

    let mut tracking = 0;
    let mut loop_overuns = 0;
    let mut last_draw = Instant::now();
    loop {
        let start_time = Instant::now();
        for _ in 0..SUB_STEPS {
            robots.iter().for_each(|r| {
                r.modules.iter().for_each(|module| {
                    module.apply_voltages(
                        quantity!(-12.0, volt, f32),
                        quantity!(0.0, volt, f32),
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
            println!("processing time: {:?}", processing_time);
            println!("loop overuns: {}", loop_overuns);
        }
        tracking += 1;
        if processing_time
            <= Duration::from_secs_f32(value!(SIMULATION_TIMESTEP * SUB_STEPS as f32, s, f32))
        {
            thread::sleep(
                Duration::from_secs_f32(value!(SIMULATION_TIMESTEP * SUB_STEPS as f32, s, f32)) * 5
                    - processing_time,
            );
        } else {
            loop_overuns += 1;
        }
    }
}
