use std::{
    process::Command,
    sync::mpsc::{self, Receiver, Sender},
    thread::JoinHandle,
};

use ipc_channel::ipc::{IpcOneShotServer, IpcSender};
use ipc_types::{DebugLine, Frame};
use rapier3d::{
    math::Vector,
    pipeline::{
        DebugColor, DebugRenderBackend, DebugRenderMode, DebugRenderObject, DebugRenderPipeline,
        DebugRenderStyle,
    },
};

use crate::physics_world::PhysicsWorld;
fn hsla_to_rgb(hsla: rapier3d::prelude::DebugColor) -> (f32, f32, f32, f32) {
    let h = hsla[0];
    let s = hsla[1];
    let l = hsla[2];
    let a = hsla[3];
    if s == 0.0 {
        return (l, l, l, a);
    }

    let q = if l < 0.5 {
        l * (1.0 + s)
    } else {
        l + s - l * s
    };
    let p = 2.0 * l - q;

    let r = hue_to_rgb(p, q, h / 360.0 + 1.0 / 3.0);
    let g = hue_to_rgb(p, q, h / 360.0);
    let b = hue_to_rgb(p, q, h / 360.0 - 1.0 / 3.0);

    (r, g, b, a)
}

fn hue_to_rgb(p: f32, q: f32, t: f32) -> f32 {
    let t = if t < 0.0 {
        t + 1.0
    } else if t > 1.0 {
        t - 1.0
    } else {
        t
    };

    if t < 1.0 / 6.0 {
        p + (q - p) * 6.0 * t
    } else if t < 1.0 / 2.0 {
        q
    } else if t < 2.0 / 3.0 {
        p + (q - p) * (2.0 / 3.0 - t) * 6.0
    } else {
        p
    }
}

#[cfg(test)]
mod hsla_test {
    use crate::util::debug_render::hsla_to_rgb;

    #[test]
    fn test_conversion() {
        assert_eq!(
            hsla_to_rgb([210.0, 0.79, 0.3, 0.5]),
            (0.06299999, 0.3, 0.53700006, 0.5),
        );
        assert_eq!(
            hsla_to_rgb([124.444, 0.44628, 0.47451, 0.794]),
            (0.26274568, 0.68627435, 0.294115, 0.794)
        );
        assert_eq!(
            hsla_to_rgb([38.0, 0.81768, 0.5612, 0.2]),
            (0.919998, 0.6568795, 0.20240206, 0.2)
        );
    }
}
pub struct DebugWindow {
    pub sender: Sender<Frame>,
    pub handle: JoinHandle<()>,
    frame_buffer: Vec<DebugLine>,
}
impl DebugWindow {
    pub fn spawn_debug_window() -> Self {
        let mut path = std::env::current_exe().unwrap();
        path.pop();
        #[cfg(target_os = "windows")]
        path.push("debug-window.exe");
        #[cfg(not(target_os = "windows"))]
        path.push("debug-window");

        let (server, token) =
            IpcOneShotServer::<IpcSender<Frame>>::new().expect("Failed to create one shot server");
        let _child = Command::new(path)
            .arg(token)
            .spawn()
            .expect("Failed to start window process");
        let (_rx, sender) = server.accept().expect("Accept failed");

        let (tx, rx): (Sender<Frame>, Receiver<Frame>) = mpsc::channel();
        let handle = std::thread::spawn(move || {
            let rx = rx;
            loop {
                sender
                    .send(rx.recv().expect("Failed to receive frame on render thread"))
                    .expect("Failed to send frame");
            }
        });
        Self {
            sender: tx,
            handle: handle,
            frame_buffer: Vec::new(),
        }
    }

    pub fn render(&mut self, physics_world: &PhysicsWorld) {
        let mut render =
            DebugRenderPipeline::new(DebugRenderStyle::default(), DebugRenderMode::default());
        render.render(
            self,
            &physics_world.rigid_body_set,
            &physics_world.collider_set,
            &physics_world.impulse_joint_set,
            &physics_world.multibody_joint_set,
            &physics_world.narrow_phase,
        );
        self.sender
            .send(Frame {
                data: self.frame_buffer.clone(),
            })
            .expect("Failed to send frame");
        self.frame_buffer.clear();
    }
}
impl DebugRenderBackend for DebugWindow {
    fn draw_line(&mut self, _object: DebugRenderObject, a: Vector, b: Vector, color: DebugColor) {
        self.frame_buffer.push(DebugLine {
            point1: (a.x, a.y, a.z),
            point2: (b.x, b.y, b.z),
            color: hsla_to_rgb(color),
        });
    }
}
