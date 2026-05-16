use rapier3d::{
    dynamics::{ImpulseJointSet, MultibodyJointSet, RigidBodySet},
    geometry::{ColliderSet, NarrowPhase},
    math::{Pose3, Vec3, Vector},
    pipeline::{DebugRenderBackend, DebugRenderMode, DebugRenderPipeline, DebugRenderStyle},
};
use sdl3::{
    EventPump, Sdl,
    event::Event,
    keyboard::Keycode,
    pixels::{self, Color},
    render::{Canvas, FPoint},
    video::Window,
};

const FAR_CLIP: f32 = 10000.0;
const NEAR_CLIP: f32 = 0.0001;

pub struct Camera {
    pub fov: f32,
    pub pose: Pose3,
    pub aspect_ratio: f32,
    pub x_pixels: u32,
    pub y_pixels: u32,
}
impl Camera {
    fn convert_world_coordinates_to_screen_coordates(&self, point: Vector) -> Option<(f32, f32)> {
        let point = Pose3::from_translation(point);
        let inverse_cam = self.pose.inverse();
        let cam_space_point = inverse_cam * point;
        if cam_space_point.translation.x < NEAR_CLIP {
            return None;
        }
        // x is forward so y is x on the screen
        let screen_x: f32 = (cam_space_point.translation.y) * (1.0 / (self.fov / 2.0).tan())
            / cam_space_point.translation.x;
        let screen_y: f32 = (cam_space_point.translation.z)
            * (1.0 / ((self.fov / self.aspect_ratio) / 2.0).tan())
            / cam_space_point.translation.x;

        // convert from a -1 to 1 range to 0 to 1
        let screen_x = -(screen_x / 2.0) + 0.5;
        let screen_y = (-(screen_y / 2.0) / self.aspect_ratio) + 0.5;

        return Some((screen_x, screen_y));
    }
}
fn convert_hsla_to_rgb(hsla: rapier3d::prelude::DebugColor) -> pixels::Color {
    // https://www.baeldung.com/cs/convert-color-hsl-rgb
    let chroma = (1.0 - ((2.0 * hsla[2]) - 1.0).abs()) * hsla[1];
    let h_prime = hsla[0] / 60.0;
    let x = chroma * (1.0 - (h_prime.rem_euclid(2.0) - 1.0).abs());
    if h_prime < 0.0 {
        panic!("Hue out of valid range")
    }
    let color: (f32, f32, f32) = if h_prime < 1.0 {
        (chroma, x, 0.0)
    } else if h_prime < 2.0 {
        (x, chroma, 0.0)
    } else if h_prime < 3.0 {
        (0.0, chroma, x)
    } else if h_prime < 4.0 {
        (0.0, x, chroma)
    } else if h_prime < 5.0 {
        (x, 0.0, chroma)
    } else if h_prime <= 6.0 {
        (chroma, 0.0, x)
    } else {
        panic!("Hue out of valid range")
    };
    let m = hsla[2] - (chroma / 2.0);
    let color = (color.0 + m, color.1 + m, color.2 + m);
    return pixels::Color {
        r: (color.0 * 255.0).round() as u8,
        g: (color.1 * 255.0).round() as u8,
        b: (color.2 * 255.0).round() as u8,
        a: (hsla[3] * 255.0).round() as u8,
    };
}

#[cfg(test)]
mod hsla_test {
    use std::f32::consts::PI;

    use assert_approx_eq::assert_approx_eq;
    use rapier3d::math::{Pose3, Vec3};
    use sdl3::pixels;

    use crate::util::debug_render::{Camera, convert_hsla_to_rgb};

    #[test]
    fn test_conversion() {
        assert_eq!(
            convert_hsla_to_rgb([210.0, 0.79, 0.3, 0.5]),
            pixels::Color::RGBA(16, 77, 137, 128)
        );
        assert_eq!(
            convert_hsla_to_rgb([124.444, 0.44628, 0.47451, 0.794]),
            pixels::Color::RGBA(67, 175, 75, 202)
        );
        assert_eq!(
            convert_hsla_to_rgb([38.0, 0.81768, 0.5612, 0.2]),
            pixels::Color::RGBA(235, 168, 52, 51)
        );
    }
    #[test]
    fn test_inverse_pose() {
        let cam_pose: Pose3 = Pose3::new(
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: PI / -2.0,
            },
        );
        let inv_cam = cam_pose.inverse();
        let point = Pose3::new(Vec3::default(), Vec3::default());
        let point_cam_space = point * inv_cam;
        assert_approx_eq!(point_cam_space.translation.x, 1.0);
        assert_approx_eq!(point_cam_space.translation.y, 0.0);
        assert_approx_eq!(point_cam_space.translation.z, 0.0);

        let cam_pose: Pose3 = Pose3::new(
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );
        let inv_cam = cam_pose.inverse();
        let point = Pose3::new(
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3::default(),
        );
        let point_cam_space = point * inv_cam;
        assert_approx_eq!(point_cam_space.translation.x, 1.0);
        assert_approx_eq!(point_cam_space.translation.y, 0.0);
        assert_approx_eq!(point_cam_space.translation.z, 0.0);

        let cam_pose: Pose3 = Pose3::new(
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: PI / 2.0,
            },
        );
        let inv_cam = cam_pose.inverse();
        let point = Pose3::new(
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3::default(),
        );
        let point_cam_space = inv_cam * point;
        println!("transformed point: {:?}", point_cam_space);
        println!(
            "inv cam: {:?}",
            inv_cam.rotation.to_euler(rapier3d::glamx::EulerRot::XYZ)
        );
        assert_approx_eq!(point_cam_space.translation.x, 0.0);
        assert_approx_eq!(point_cam_space.translation.y, -1.0);
        assert_approx_eq!(point_cam_space.translation.z, 0.0);
    }
    #[test]
    fn test_coordinate_conversion() {
        let mut camera = Camera {
            fov: PI / 2.0,
            pose: Pose3::from_translation(Vec3 {
                x: -2.0,
                y: 0.0,
                z: 0.0,
            }),
            aspect_ratio: 1.0,
            x_pixels: 640,
            y_pixels: 640,
        };
        assert_eq!(
            camera.convert_world_coordinates_to_screen_coordates(Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }),
            Some((0.5, 0.5))
        );
        assert_eq!(
            camera.convert_world_coordinates_to_screen_coordates(Vec3 {
                x: -1.0,
                y: 0.0,
                z: 0.0
            }),
            Some((0.5, 0.5))
        );
        camera.pose = Pose3::new(
            camera.pose.translation,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: PI / -4.0,
            },
        );
        let point = camera
            .convert_world_coordinates_to_screen_coordates(Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            })
            .unwrap();
        assert_approx_eq!(point.0, 0.0);
        assert_approx_eq!(point.1, 0.5);
        camera.pose = Pose3::new(Vec3::default(), Vec3::default());
        let point = camera
            .convert_world_coordinates_to_screen_coordates(Vec3 {
                x: 1.0,
                y: 0.0,
                z: 1.0,
            })
            .unwrap();
        assert_approx_eq!(point.0, 0.5);
        assert_approx_eq!(point.1, 0.0);

        camera.pose = Pose3::new(
            Vec3::default(),
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: PI / -4.0,
            },
        );
        let point = camera
            .convert_world_coordinates_to_screen_coordates(Vec3 {
                x: 1.0,
                y: 0.0,
                z: 1.0,
            })
            .unwrap();
        // assert_approx_eq!(point.0, 0.0);
        // assert_approx_eq!(point.1, 0.0);
    }
}

pub struct DebugWindow {
    pub canvas: Canvas<Window>,
    pub event_pump: EventPump,
    pub camera: Camera,
}
impl DebugRenderBackend for DebugWindow {
    fn draw_line(
        &mut self,
        object: rapier3d::prelude::DebugRenderObject,
        a: rapier3d::prelude::Vector,
        b: rapier3d::prelude::Vector,
        color: rapier3d::prelude::DebugColor,
    ) {
        let a_screenspace = match self.camera.convert_world_coordinates_to_screen_coordates(a) {
            Some(p) => p,
            None => return,
        };
        let b_screenspace = match self.camera.convert_world_coordinates_to_screen_coordates(b) {
            Some(p) => p,
            None => return,
        };
        // println!(
        //     "point a x: {:?}\npoint a y: {:?}",
        //     a_screenspace.0 * self.camera.x_pixels as f32,
        //     a_screenspace.1 * self.camera.y_pixels as f32
        // );
        // println!(
        //     "point b x: {:?}\npoint b y: {:?}",
        //     b_screenspace.0 * self.camera.x_pixels as f32,
        //     b_screenspace.1 * self.camera.y_pixels as f32,
        // );
        // println!("color: {:?}", convert_hsla_to_rgb(color));
        // println!();
        self.canvas.set_draw_color(convert_hsla_to_rgb(color));
        self.canvas
            .draw_line(
                FPoint::new(
                    a_screenspace.0 * self.camera.x_pixels as f32,
                    a_screenspace.1 * self.camera.y_pixels as f32,
                ),
                FPoint::new(
                    b_screenspace.0 * self.camera.x_pixels as f32,
                    b_screenspace.1 * self.camera.y_pixels as f32,
                ),
            )
            .unwrap();
    }
}

pub fn spawn_debug_window() -> (Canvas<Window>, Sdl) {
    let sdl_context = sdl3::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();

    let window = video_subsystem
        .window("swerve sim debug window", 800, 600)
        .position_centered()
        .build()
        .unwrap();

    return (window.into_canvas(), sdl_context);
}

pub fn draw_debug_window(
    window: Option<&mut DebugWindow>,
    bodies: &RigidBodySet,
    colliders: &ColliderSet,
    impulse_joints: &ImpulseJointSet,
    multibody_joints: &MultibodyJointSet,
    narrow_phase: &NarrowPhase,
) {
    let window = match window {
        None => return,
        Some(window) => window,
    };
    let mut render =
        DebugRenderPipeline::new(DebugRenderStyle::default(), DebugRenderMode::default());
    window.canvas.set_draw_color(Color::CYAN);
    window.canvas.clear();
    render.render(
        window,
        bodies,
        colliders,
        impulse_joints,
        multibody_joints,
        narrow_phase,
    );
    for event in window.event_pump.poll_iter() {
        match event {
            Event::Quit { .. }
            | Event::KeyDown {
                keycode: Some(Keycode::Escape),
                ..
            } => panic!("uhh close the app i guess"),
            Event::KeyDown {
                keycode: Some(Keycode::W),
                ..
            } => {
                window.camera.pose = window.camera.pose.append_translation(Vec3 {
                    x: 0.01,
                    y: 0.0,
                    z: 0.0,
                })
            }
            Event::KeyDown {
                keycode: Some(Keycode::S),
                ..
            } => {
                window.camera.pose = window.camera.pose.append_translation(Vec3 {
                    x: -0.01,
                    y: 0.0,
                    z: 0.0,
                })
            }
            Event::KeyDown {
                keycode: Some(Keycode::A),
                ..
            } => {
                window.camera.pose = window.camera.pose.append_translation(Vec3 {
                    x: 0.0,
                    y: 0.01,
                    z: 0.0,
                })
            }
            Event::KeyDown {
                keycode: Some(Keycode::D),
                ..
            } => {
                window.camera.pose = window.camera.pose.append_translation(Vec3 {
                    x: 0.0,
                    y: -0.01,
                    z: 0.0,
                })
            }
            Event::KeyDown {
                keycode: Some(Keycode::Q),
                ..
            } => {
                window.camera.pose = window.camera.pose.append_translation(Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: -0.01,
                })
            }
            Event::KeyDown {
                keycode: Some(Keycode::E),
                ..
            } => {
                window.camera.pose = window.camera.pose.append_translation(Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.01,
                })
            }
            Event::KeyDown {
                keycode: Some(Keycode::Left),
                ..
            } => {
                let current_rot = window
                    .camera
                    .pose
                    .rotation
                    .to_euler(rapier3d::glamx::EulerRot::XYZ);
                window.camera.pose = Pose3::new(
                    window.camera.pose.translation,
                    Vec3 {
                        x: current_rot.0,
                        y: current_rot.1,
                        z: current_rot.2 + 0.05,
                    },
                )
            }
            Event::KeyDown {
                keycode: Some(Keycode::Right),
                ..
            } => {
                let current_rot = window
                    .camera
                    .pose
                    .rotation
                    .to_euler(rapier3d::glamx::EulerRot::XYZ);
                window.camera.pose = Pose3::new(
                    window.camera.pose.translation,
                    Vec3 {
                        x: current_rot.0,
                        y: current_rot.1,
                        z: current_rot.2 - 0.05,
                    },
                )
            }
            Event::KeyDown {
                keycode: Some(Keycode::Up),
                ..
            } => {
                let current_rot = window
                    .camera
                    .pose
                    .rotation
                    .to_euler(rapier3d::glamx::EulerRot::XYZ);
                window.camera.pose = Pose3::new(
                    window.camera.pose.translation,
                    Vec3 {
                        x: current_rot.0,
                        y: current_rot.1 + 0.05,
                        z: current_rot.2,
                    },
                )
            }
            Event::KeyDown {
                keycode: Some(Keycode::Down),
                ..
            } => {
                let current_rot = window
                    .camera
                    .pose
                    .rotation
                    .to_euler(rapier3d::glamx::EulerRot::XYZ);
                window.camera.pose = Pose3::new(
                    window.camera.pose.translation,
                    Vec3 {
                        x: current_rot.0,
                        y: current_rot.1 - 0.05,
                        z: current_rot.2,
                    },
                )
            }
            _ => {}
        }
    }
    window.canvas.present();
}
