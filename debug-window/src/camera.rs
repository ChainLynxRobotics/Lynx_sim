use winit::keyboard::KeyCode;

#[rustfmt::skip]
pub const OPENGL_TO_WGPU_MATRIX: cgmath::Matrix4<f32> = cgmath::Matrix4::from_cols(
    cgmath::Vector4::new(1.0, 0.0, 0.0, 0.0),
    cgmath::Vector4::new(0.0, 1.0, 0.0, 0.0),
    cgmath::Vector4::new(0.0, 0.0, 0.5, 0.0),
    cgmath::Vector4::new(0.0, 0.0, 0.5, 1.0),
);

pub struct Camera {
    eye: cgmath::Point3<f32>,
    target: cgmath::Point3<f32>,
    up: cgmath::Vector3<f32>,
    asspect: f32,
    fovy: f32,
    znear: f32,
    zfar: f32,
}
impl Camera {
    fn build_view_projection_matrix(&self) -> cgmath::Matrix4<f32> {
        let view = cgmath::Matrix4::look_at_rh(self.eye, self.target, self.up);
        let proj = cgmath::perspective(cgmath::Deg(self.fovy), self.asspect, self.znear, self.zfar);

        return OPENGL_TO_WGPU_MATRIX * proj * view;
    }
}
impl Camera {
    pub fn new(width: u32, height: u32) -> Self {
        Camera {
            eye: (-2.0, 0.0, -1.0).into(),
            target: (-1.0, 0.0, -1.0).into(),
            up: cgmath::Vector3::unit_z(),
            asspect: (width as f32) / (height as f32),
            fovy: 45.0,
            znear: 0.01,
            zfar: 1000.0,
        }
    }
}
#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniform {
    // We can't use cgmath with bytemuck directly, so we'll have
    // to convert the Matrix4 into a 4x4 f32 array
    view_proj: [[f32; 4]; 4],
}
impl CameraUniform {
    pub fn new() -> Self {
        use cgmath::SquareMatrix;
        Self {
            view_proj: cgmath::Matrix4::identity().into(),
        }
    }

    pub fn update_view_proj(&mut self, camera: &Camera) {
        self.view_proj = camera.build_view_projection_matrix().into();
    }
}

pub struct CameraController {
    translation_speed: f32,
    rotation_speed: f32,
    is_forward_pressed: bool,
    is_backward_pressed: bool,
    is_left_pressed: bool,
    is_right_pressed: bool,
    is_up_pressed: bool,
    is_down_pressed: bool,
    is_yaw_right_pressed: bool,
    is_yaw_left_pressed: bool,
    is_pitch_up_pressed: bool,
    is_pitch_down_pressed: bool,
}

impl CameraController {
    pub fn new(translation_speed: f32, rotation_speed: f32) -> Self {
        Self {
            translation_speed,
            rotation_speed,
            is_forward_pressed: false,
            is_backward_pressed: false,
            is_left_pressed: false,
            is_right_pressed: false,
            is_up_pressed: false,
            is_down_pressed: false,
            is_yaw_right_pressed: false,
            is_yaw_left_pressed: false,
            is_pitch_up_pressed: false,
            is_pitch_down_pressed: false,
        }
    }

    pub fn handle_key(&mut self, code: KeyCode, is_pressed: bool) -> bool {
        match code {
            KeyCode::KeyW => {
                self.is_forward_pressed = is_pressed;
                true
            }
            KeyCode::KeyA => {
                self.is_left_pressed = is_pressed;
                true
            }
            KeyCode::KeyS => {
                self.is_backward_pressed = is_pressed;
                true
            }
            KeyCode::KeyD => {
                self.is_right_pressed = is_pressed;
                true
            }
            KeyCode::KeyE | KeyCode::Space => {
                self.is_up_pressed = is_pressed;
                true
            }
            KeyCode::KeyQ | KeyCode::ShiftLeft | KeyCode::ShiftRight => {
                self.is_down_pressed = is_pressed;
                true
            }
            KeyCode::ArrowLeft => {
                self.is_yaw_left_pressed = is_pressed;
                true
            }
            KeyCode::ArrowRight => {
                self.is_yaw_right_pressed = is_pressed;
                true
            }
            KeyCode::ArrowUp => {
                self.is_pitch_up_pressed = is_pressed;
                true
            }
            KeyCode::ArrowDown => {
                self.is_pitch_down_pressed = is_pressed;
                true
            }
            _ => false,
        }
    }

    pub fn update_camera(&self, camera: &mut Camera) {
        use cgmath::InnerSpace;
        let forward = camera.target - camera.eye;
        let forward_norm = forward.normalize();

        let right = forward_norm.cross(camera.up).normalize();

        // Prevents glitching when the camera gets too close to the
        // center of the scene.
        if self.is_forward_pressed {
            camera.eye += forward_norm * self.translation_speed;
            camera.target += forward_norm * self.translation_speed;
        }
        if self.is_backward_pressed {
            camera.eye -= forward_norm * self.translation_speed;
            camera.target -= forward_norm * self.translation_speed;
        }
        if self.is_left_pressed {
            camera.eye -= right * self.translation_speed;
            camera.target -= right * self.translation_speed;
        }
        if self.is_right_pressed {
            camera.eye += right * self.translation_speed;
            camera.target += right * self.translation_speed;
        }
        if self.is_up_pressed {
            camera.eye += camera.up.normalize() * self.translation_speed;
            camera.target += camera.up.normalize() * self.translation_speed;
        }
        if self.is_down_pressed {
            camera.eye -= camera.up.normalize() * self.translation_speed;
            camera.target -= camera.up.normalize() * self.translation_speed;
        }

        if self.is_yaw_right_pressed {
            camera.target = camera.target + right.normalize() * self.rotation_speed;
        }
        if self.is_yaw_left_pressed {
            camera.target = camera.target - right.normalize() * self.rotation_speed;
        }
        if self.is_pitch_up_pressed {
            camera.target = camera.target + camera.up.normalize() * self.rotation_speed;
        }
        if self.is_pitch_down_pressed {
            camera.target = camera.target - camera.up.normalize() * self.rotation_speed;
        }
    }
}
