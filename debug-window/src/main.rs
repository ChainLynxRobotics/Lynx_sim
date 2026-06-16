use std::result::Result::Ok;
use std::time::{Duration, Instant};
use std::{env, process};
use std::{sync::Arc, vec};

use ipc_channel::ipc::{IpcReceiver, IpcSender};
use ipc_channel::{IpcError, ipc};
use ipc_types::DebugLine;
use wgpu::PowerPreference::LowPower;
use wgpu::PrimitiveTopology::TriangleList;
use wgpu::util::{BufferInitDescriptor, DeviceExt};
use wgpu::{
    BindGroup, BindGroupDescriptor, BindGroupEntry, BindGroupLayoutDescriptor, BlendState, Buffer,
    BufferUsages, ColorWrites, Features, MultisampleState, ShaderStages, VertexBufferLayout,
    include_wgsl,
};
use winit::event::StartCause;
use winit::event_loop::ControlFlow;
use winit::{
    application::ApplicationHandler,
    event::{KeyEvent, WindowEvent},
    event_loop::{ActiveEventLoop, EventLoop},
    keyboard::{KeyCode, PhysicalKey},
    window::Window,
};

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    position: [f32; 3],
    color: [f32; 3],
}
impl Vertex {
    fn desc() -> VertexBufferLayout<'static> {
        VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x3,
                },
            ],
        }
    }
}
const VERTICES: &[Vertex] = &[
    Vertex {
        position: [2.0, 0.5, 0.5],
        color: [1.0, 0.0, 0.0],
    },
    Vertex {
        position: [2.0, 0.5, -0.5],
        color: [0.0, 1.0, 0.0],
    },
    Vertex {
        position: [2.0, -0.5, -0.5],
        color: [0.0, 0.0, 1.0],
    },
    Vertex {
        position: [1.5, -0.5, 0.5],
        color: [0.0, 0.0, 0.0],
    },
    Vertex {
        position: [2.0, 0.5, 0.5],
        color: [1.0, 0.0, 0.0],
    },
    Vertex {
        position: [2.0, -0.5, -0.5],
        color: [0.0, 0.0, 1.0],
    },
];

#[rustfmt::skip]
pub const OPENGL_TO_WGPU_MATRIX: cgmath::Matrix4<f32> = cgmath::Matrix4::from_cols(
    cgmath::Vector4::new(1.0, 0.0, 0.0, 0.0),
    cgmath::Vector4::new(0.0, 1.0, 0.0, 0.0),
    cgmath::Vector4::new(0.0, 0.0, 0.5, 0.0),
    cgmath::Vector4::new(0.0, 0.0, 0.5, 1.0),
);

struct Camera {
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
#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct CameraUniform {
    // We can't use cgmath with bytemuck directly, so we'll have
    // to convert the Matrix4 into a 4x4 f32 array
    view_proj: [[f32; 4]; 4],
}
impl CameraUniform {
    fn new() -> Self {
        use cgmath::SquareMatrix;
        Self {
            view_proj: cgmath::Matrix4::identity().into(),
        }
    }

    fn update_view_proj(&mut self, camera: &Camera) {
        self.view_proj = camera.build_view_projection_matrix().into();
    }
}

pub struct State {
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    is_surface_configured: bool,
    window: Arc<Window>,
    render_pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    num_vertices: u32,
    camera: Camera,
    camera_uniform: CameraUniform,
    camera_buffer: Buffer,
    camera_controller: CameraController,
    camera_bind_group: BindGroup,
}
impl State {
    pub async fn new(window: Arc<Window>) -> anyhow::Result<Self> {
        let size = window.inner_size();

        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::PRIMARY,
            flags: Default::default(),
            memory_budget_thresholds: Default::default(),
            backend_options: Default::default(),
            display: None,
        });

        let surface = instance.create_surface(window.clone()).unwrap();

        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptionsBase {
                power_preference: LowPower,
                force_fallback_adapter: false,
                compatible_surface: Some(&surface),
            })
            .await?;

        let (device, queue) = adapter
            .request_device(&wgpu::wgt::DeviceDescriptor {
                label: None,
                required_features: Features::POLYGON_MODE_LINE,
                required_limits: wgpu::Limits::defaults(),
                experimental_features: wgpu::ExperimentalFeatures::disabled(),
                memory_hints: Default::default(),
                trace: wgpu::Trace::Off,
            })
            .await?;

        let surface_caps = surface.get_capabilities(&adapter);
        let surface_format = surface_caps
            .formats
            .iter()
            .find(|f| f.is_srgb())
            .copied()
            .unwrap_or(surface_caps.formats[0]);
        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: size.width,
            height: size.height,
            present_mode: surface_caps.present_modes[0],
            desired_maximum_frame_latency: 2,
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![],
        };
        let camera = Camera {
            eye: (0.0, 0.0, 0.0).into(),
            target: (1.0, 0.0, 0.0).into(),
            up: cgmath::Vector3::unit_z(),
            asspect: config.width as f32 / config.height as f32,
            fovy: 45.0,
            znear: 0.01,
            zfar: 1000.0,
        };
        let mut camera_uniform = CameraUniform::new();
        camera_uniform.update_view_proj(&camera);
        let camera_buffer = device.create_buffer_init(&BufferInitDescriptor {
            label: Some("Camera buffer"),
            contents: bytemuck::cast_slice(&[camera_uniform]),
            usage: wgpu::BufferUsages::UNIFORM | BufferUsages::COPY_DST,
        });
        let camera_bind_group_layout =
            device.create_bind_group_layout(&BindGroupLayoutDescriptor {
                label: Some("Camera bind group layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });
        let camera_bind_group = device.create_bind_group(&BindGroupDescriptor {
            label: Some("Camera bind group"),
            layout: &camera_bind_group_layout,
            entries: &[BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
        });

        let shader = device.create_shader_module(include_wgsl!("shader.wgsl"));
        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Render pipeline layout"),
                bind_group_layouts: &[Some(&camera_bind_group_layout)],
                immediate_size: 0,
            });
        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Render pipe line"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                buffers: &[Vertex::desc()],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(BlendState::REPLACE),
                    write_mask: ColorWrites::all(),
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Line,
                conservative: false,
            },
            depth_stencil: None,
            multisample: MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview_mask: None,
            cache: None,
        });

        let vertex_buffer = device.create_buffer_init(&BufferInitDescriptor {
            label: Some("Vertex buffer"),
            contents: bytemuck::cast_slice(VERTICES),
            usage: BufferUsages::VERTEX,
        });

        let num_vertices = VERTICES.len() as u32;

        let camera_controller = CameraController::new(0.2, 0.05);

        Ok(Self {
            surface,
            device,
            queue,
            config,
            is_surface_configured: false,
            window,
            render_pipeline,
            vertex_buffer,
            num_vertices,
            camera,
            camera_uniform,
            camera_buffer,
            camera_bind_group,
            camera_controller,
        })
    }

    pub fn resize(&mut self, width: u32, height: u32) {
        if width > 0 && height > 0 {
            self.config.width = width;
            self.config.height = height;
            self.surface.configure(&self.device, &self.config);
            self.is_surface_configured = true;
        }
    }
    pub fn render(&mut self) -> anyhow::Result<()> {
        if !self.is_surface_configured {
            return Ok(());
        }

        let output = match self.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(surface_texture) => surface_texture,
            wgpu::CurrentSurfaceTexture::Suboptimal(surface_texture) => {
                self.surface.configure(&self.device, &self.config);
                surface_texture
            }
            wgpu::CurrentSurfaceTexture::Timeout
            | wgpu::CurrentSurfaceTexture::Occluded
            | wgpu::CurrentSurfaceTexture::Validation => {
                // skip this frame
                return Ok(());
            }
            wgpu::CurrentSurfaceTexture::Outdated => {
                self.surface.configure(&self.device, &self.config);
                return Ok(());
            }
            wgpu::CurrentSurfaceTexture::Lost => {
                anyhow::bail!("Lost device");
            }
        };

        let view = output
            .texture
            .create_view(&wgpu::wgt::TextureViewDescriptor::default());
        let mut encoder =
            self.device
                .create_command_encoder(&wgpu::wgt::CommandEncoderDescriptor {
                    label: Some("Render encoder"),
                });
        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.1,
                            g: 0.2,
                            b: 0.3,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });

            render_pass.set_pipeline(&self.render_pipeline);
            render_pass.set_bind_group(0, Some(&self.camera_bind_group), &[]);
            render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
            render_pass.draw(0..self.num_vertices, 0..1);
        }

        self.queue.submit(std::iter::once(encoder.finish()));
        output.present();

        Ok(())
    }

    fn handle_key(&mut self, event_loop: &ActiveEventLoop, code: KeyCode, is_pressed: bool) {
        self.camera_controller.handle_key(code, is_pressed);
        match (code, is_pressed) {
            (KeyCode::Escape, true) => event_loop.exit(),
            _ => {}
        }
    }

    fn update(&mut self) {
        self.camera_controller.update_camera(&mut self.camera);
        self.camera_uniform.update_view_proj(&self.camera);
        self.queue.write_buffer(
            &self.camera_buffer,
            0,
            bytemuck::cast_slice(&[self.camera_uniform]),
        );
    }
}

pub struct App {
    state: Option<State>,
    render_target: Instant,
    line_receiver: IpcReceiver<DebugLine>,
}
impl App {
    pub fn new(line_receiver: IpcReceiver<DebugLine>) -> Self {
        Self {
            state: None,
            render_target: Instant::now(),
            line_receiver,
        }
    }
}
impl ApplicationHandler<State> for App {
    fn resumed(&mut self, event_loop: &winit::event_loop::ActiveEventLoop) {
        let mut window_attributes = Window::default_attributes();

        let window = Arc::new(event_loop.create_window(window_attributes).unwrap());

        self.state = Some(pollster::block_on(State::new(window)).unwrap());
    }

    fn user_event(&mut self, event_loop: &winit::event_loop::ActiveEventLoop, event: State) {
        self.state = Some(event);
    }

    fn window_event(
        &mut self,
        event_loop: &winit::event_loop::ActiveEventLoop,
        window_id: winit::window::WindowId,
        event: winit::event::WindowEvent,
    ) {
        let state = match &mut self.state {
            Some(canvas) => canvas,
            None => return,
        };

        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => state.resize(size.width, size.height),
            WindowEvent::RedrawRequested => {
                state.update();
                match state.render() {
                    Ok(_) => {}
                    Err(e) => {
                        log::error!("{e}");
                        event_loop.exit();
                    }
                }

                let now = Instant::now();
                if self.render_target <= now {
                    self.render_target = now + Duration::from_secs_f32(FRAME_TIME);
                    state.window.request_redraw();
                }
            }
            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        physical_key: PhysicalKey::Code(code),
                        state: key_state,
                        ..
                    },
                ..
            } => state.handle_key(event_loop, code, key_state.is_pressed()),
            _ => {}
        }
    }

    fn new_events(&mut self, _: &ActiveEventLoop, _: StartCause) {
        if self.render_target <= Instant::now() {
            self.render_target += Duration::from_secs_f32(FRAME_TIME);
            let state = match &mut self.state {
                Some(canvas) => canvas,
                None => return,
            };
            state.window.request_redraw();
        };
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        // make sure we don't sleep past next render target time
        event_loop.set_control_flow(ControlFlow::WaitUntil(self.render_target));
    }
}

struct CameraController {
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
    fn new(translation_speed: f32, rotation_speed: f32) -> Self {
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

    fn handle_key(&mut self, code: KeyCode, is_pressed: bool) -> bool {
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
            KeyCode::KeyE => {
                self.is_up_pressed = is_pressed;
                true
            }
            KeyCode::KeyQ => {
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

    fn update_camera(&self, camera: &mut Camera) {
        use cgmath::InnerSpace;
        let forward = camera.target - camera.eye;
        let forward_norm = forward.normalize();
        let forward_mag = forward.magnitude();

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

        // Redo radius calc in case the forward/backward is pressed.
        let forward = camera.target - camera.eye;

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

const FPS: f32 = 30.0;
const FRAME_TIME: f32 = 1.0 / FPS;
pub fn run(line_receiver: IpcReceiver<DebugLine>) -> anyhow::Result<()> {
    env_logger::init();

    let event_loop = EventLoop::with_user_event().build()?;
    event_loop.set_control_flow(ControlFlow::Poll);
    let mut app = App::new(line_receiver);

    event_loop.run_app(&mut app)?;

    return Ok(());
}

pub fn main() {
    let args: Vec<String> = env::args().collect();
    let token = args.get(1).expect("missing argument");

    let tx: IpcSender<IpcSender<DebugLine>> =
        IpcSender::connect(token.to_string()).expect("connect failed");
    let (sender, receiver): (IpcSender<DebugLine>, IpcReceiver<DebugLine>) =
        ipc::channel().expect("Failed to make channel");
    tx.send(sender).expect("send failed");

    let line = receiver.recv().unwrap();
    println!("{:?}", line);

    _ = run(receiver).expect("Window failed to spawn");
    println!("After close");
}

fn get_all_lines(line_receiver: &IpcReceiver<DebugLine>) -> anyhow::Result<Vec<DebugLine>> {
    let mut lines: Vec<DebugLine> = Vec::new();
    loop {
        match line_receiver.try_recv() {
            Ok(l) => lines.push(l),
            Err(e) => match e {
                ipc_channel::TryRecvError::IpcError(ipc_error) => return Err(ipc_error.into()),
                ipc_channel::TryRecvError::Empty => break,
            },
        }
    }
    return Ok(lines);
}
