use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use ipc_channel::ipc::IpcReceiver;
use ipc_types::Message;
use winit::{
    application::ApplicationHandler,
    event::{KeyEvent, StartCause, WindowEvent},
    event_loop::{ActiveEventLoop, ControlFlow},
    keyboard::PhysicalKey,
    window::Window,
};

use crate::state::State;

const FPS: f32 = 30.0;
const FRAME_TIME: f32 = 1.0 / FPS;
pub struct App {
    state: Option<State>,
    render_target: Instant,
    line_receiver: Arc<IpcReceiver<Message>>,
}
impl App {
    pub fn new(line_receiver: IpcReceiver<Message>) -> Self {
        Self {
            state: None,
            render_target: Instant::now(),
            line_receiver: Arc::new(line_receiver),
        }
    }
}
impl ApplicationHandler<State> for App {
    fn resumed(&mut self, event_loop: &winit::event_loop::ActiveEventLoop) {
        let mut window_attributes = Window::default_attributes();

        let window = Arc::new(event_loop.create_window(window_attributes).unwrap());

        self.state =
            Some(pollster::block_on(State::new(window, self.line_receiver.clone())).unwrap());
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
