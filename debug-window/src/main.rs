use std::env;
use std::result::Result::Ok;

use ipc_channel::ipc;
use ipc_channel::ipc::{IpcReceiver, IpcSender};
use ipc_types::Message;
use winit::event_loop::ControlFlow;
use winit::event_loop::EventLoop;

use crate::app::App;

mod app;
mod camera;
mod state;
mod vertex;

pub fn run(line_receiver: IpcReceiver<Message>) -> anyhow::Result<()> {
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

    let tx: IpcSender<IpcSender<Message>> =
        IpcSender::connect(token.to_string()).expect("connect failed");
    let (sender, receiver): (IpcSender<Message>, IpcReceiver<Message>) =
        ipc::channel().expect("Failed to make channel");
    tx.send(sender).expect("send failed");

    _ = run(receiver).expect("Window failed to spawn");
    println!("After close");
}
