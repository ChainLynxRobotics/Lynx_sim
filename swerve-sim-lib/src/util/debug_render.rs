use std::{
    process::Command,
    thread,
    time::{Duration, Instant},
};

use ipc_channel::ipc::{self, IpcOneShotServer, IpcReceiver, IpcSender, channel};
use ipc_types::{DebugLine, DebugWindow, Message};
fn convert_hsla_to_rgb(hsla: rapier3d::prelude::DebugColor) -> (f32, f32, f32, f32) {
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

    return (color.0, color.1, color.2, hsla[3]);
}

#[cfg(test)]
mod hsla_test {
    use crate::util::debug_render::convert_hsla_to_rgb;

    #[test]
    fn test_conversion() {
        assert_eq!(
            convert_hsla_to_rgb([210.0, 0.79, 0.3, 0.5]),
            (0.06299999, 0.3, 0.53700006, 0.5)
        );
        assert_eq!(
            convert_hsla_to_rgb([124.444, 0.44628, 0.47451, 0.794]),
            (0.26274568, 0.68627435, 0.294115, 0.794)
        );
        assert_eq!(
            convert_hsla_to_rgb([38.0, 0.81768, 0.5612, 0.2]),
            (0.919998, 0.6568795, 0.20240206, 0.2)
        );
    }
}

pub fn spawn_debug_window() -> DebugWindow {
    let mut path = std::env::current_exe().unwrap();
    path.pop();
    path.push("debug-window");

    let (server, token) =
        IpcOneShotServer::<IpcSender<Message>>::new().expect("Failed to create one shot server");
    let mut child = Command::new(path)
        .arg(token)
        .spawn()
        .expect("Failed to start window process");
    let (_rx, sender) = server.accept().expect("Accept failed");
    let start_time = Instant::now();
    loop {
        let timer = Instant::now().duration_since(start_time).as_nanos() as f64 / 1000000000.0;
        sender.send(Message::StartTransfer).expect("msg");
        sender.send(Message::EndTransfer).expect("msg");
        thread::sleep(Duration::from_secs_f32(1.0 / 60.0));
    }
    return DebugWindow { sender };
}
