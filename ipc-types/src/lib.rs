use ipc_channel::ipc::IpcSender;

#[derive(Debug, Clone, Copy, serde::Deserialize, serde::Serialize)]
pub struct DebugLine {
    point1: (f32, f32, f32),
    point2: (f32, f32, f32),
    color: (f32, f32, f32, f32),
}
pub struct DebugWindow {
    sender: IpcSender<DebugLine>,
}
