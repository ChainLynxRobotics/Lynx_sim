#[derive(Debug, Clone, Copy, serde::Deserialize, serde::Serialize, PartialEq)]
pub struct DebugLine {
    pub point1: (f32, f32, f32),
    pub point2: (f32, f32, f32),
    pub color: (f32, f32, f32, f32),
}
#[derive(Debug, Clone, Copy, serde::Deserialize, serde::Serialize, PartialEq)]
pub enum Message {
    Line(DebugLine),
    StartTransfer,
    EndTransfer,
}
