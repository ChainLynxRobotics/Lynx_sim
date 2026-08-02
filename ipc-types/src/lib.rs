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

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize, PartialEq)]
pub struct Frame {
    pub data: Vec<DebugLine>,
}

pub const FRAME_RATE: f32 = 60.0;
