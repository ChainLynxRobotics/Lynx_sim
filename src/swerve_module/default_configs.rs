use super::config::SwerveModuleConfig;
use culit::culit;
use whippyunits::{quantity, unit};

#[culit(whippyunits::default_declarators::literals)]
impl SwerveModuleConfig {
    pub const ZERO: Self = Self::new(
        0.0m_f32,
        0.0m_f32,
        0.0kg_f32,
        quantity!(0.0, kg * m ^ 2, f32),
        quantity!(0.0, kg * m ^ 2, f32),
        quantity!(0.0, kg * m ^ 2, f32),
        quantity!(0.0, kg * m ^ 2, f32),
        0.0kg_f32,
        0.0m_f32,
        0.0m_f32,
        0.0m_f32,
        0.0m_f32,
        0.0,
        0.0,
    );
}
