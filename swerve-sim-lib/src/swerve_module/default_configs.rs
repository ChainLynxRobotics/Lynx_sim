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
        0.0,
        0.0,
        0.0,
    );

    pub const fn get_mk4i(_gear_ratio: Mk4iGearRatio, _wheel: Mk4iWheel) -> Self {
        todo!()
    }
}

struct WheelProperties {
    radius: unit!(m, f32),
    width: unit!(m, f32),
    mass: unit!(kg, f32),
    wheel_moi: unit!(kg * m ^ 2, f32),
    wheel_secondary_moi: unit!(kg * m ^ 2, f32),
    cof: f32,
    coefficient_of_restituation: f32,
}
struct GearTrainProperties {
    drive_moi: unit!(kg * m ^ 2, f32),
    azumith_moi: unit!(kg * m ^ 2, f32),
    azumith_secondary_moi: unit!(kg * m ^ 2, f32),
    azumith_mass: unit!(kg, f32),
    azumith_center_height: unit!(m, f32),
    azumith_thickness: unit!(m, f32),
    azumith_radius: unit!(m, f32),
    wheel_center_height: unit!(m, f32),
    drive_gear_ratio: f32,
    turn_gear_ratio: f32,
    coupling_ratio: f32,
}

pub enum Mk4iGearRatio {
    L1,
    L1Plus,
    L2,
    L2Plus,
    L3,
    L3Plus,
}
impl Mk4iGearRatio {
    const fn generate_gear_train_properties(&self) -> GearTrainProperties {
        let azumith_moi = quantity!(0.000763062857, kg * m ^ 2, f32);
        let azumith_secondary_moi = quantity!(0.0004489, kg * m ^ 2, f32);
        let azumith_mass = quantity!(0.41940695, kg, f32);
        let azumith_center_height = quantity!(0.061, m, f32);
        let azumith_thickness = quantity!(0.02708200, m, f32);
        let azumith_radius = quantity!(0.0492125, m, f32);

        let turn_gear_ratio: f32 = 150.0 / 7.0;
        let coupling_ratio: f32 = 50.0 / 14.0;
        let wheel_center_height = quantity!(-0.04133150, m, f32);

        let drive_moi = match self {
            Mk4iGearRatio::L1 => todo!(),
            Mk4iGearRatio::L1Plus => todo!(),
            Mk4iGearRatio::L2 => todo!(),
            // 0.00002277 + (0.00000319) * 3 + (0.00007443) * (17/27) * 3 + (0.00005322) * 1 * (17/27) * 3 + (0.0000024)* (50/16) * 1 * (17/27) * 3
            Mk4iGearRatio::L2Plus => quantity!(0.000287623333333, kg * m ^ 2, f32),
            Mk4iGearRatio::L3 => todo!(),
            Mk4iGearRatio::L3Plus => todo!(),
        };
        let drive_gear_ratio: f32 = match self {
            Mk4iGearRatio::L1 => todo!(),
            Mk4iGearRatio::L1Plus => todo!(),
            Mk4iGearRatio::L2 => todo!(),
            Mk4iGearRatio::L2Plus => (50.0 / 16.0) * 1.0 * (17.0 / 27.0) * 3.0,
            Mk4iGearRatio::L3 => todo!(),
            Mk4iGearRatio::L3Plus => todo!(),
        };

        return GearTrainProperties {
            drive_moi,
            azumith_moi,
            azumith_secondary_moi,
            azumith_mass,
            azumith_center_height,
            azumith_thickness,
            azumith_radius,
            wheel_center_height,
            drive_gear_ratio,
            turn_gear_ratio,
            coupling_ratio,
        };
    }
}

pub enum Mk4iWheel {
    Billet,
    Colson,
}
impl Mk4iWheel {
    const fn generate_wheel_properties(&self) -> WheelProperties {
        let radius = quantity!(0.0508, m, f32);
        let width = quantity!(0.03810000, m, f32);
        let mass = match self {
            Mk4iWheel::Billet => quantity!(0.22631947, kg, f32),
            Mk4iWheel::Colson => quantity!(0.31121417, kg, f32),
        };
        let wheel_moi = match self {
            Mk4iWheel::Billet => quantity!(0.00022151, kg * m ^ 2, f32),
            Mk4iWheel::Colson => quantity!(0.00032068, kg * m ^ 2, f32),
        };
        let wheel_secondary_moi = match self {
            Mk4iWheel::Billet => quantity!(0.00014287, kg * m ^ 2, f32),
            Mk4iWheel::Colson => quantity!(0.00019662, kg * m ^ 2, f32),
        };
        // https://www.chiefdelphi.com/t/wildstang-robotics-program-team-111-and-112-build-blog-2025/477716/36
        let cof = match self {
            Mk4iWheel::Billet => 1.1f32,
            Mk4iWheel::Colson => 0.8f32,
        };
        // just a random guess
        let coefficient_of_restituation = 0.0f32;
        return WheelProperties {
            radius,
            width,
            mass,
            wheel_moi,
            wheel_secondary_moi,
            cof,
            coefficient_of_restituation,
        };
    }
}

pub fn generate_mk4i_swerve_config(
    gear_ratio: Mk4iGearRatio,
    wheel: Mk4iWheel,
) -> SwerveModuleConfig {
    let gear_train_properties = gear_ratio.generate_gear_train_properties();
    let wheel_properties = wheel.generate_wheel_properties();
    return SwerveModuleConfig {
        wheel_radius: wheel_properties.radius,
        wheel_width: wheel_properties.width,
        wheel_mass: wheel_properties.mass,
        drive_moi: wheel_properties.wheel_moi + gear_train_properties.drive_moi,
        wheel_secondary_moi: wheel_properties.wheel_secondary_moi,
        azumith_moi: gear_train_properties.azumith_moi,
        azumith_secondary_moi: gear_train_properties.azumith_secondary_moi,
        azumith_mass: gear_train_properties.azumith_mass,
        azumith_center_height: gear_train_properties.azumith_center_height,
        azumith_thickness: gear_train_properties.azumith_thickness,
        azumith_radius: gear_train_properties.azumith_radius,
        wheel_center_height: gear_train_properties.wheel_center_height,
        wheel_cof: wheel_properties.cof,
        wheel_coefficient_of_restetution: wheel_properties.coefficient_of_restituation,
        drive_gear_ratio: gear_train_properties.drive_gear_ratio,
        turn_gear_ratio: gear_train_properties.turn_gear_ratio,
        coupling_ratio: gear_train_properties.coupling_ratio,
    };
}
