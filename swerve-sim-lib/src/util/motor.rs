// Copyright (c) FIRST and other WPILib contributors.

// translation of the importiant parts of
// https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/org/wpilib/math/system/DCMotor.java

use whippyunits::{quantity, unit};

struct Motor {
    pub nominal_voltage: unit!(volt, f32),
    pub stall_torque: unit!(newton / m, f32),
    pub stall_current: unit!(ampere, f32),
    pub free_current: unit!(ampere, f32),
    pub free_speed: unit!(rad / s, f32),
    pub internal_resistance: unit!(ohm, f32),
    pub kv: unit!((rad / s) / volt, f32),
    pub kt: unit!((newton / m) / ampere, f32),
}
impl Motor {
    fn new(
        nominal_voltage: unit!(volt, f32),
        stall_torque: unit!(newton / m, f32),
        stall_current: unit!(ampere, f32),
        free_current: unit!(ampere, f32),
        free_speed: unit!(rad / s, f32),
    ) -> Self {
        let internal_resistance = nominal_voltage / stall_current;
        Self {
            nominal_voltage,
            stall_torque,
            stall_current,
            free_current,
            free_speed,
            internal_resistance,
            kv: free_speed / (nominal_voltage - (internal_resistance * free_current)),
            kt: stall_torque / stall_current,
        }
    }
    /// Calculate current drawn by motor with given velocity and input voltage.
    ///
    /// # Parameters
    /// * `velocity` The current angular velocity of the motor.
    /// * `voltageInput` The voltage being applied to the motor.
    pub fn get_current(
        &self,
        velocity: unit!(rad / s, f32),
        voltage_input: unit!(volt, f32),
    ) -> unit!(ampere, f32) {
        return -1.0 / self.kv / self.internal_resistance * velocity
            + 1.0 / self.internal_resistance * voltage_input;
    }

    /// Calculate current drawn by motor for a given torque.
    ///
    /// # Parameters
    /// * `torque` The torque produced by the motor.
    pub fn get_current_from_torque(self, torque: unit!(newton / m, f32)) -> unit!(ampere, f32) {
        return torque / self.kt;
    }

    /// Calculate torque produced by the motor with a given current.
    ///
    /// # Parameters
    /// * `current` The current drawn by the motor.
    pub fn get_torque(self, current: unit!(ampere, f32)) -> unit!(newton / m, f32) {
        return current * self.kt;
    }

    /// Calculate the voltage provided to the motor for a given torque and angular velocity.
    ///
    /// # Parameter
    /// * `torque` The torque produced by the motor.
    /// * `velocity` The current angular velocity of the motor.
    pub fn get_voltage(
        self,
        torque: unit!(newton / m, f32),
        velocity: unit!(rad / s, f32),
    ) -> unit!(volt, f32) {
        return 1.0 / self.kv * velocity + 1.0 / self.kt * self.internal_resistance * torque;
    }

    /// Calculates the angular velocity produced by the motor at a given torque and input voltage.
    ///
    /// # Parameters
    /// * `torque` The torque produced by the motor.
    /// * `voltageInput` The voltage applied to the motor.
    pub fn get_velocity(
        self,
        torque: unit!(newton / m, f32),
        voltage_input: unit!(volt, f32),
    ) -> unit!(rad / s, f32) {
        return voltage_input * self.kv
            - 1.0 / self.kt * torque * self.internal_resistance * self.kv;
    }
}
