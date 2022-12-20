use crate::{physics::VehiclePhysics, pid::PidInit};
use pid::Pid;

#[derive(Debug, Clone)]
pub struct AccelControllerInit {
    pub pid: PidInit,
    pub max_pedal: f64,
}

impl AccelControllerInit {
    pub fn from_physics(physics: &VehiclePhysics) -> Self {
        Self {
            pid: PidInit {
                kp: 0.05,
                ki: 0.0,
                kd: 0.05,
                output_limit: 1.0,
            },
            max_pedal: physics.max_accel().min(physics.max_deceleration()),
        }
    }

    pub fn build(&self) -> AccelController {
        let Self { ref pid, max_pedal } = *self;
        AccelController {
            accel_pid: pid.build(),
            target_accel: 0.0,
            target_pedal: 0.0,
            max_pedal,
        }
    }
}

#[derive(Debug)]
pub struct AccelController {
    accel_pid: Pid<f64>,
    target_accel: f64,
    target_pedal: f64,
    max_pedal: f64,
}

impl AccelController {
    pub fn set_target_accel(&mut self, target_accel: f64) {
        self.target_accel = target_accel;
    }

    pub fn reset_target_pedal(&mut self) {
        self.target_pedal = 0.0;
    }

    pub fn step(
        &mut self,
        current_accel: f64,
        // is_full_stop: bool,
    ) -> AccelControl {
        let Self {
            ref mut accel_pid,
            target_pedal: prev_target_pedal,
            max_pedal,
            target_accel,
        } = *self;

        accel_pid.setpoint = target_accel;
        let pedal_delta = accel_pid.next_control_output(current_accel).output;
        let curr_pedal_target = (prev_target_pedal + pedal_delta).clamp(-max_pedal, max_pedal);
        self.target_pedal = curr_pedal_target;

        AccelControl {
            pedal_target: curr_pedal_target,
            pedal_delta,
        }
    }

    pub fn max_pedal(&self) -> f64 {
        self.max_pedal
    }
}

pub struct AccelControl {
    pub pedal_target: f64,
    pub pedal_delta: f64,
}
