use crate::{
    constants::{FULL_STOP_SPEED_MS, INTERNAL_ACCEL_MS2, STAND_STILL_SPEED_MS},
    physics::VehiclePhysics,
    pid::PidInit,
};
use pid::Pid;

#[derive(Debug, Clone)]
pub struct SpeedControllerInit {
    pub pid: PidInit,
    pub max_speed: f64,
    pub max_accel: f64,
    pub min_accel: f64,
    pub max_decel: f64,
}

impl SpeedControllerInit {
    pub fn from_physics(physics: &VehiclePhysics, min_accel: Option<f64>) -> Self {
        Self {
            pid: PidInit {
                kp: 0.05,
                ki: 0.0,
                kd: 0.5,
                output_limit: 1.0,
            },
            max_speed: physics.max_speed(),
            max_accel: physics.max_accel(),
            min_accel: min_accel.unwrap_or(1.0),
            max_decel: physics.max_deceleration(),
        }
    }

    pub fn build(&self) -> SpeedController {
        let Self {
            ref pid,
            max_speed,
            max_accel,
            min_accel,
            max_decel,
        } = *self;

        SpeedController {
            speed_pid: pid.build(),
            accel_activator: DelayedActivator::new(5),
            target_speed: 0.0,
            target_accel: 0.0,
            max_speed,
            max_accel,
            min_accel,
            max_decel,
        }
    }
}

#[derive(Debug)]
pub struct SpeedController {
    speed_pid: Pid<f64>,
    accel_activator: DelayedActivator,
    target_speed: f64,
    target_accel: f64,
    max_speed: f64,
    max_accel: f64,
    min_accel: f64,
    max_decel: f64,
}

impl SpeedController {
    pub fn target_speed(&self) -> f64 {
        self.target_speed
    }

    pub fn set_target(&mut self, target_speed: f64, target_accel: f64) {
        let Self {
            max_speed,
            max_accel,
            max_decel,
            ..
        } = *self;
        let target_speed = target_speed.clamp(-max_speed, max_speed);
        let target_accel = if target_speed.abs() >= FULL_STOP_SPEED_MS {
            target_accel.clamp(-max_decel, max_accel)
        } else {
            -max_decel
        };

        self.target_speed = target_speed;
        self.target_accel = target_accel;
    }

    pub fn step(&mut self, current_speed: f64) -> SpeedControl {
        let Self {
            ref mut speed_pid,
            // ref mut accel_activator,
            target_speed,
            target_accel,
            // min_accel,
            max_accel,
            max_decel,
            ..
        } = *self;

        let is_standing = current_speed.abs() < STAND_STILL_SPEED_MS;
        let is_stopping = target_speed.abs() < FULL_STOP_SPEED_MS;
        let is_full_stop = is_standing && is_stopping;

        let setpoint_speed = match (is_standing, is_stopping) {
            (true, true) => 0.0,
            (true, false) => target_speed,
            _ => {
                if current_speed.is_sign_positive() ^ target_speed.is_sign_positive() {
                    0.0
                } else {
                    target_speed
                }
            }
        };

        let target_accel_abs = target_accel.abs();
        let is_inertial = target_accel_abs < INTERNAL_ACCEL_MS2;
        // let is_speed_control_enabled = {
        //     let is_accel_triggered = !is_inertial && target_accel_abs >= min_accel;

        //     if is_accel_triggered {
        //         accel_activator.inc()
        //     } else {
        //         accel_activator.dec();
        //         false
        //     }
        // };
        let is_speed_control_enabled = true;

        let (setpoint_accel, delta_accel) = if is_speed_control_enabled {
            speed_pid.setpoint = setpoint_speed.abs();
            let delta = speed_pid.next_control_output(current_speed).output;

            let (lower, upper) = if is_inertial {
                (-max_decel, max_accel)
            } else {
                (-target_accel_abs, target_accel_abs)
            };

            let prev_target = if is_full_stop { 0.0 } else { target_accel };
            let target = (prev_target + delta).clamp(lower, upper);
            (target, delta)
        } else {
            (target_accel, 0.0)
        };

        SpeedControl {
            setpoint_accel,
            delta_accel,
            full_stop: is_full_stop,
        }
    }
}

pub struct SpeedControl {
    pub setpoint_accel: f64,
    pub delta_accel: f64,
    pub full_stop: bool,
}

#[derive(Debug)]
struct DelayedActivator {
    max: usize,
    cur: usize,
}

impl DelayedActivator {
    pub fn new(max: usize) -> Self {
        Self { max, cur: 0 }
    }

    pub fn inc(&mut self) -> bool {
        let Self { max, cur } = *self;
        let next = if max == cur { max } else { cur + 1 };
        self.cur = next;
        next == max
    }

    pub fn dec(&mut self) {
        if let Some(next) = self.cur.checked_sub(1) {
            self.cur = next;
        }
    }
}
