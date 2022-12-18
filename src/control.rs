use carla::rpc::VehiclePhysicsControl;
use pid::Pid;

use crate::{
    constants::{FULL_STOP_SPEED_MS, INTERNAL_ACCEL_MS2, STAND_STILL_SPEED_MS},
    physics::VehiclePhysics,
};

#[derive(Debug, Clone)]
pub struct Restrictions {
    pub max_steering_angle: f64,
    pub max_speed: f64,
    pub max_accel: f64,
    pub min_accel: f64,
    pub max_decel: f64,
    pub max_pedal: f64,
}

#[derive(Debug, Clone)]
pub struct Target {
    pub steering_angle: f64,
    pub speed: f64,
    pub accel: f64,
    pub jerk: f64,
}

#[derive(Debug, Clone)]
pub struct Current {
    pub time_sec: f64,
    pub speed: f64,
    pub accel: f64,
}

#[derive(Debug, Clone)]
pub struct Status {
    pub status: Option<StatusKind>,
    pub accel_target: f64,
    pub pedal_target: f64,
}

#[derive(Debug, Clone)]
pub struct Report {
    pub status: StatusKind,
    pub accel_target: f64,
    pub pedal_target: f64,
    pub accel_delta: f64,
    pub pedal_delta: f64,
}

#[derive(Debug, Clone)]
pub struct Output {
    pub throttle: f64,
    pub brake: f64,
    pub steer: f64,
    pub reverse: bool,
    pub hand_brake: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StatusKind {
    FullStop,
    Accelerating,
    Coasting,
    Braking,
}

impl Default for Target {
    fn default() -> Self {
        Self {
            steering_angle: 0.0,
            speed: 0.0,
            accel: 0.0,
            jerk: 0.0,
        }
    }
}

impl Default for Current {
    fn default() -> Self {
        Self {
            time_sec: 0.0,
            speed: 0.0,
            accel: 0.0,
        }
    }
}

impl Default for Status {
    fn default() -> Self {
        Self {
            status: None,
            accel_target: 0.0,
            pedal_target: 0.0,
        }
    }
}

impl Default for Output {
    fn default() -> Self {
        Self {
            throttle: 0.0,
            brake: 1.0,
            steer: 0.0,
            reverse: false,
            hand_brake: true,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Info {
    target: Target,
    restrictions: Restrictions,
    current: Current,
    status: Status,
}

#[derive(Debug)]
pub struct Controller {
    info: Info,
    speed_pid: Pid<f64>,
    accel_pid: Pid<f64>,
    accel_activator: DelayedActivator,
    physics_control: VehiclePhysicsControl,
}

impl Controller {
    pub fn set_target(&mut self, target: Target) {
        let Info {
            restrictions:
                Restrictions {
                    max_steering_angle: max_steer,
                    max_speed,
                    max_accel,
                    max_decel,
                    ..
                },
            target:
                Target {
                    steering_angle: target_steer,
                    speed: target_speed,
                    accel: target_accel,
                    ..
                },
            ..
        } = self.info;

        let steering_angle = (-target_steer).clamp(-max_steer, max_steer);
        let speed = target_speed.clamp(-max_speed, max_speed);
        let speed_abs = speed.abs();
        let accel = if speed_abs >= FULL_STOP_SPEED_MS {
            target_accel.clamp(-max_decel, max_accel)
        } else {
            -max_decel
        };

        self.info.target = Target {
            steering_angle,
            speed,
            accel,
            ..target
        };
    }

    pub fn step(&mut self, time_delta_sec: f64, current_speed: f64, pitch_radians: f64) -> Output {
        assert!(time_delta_sec > 0.0);

        let Self {
            info:
                Info {
                    target,
                    restrictions,
                    current: previous,
                    status,
                    ..
                },
            speed_pid,
            accel_pid,
            accel_activator,
            physics_control,
        } = self;

        let current = {
            let speed_delta = current_speed - previous.speed;
            let current_accel = speed_delta / time_delta_sec;

            Current {
                time_sec: previous.time_sec + time_delta_sec,
                speed: current_speed,
                accel: current_accel,
            }
        };

        let speed_kind = SpeedKind::from_speed_ms(current.speed);
        let reverse = target.speed < 0.0;
        let steer = target.steering_angle / restrictions.max_steering_angle;

        let current = if speed_kind == SpeedKind::FullStop {
            Current {
                speed: 0.0,
                accel: 0.0,
                ..current
            }
        } else {
            current
        };

        let target = {
            let target_speed = match speed_kind {
                SpeedKind::FullStop => 0.0,
                SpeedKind::StandStill => target.speed,
                SpeedKind::Driving => {
                    if current.speed.is_sign_positive() ^ target.speed.is_sign_positive() {
                        0.0
                    } else {
                        target.speed
                    }
                }
            };

            Target {
                speed: target_speed,
                ..*target
            }
        };
        let accel_target = {
            let prev_target = if speed_kind == SpeedKind::FullStop {
                0.0
            } else {
                status.accel_target
            };

            let target_accel_abs = target.accel.abs();
            let is_inertial = target_accel_abs < INTERNAL_ACCEL_MS2;
            let is_accel_triggered = !is_inertial && target_accel_abs >= restrictions.min_accel;

            let is_speed_control_enabled = if is_accel_triggered {
                accel_activator.inc()
            } else {
                accel_activator.dec();
                false
            };

            if is_speed_control_enabled {
                speed_pid.setpoint = target.speed.abs();
                let delta = speed_pid.next_control_output(current.speed).output;

                let (min_accel, max_accel) = if is_inertial {
                    let Restrictions {
                        max_decel,
                        max_accel,
                        ..
                    } = *restrictions;
                    (-max_decel, max_accel)
                } else {
                    (-target_accel_abs, target_accel_abs)
                };

                (prev_target + delta).clamp(min_accel, max_accel)
            } else {
                target.accel
            }
        };
        let pedal_target = {
            let prev_target = if speed_kind == SpeedKind::FullStop {
                0.0
            } else {
                status.pedal_target
            };
            accel_pid.setpoint = accel_target;
            let delta = accel_pid.next_control_output(current.accel).output;
            let max_pedal = restrictions.max_pedal;
            (prev_target + delta).clamp(-max_pedal, max_pedal)
        };

        let physics = VehiclePhysics::new(physics_control, current.speed, pitch_radians, reverse);
        let throttle_lower_border = physics.driving_impedance_acceleration;
        let brake_upper_border = throttle_lower_border + physics.lay_off_engine_acceleration;

        let (status_kind, output) = if speed_kind == SpeedKind::FullStop {
            let kind = StatusKind::FullStop;
            let output = Output {
                hand_brake: true,
                steer,
                reverse,
                brake: 1.0,
                throttle: 0.0,
            };
            (kind, output)
        } else if pedal_target > throttle_lower_border {
            let kind = StatusKind::Accelerating;
            let throttle = (pedal_target - throttle_lower_border) / restrictions.max_pedal;
            let output = Output {
                hand_brake: false,
                steer,
                reverse,
                brake: 0.0,
                throttle,
            };
            (kind, output)
        } else if pedal_target > brake_upper_border {
            let kind = StatusKind::Coasting;
            let output = Output {
                hand_brake: false,
                steer,
                reverse,
                brake: 0.0,
                throttle: 0.0,
            };
            (kind, output)
        } else {
            let kind = StatusKind::Braking;
            let brake = (brake_upper_border - pedal_target) / restrictions.max_pedal;
            let output = Output {
                hand_brake: false,
                steer,
                reverse,
                brake,
                throttle: 0.0,
            };
            (kind, output)
        };

        let status = Status {
            status: Some(status_kind),
            accel_target,
            pedal_target,
        };

        self.info = Info {
            target,
            status,
            current,
            ..self.info.clone()
        };

        output
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum SpeedKind {
    FullStop,
    StandStill,
    Driving,
}

impl SpeedKind {
    pub fn from_speed_ms(speed_ms: f64) -> Self {
        let speed_ms = speed_ms.abs();

        if speed_ms < FULL_STOP_SPEED_MS {
            Self::FullStop
        } else if speed_ms < STAND_STILL_SPEED_MS {
            Self::StandStill
        } else {
            Self::Driving
        }
    }
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
