use crate::{
    accel_control::{AccelControl, AccelController, AccelControllerInit},
    constants::FULL_STOP_SPEED_MS,
    physics::VehiclePhysics,
    speed_control::{SpeedControl, SpeedController, SpeedControllerInit},
    steer_control::SteerController,
};
use carla::rpc::VehiclePhysicsControl;

/// Initializer of [VehicleController].
#[derive(Debug, Clone)]
pub struct VehicleControllerInit {
    pub physics: VehiclePhysics,
    pub speed_controller: SpeedControllerInit,
    pub accel_controller: AccelControllerInit,
    pub max_steering_angle: f64,
}

impl VehicleControllerInit {
    pub fn from_physics_control(
        physics_control: &VehiclePhysicsControl,
        min_accel: Option<f64>,
    ) -> Self {
        Self::from_physics(VehiclePhysics::new(physics_control), min_accel)
    }

    pub fn from_physics(physics: VehiclePhysics, min_accel: Option<f64>) -> Self {
        Self {
            speed_controller: SpeedControllerInit::from_physics(&physics, min_accel),
            accel_controller: AccelControllerInit::from_physics(&physics),
            max_steering_angle: physics.max_steering_angle(),
            physics,
        }
    }

    pub fn build(self) -> VehicleController {
        let Self {
            physics,
            speed_controller,
            accel_controller,
            max_steering_angle,
        } = self;

        VehicleController {
            measurement: Measurement::default(),
            physics,
            speed_controller: speed_controller.build(),
            accel_controller: accel_controller.build(),
            steer_controller: SteerController::new(max_steering_angle),
        }
    }
}

/// A controller that controls the speed and steering of a vehicle.
#[derive(Debug)]
pub struct VehicleController {
    measurement: Measurement,
    physics: VehiclePhysics,
    speed_controller: SpeedController,
    accel_controller: AccelController,
    steer_controller: SteerController,
}

/// Desired target values passed to [VehicleController].
#[derive(Debug, Clone)]
pub struct TargetRequest {
    pub steering_angle: f64,
    pub speed: f64,
    pub accel: f64,
}

/// The report created by [VehicleController::step].
#[derive(Debug, Clone)]
pub struct Report {
    pub status: Status,
    pub setpoint_accel: f64,
    pub target_pedal: f64,
    pub delta_accel: f64,
    pub pedal_delta: f64,
}

/// Output of [VehicleController::step].
#[derive(Debug, Clone)]
pub struct Output {
    pub throttle: f64,
    pub brake: f64,
    pub steer: f64,
    pub reverse: bool,
    pub hand_brake: bool,
}

#[derive(Debug, Clone)]
struct Measurement {
    pub time_sec: f64,
    pub speed: f64,
    pub accel: f64,
}

/// The status reported by [VehicleController].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Status {
    FullStop,
    Accelerating,
    Coasting,
    Braking,
}

impl Measurement {
    pub fn update(&mut self, time_delta_sec: f64, current_speed: f64) {
        let speed_delta = current_speed - self.speed;
        let current_accel = speed_delta / time_delta_sec;
        let time_sec = self.time_sec + time_delta_sec;
        let is_full_stop = current_speed < FULL_STOP_SPEED_MS;

        *self = if is_full_stop {
            Measurement {
                time_sec,
                speed: 0.0,
                accel: 0.0,
            }
        } else {
            Measurement {
                time_sec,
                speed: current_speed,
                accel: current_accel,
            }
        };
    }
}

impl Default for Measurement {
    fn default() -> Self {
        Self {
            time_sec: 0.0,
            speed: 0.0,
            accel: 0.0,
        }
    }
}

impl VehicleController {
    /// Creates a controller from an [VehiclePhysicsControl] object.
    ///
    /// The `physics_control` can be created by
    /// [vehicle.physics_control()](Vehicle::physics_control).
    pub fn from_physics_control(
        physics_control: &VehiclePhysicsControl,
        min_accel: Option<f64>,
    ) -> Self {
        Self::from_physics(VehiclePhysics::new(physics_control), min_accel)
    }

    /// Creates a controller from an [VehiclePhysics] object.
    pub fn from_physics(physics: VehiclePhysics, min_accel: Option<f64>) -> Self {
        VehicleControllerInit {
            speed_controller: SpeedControllerInit::from_physics(&physics, min_accel),
            accel_controller: AccelControllerInit::from_physics(&physics),
            max_steering_angle: physics.max_steering_angle(),
            physics,
        }
        .build()
    }

    /// Set target values for the controller.
    pub fn set_target(&mut self, target: TargetRequest) {
        let TargetRequest {
            steering_angle,
            speed,
            accel,
        } = target;
        self.steer_controller.set_target(steering_angle);
        self.speed_controller.set_target(speed, accel);
    }

    /// Produces a controlling command.
    ///
    /// # Parameters
    /// - `time_delta_sec` is elapsed seconds since last step.
    /// - `current_speed` is the current speed of the car.
    /// - `pitch_radians` is the current pitch angle of the car.
    pub fn step(
        &mut self,
        time_delta_sec: f64,
        current_speed: f64,
        pitch_radians: f64,
    ) -> (Output, Report) {
        assert!(time_delta_sec > 0.0);

        let Self {
            measurement,
            physics,
            speed_controller,
            accel_controller,
            steer_controller,
        } = self;

        // Save measurements
        measurement.update(time_delta_sec, current_speed);

        // Compute steer ratio
        let steer = steer_controller.steer_ratio();

        // Run speed controller
        let SpeedControl {
            setpoint_accel,
            delta_accel,
            full_stop,
        } = speed_controller.step(current_speed);

        // Run acceleration controller
        accel_controller.set_target_accel(setpoint_accel);
        if full_stop {
            accel_controller.reset_target_pedal();
        }
        let AccelControl {
            target_pedal,
            pedal_delta,
        } = accel_controller.step(measurement.accel);

        let reverse = speed_controller.target_speed() < 0.0;
        let throttle_lower_border =
            physics.driving_impedance_acceleration(measurement.speed, pitch_radians, reverse);
        let brake_upper_border = throttle_lower_border + physics.lay_off_engine_acceleration();

        let (status_kind, output) = if full_stop {
            let kind = Status::FullStop;
            let output = Output {
                hand_brake: true,
                steer,
                reverse,
                brake: 1.0,
                throttle: 0.0,
            };
            (kind, output)
        } else if target_pedal > throttle_lower_border {
            let kind = Status::Accelerating;
            let throttle = (target_pedal - throttle_lower_border) / accel_controller.max_pedal();
            let output = Output {
                hand_brake: false,
                steer,
                reverse,
                brake: 0.0,
                throttle,
            };
            (kind, output)
        } else if target_pedal > brake_upper_border {
            let kind = Status::Coasting;
            let output = Output {
                hand_brake: false,
                steer,
                reverse,
                brake: 0.0,
                throttle: 0.0,
            };
            (kind, output)
        } else {
            let kind = Status::Braking;
            let brake = (brake_upper_border - target_pedal) / accel_controller.max_pedal();
            let output = Output {
                hand_brake: false,
                steer,
                reverse,
                brake,
                throttle: 0.0,
            };
            (kind, output)
        };

        let report = Report {
            status: status_kind,
            setpoint_accel,
            target_pedal,
            delta_accel,
            pedal_delta,
        };

        (output, report)
    }
}
