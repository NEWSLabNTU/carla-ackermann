use crate::constants::DEFAULT_MAX_STEERING_DEGREES;
use carla::rpc::VehiclePhysicsControl;
use noisy_float::types::r64;

const ACCELERATION_OF_GRAVITY: f64 = 9.81;

#[derive(Debug, Clone, PartialEq)]
pub struct VehiclePhysics {
    engine_brake_force: f64,
    mass: f64,
    lay_off_engine_acceleration: f64,
    weight_force: f64,
    rolling_resistance_force: f64,
    max_steering_angle: f64,
    max_speed: f64,
    max_acceleration: f64,
    max_deceleration: f64,
}

impl VehiclePhysics {
    pub fn new(physics_control: &VehiclePhysicsControl) -> Self {
        let VehiclePhysicsControl {
            mass, ref wheels, ..
        } = *physics_control;
        let mass = mass as f64;
        let rolling_resistance_coefficient = 0.01;
        let engine_brake_force = 500.0;
        let lay_off_engine_acceleration = -engine_brake_force / mass;
        let weight_force = mass * ACCELERATION_OF_GRAVITY;
        let rolling_resistance_force = rolling_resistance_coefficient * weight_force;
        let max_steering_angle = wheels
            .iter()
            .map(|wheel| r64(wheel.max_steer_angle as f64))
            .max()
            .map(|val| val.raw())
            .unwrap_or_else(|| DEFAULT_MAX_STEERING_DEGREES.to_radians());
        let max_speed = 180.0 / 3.6;
        let max_accel = 3.0;
        let max_deceleration = 8.0;

        Self {
            mass,
            engine_brake_force,
            lay_off_engine_acceleration,
            weight_force,
            rolling_resistance_force,
            max_steering_angle,
            max_speed,
            max_acceleration: max_accel,
            max_deceleration,
        }
    }

    pub fn driving_impedance_acceleration(
        &self,
        speed: f64,
        pitch_radians: f64,
        reverse: bool,
    ) -> f64 {
        let Self {
            mass,
            rolling_resistance_force,
            ..
        } = *self;
        let speed_squared = speed.powi(2);
        let slope_force_value = -ACCELERATION_OF_GRAVITY * mass * pitch_radians.sin();
        let slope_force = if reverse {
            -slope_force_value
        } else {
            slope_force_value
        };
        let aerodynamic_drag_force = {
            let default_aerodynamic_drag_coefficient = 0.3;
            let default_drag_reference_area = 2.37;
            let drag_area = default_aerodynamic_drag_coefficient * default_drag_reference_area;
            let rho_air_25 = 1.184;
            0.5 * drag_area * rho_air_25 * speed_squared
        };

        -(rolling_resistance_force + aerodynamic_drag_force + slope_force) / mass
    }

    pub fn engine_brake_force(&self) -> f64 {
        self.engine_brake_force
    }

    pub fn mass(&self) -> f64 {
        self.mass
    }

    pub fn lay_off_engine_acceleration(&self) -> f64 {
        self.lay_off_engine_acceleration
    }

    pub fn weight_force(&self) -> f64 {
        self.weight_force
    }

    pub fn rolling_resistance_force(&self) -> f64 {
        self.rolling_resistance_force
    }

    pub fn max_steering_angle(&self) -> f64 {
        self.max_steering_angle
    }

    pub fn max_speed(&self) -> f64 {
        self.max_speed
    }

    pub fn max_accel(&self) -> f64 {
        self.max_acceleration
    }

    pub fn max_deceleration(&self) -> f64 {
        self.max_deceleration
    }
}
