use crate::physics::VehiclePhysics;

#[derive(Debug)]
pub struct SteerController {
    pub target_steering_angle: f64,
    pub max_steering_angle: f64,
}

impl SteerController {
    pub fn from_physics(physics: &VehiclePhysics) -> Self {
        Self::new(physics.max_steering_angle())
    }

    pub fn new(max_steering_angle: f64) -> Self {
        Self {
            max_steering_angle,
            target_steering_angle: 0.0,
        }
    }

    pub fn set_target(&mut self, target_steering_angle: f64) {
        let max = self.max_steering_angle;
        self.target_steering_angle = target_steering_angle.clamp(-max, max);
    }

    pub fn steer_ratio(&self) -> f64 {
        self.target_steering_angle / self.max_steering_angle
    }
}
