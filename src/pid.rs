use pid::Pid;

#[derive(Debug, Clone)]
pub struct PidInit {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub output_limit: f64,
}

impl PidInit {
    pub fn build(&self) -> Pid<f64> {
        let Self {
            kp,
            ki,
            kd,
            output_limit,
        } = *self;
        Pid::new(kp, ki, kd, f64::MAX, f64::MAX, f64::MAX, output_limit, 0.0)
    }
}
