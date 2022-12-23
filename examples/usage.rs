use anyhow::Result;
use carla::{
    client::{ActorBase, Client, Vehicle},
    rpc::VehicleControl,
};
use carla_ackermann::{TargetRequest, VehicleController};
use clap::Parser;
use rand::prelude::*;

#[derive(Parser)]
struct Opts {
    #[clap(default_value = "127.0.0.1")]
    pub address: String,
    #[clap(default_value = "2000")]
    pub port: u16,
}

fn main() -> Result<()> {
    let Opts { address, port } = Opts::parse();

    // Connect to Carla server
    let client = Client::connect(&address, port, None);
    let mut world = client.world();

    // Spawn a car
    let mut vehicle: Vehicle = {
        let mut rng = rand::thread_rng();

        let spawn_point = {
            let spawn_points = world.map().recommended_spawn_points();
            let index = rng.gen_range(0..spawn_points.len());
            spawn_points.get(index).unwrap()
        };

        world
            .actor_builder("vehicle.tesla.model3")?
            .spawn_vehicle(&spawn_point)?
    };

    // Create a vehicle controller
    let mut controller = VehicleController::from_physics_control(&vehicle.physics_control(), None);
    controller.set_target(TargetRequest {
        steering_angle: 0.0,
        speed: 5.0,
        accel: 1.0,
    });

    // Get initial world ID and elapsed simulation time.
    let snapshot = world.wait_for_tick();
    let mut world_id = snapshot.id();
    let mut time_secs = snapshot.timestamp().elapsed_seconds;

    loop {
        // Compute time delta since last step.
        // If the world is reloaded, the time delta is set to zero.
        let curr_id = snapshot.id();
        let curr_secs = snapshot.timestamp().elapsed_seconds;

        let time_delta_secs = if curr_id != world_id {
            0.0
        } else {
            curr_secs - time_secs
        };
        world_id = curr_id;
        time_secs = curr_secs;

        // Generate a control command from the controller
        let speed = vehicle.velocity().norm();
        let (_, pitch, _) = vehicle.transform().rotation.euler_angles();
        let (output, _report) = controller.step(time_delta_secs, speed as f64, pitch as f64);

        // Apply control to the car
        vehicle.apply_control(&VehicleControl {
            throttle: output.throttle as f32,
            steer: output.steer as f32,
            brake: output.brake as f32,
            hand_brake: output.hand_brake,
            reverse: output.reverse,
            manual_gear_shift: false,
            gear: 0,
        });
    }
}
