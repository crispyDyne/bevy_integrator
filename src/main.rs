use bevy::prelude::*;
mod camera_az_el;
mod enviornment;
pub mod integrator;
mod model;

use camera_az_el::camera_builder;
use integrator::{
    bevy_joint_positions, create_physics_schedule, integrator_schedule, PhysicsSchedule,
};
use model::{apply_gravity, damping_force, setup, spring_force};

// set a larger timestep if the animation lags
const FIXED_TIMESTEP: f32 = 0.002; // 0.002s => 500 fps (starts lagging around 0.0002 => 5000 fps)
                                   // rk4 is a 4th order method, and is stable up to 0.5s
                                   // euler is a 1st order method, and is stable up to ~0.01s

// Main function
fn main() {
    // create the physics schedule
    let physics_schedule = create_physics_schedule((spring_force, damping_force, apply_gravity));

    // Create App
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                resolution: (1200., 900.).into(),
                title: "Linear".to_string(),
                resizable: true,
                ..default()
            }),
            ..default()
        }))
        .add_startup_system(camera_builder(
            Vec3 {
                x: 0.,
                y: 0.,
                z: 2.,
            },
            -10.0_f32.to_radians(),
            10.0_f32.to_radians(),
            10.,
            camera_az_el::UpDirection::Z,
        ))
        .add_system(camera_az_el::az_el_camera) // setup the camera
        .add_startup_system(setup) // setup the car model and environment
        .insert_resource(FixedTime::new_from_secs(FIXED_TIMESTEP)) // set the fixed timestep
        .add_schedule(PhysicsSchedule, physics_schedule) // add the physics schedule
        .add_system(integrator_schedule.in_schedule(CoreSchedule::FixedUpdate)) // run the physics schedule in the fixed timestep loop
        .add_system(bevy_joint_positions) // update the bevy joint positions
        .run();
}
