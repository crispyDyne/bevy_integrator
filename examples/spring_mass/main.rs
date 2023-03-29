mod joint;
pub mod model;

use joint::{bevy_joint_positions, calculate_acceleration, Joint};
use model::{apply_gravity, damping_force, spring_force};

use bevy::prelude::*;
use bevy_integrator::{
    camera_az_el::{self, camera_builder},
    integrator::{
        initialize_state, integrator_schedule, PhysicsSchedule, PhysicsScheduleExt, Solver,
    },
};

// set a larger timestep if the animation lags
const FIXED_TIMESTEP: f32 = 0.002; // 0.002s => 500 fps (starts lagging around 0.0002 => 5000 fps)
                                   // RK4 is a 4th order method, and is stable up to 0.5s
                                   // Euler is a 1st order method, and is stable up to ~0.01s

// Main function
fn main() {
    let mut physics_schedule = Schedule::new();
    physics_schedule.add_physics_systems::<Joint, _, _, _>(
        (),
        (spring_force, damping_force, apply_gravity),
        (calculate_acceleration,),
    );

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                resolution: (1200., 900.).into(),
                title: "integrator".to_string(),
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
        .add_system(camera_az_el::az_el_camera)
        .add_startup_system(model::setup) // setup the model and environment
        .add_startup_system(initialize_state::<Joint>.in_base_set(StartupSet::PostStartup)) // setup the car model and environment
        .insert_resource(FixedTime::new_from_secs(FIXED_TIMESTEP)) // set the fixed timestep
        .add_schedule(PhysicsSchedule, physics_schedule) // add the physics schedule
        .insert_resource(Solver::RK4) // set the solver to use
        .add_system(integrator_schedule::<Joint>.in_schedule(CoreSchedule::FixedUpdate)) // run the physics schedule in the fixed timestep loop
        .add_system(bevy_joint_positions) // update the bevy joint positions
        .run();
}
