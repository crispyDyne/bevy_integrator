use bevy::{ecs::schedule::ScheduleLabel, prelude::*, utils::HashMap};
mod camera_az_el;
mod enviornment;

use camera_az_el::camera_builder;
use enviornment::build_environment;

// set a larger timestep if the animation lags
const FIXED_TIMESTEP: f32 = 0.002; // 500 fps

// Define physics system sets, which are used to group systems together, and define the order in which they are run
#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
enum PhysicsSet {
    Initialize,
    Physics,
    Finalize,
}

// Define the physics schedule which will be run in the fixed timestep loop
#[derive(ScheduleLabel, Debug, Hash, PartialEq, Eq, Clone)]
struct PhysicsSchedule;

// derive resource
#[derive(Resource, Default)]
struct PhysicsState {
    state: HashMap<Entity, (f32, f32, f32)>,
}

fn integrator_schedule(world: &mut World) {
    // get the initial state
    let state_0 = world.get_resource::<PhysicsState>().unwrap().state.clone();

    // run the physics schedule
    world.run_schedule(PhysicsSchedule);

    // get the state after the physics schedule has run, which include state derivatives
    let state = &mut world.get_resource_mut::<PhysicsState>().unwrap().state;

    for (_entity_0, (position_0, velocity_0, _acceleration_0)) in state_0.iter() {
        let (position, velocity, acceleration) = state.get_mut(_entity_0).unwrap();
        *position = *position_0 + *velocity * FIXED_TIMESTEP;
        *velocity = *velocity_0 + *acceleration * FIXED_TIMESTEP;
    }
}

// Main function
fn main() {
    // create the physics schedule
    let mut phys_schedule = Schedule::new();
    phys_schedule
        .configure_sets(
            (
                PhysicsSet::Initialize,
                PhysicsSet::Physics,
                PhysicsSet::Finalize,
            )
                .chain(), // This defines the ordering of the system sets
        )
        .add_system(distribute_joint_state.in_set(PhysicsSet::Initialize))
        .add_systems(
            (spring_force, damping_force, apply_gravity) // no "chain" here, these systems can be run in any order
                .in_set(PhysicsSet::Physics),
        )
        .add_systems(
            (calculate_acceleration, collect_joint_state)
                .chain() // these systems should be run in order
                .in_set(PhysicsSet::Finalize),
        );

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
        .add_startup_system(setup_system) // setup the car model and environment
        .init_resource::<PhysicsState>() // add the physics state resource
        .insert_resource(FixedTime::new_from_secs(FIXED_TIMESTEP)) // set the fixed timestep
        .add_schedule(PhysicsSchedule, phys_schedule) // add the physics schedule
        .add_system(integrator_schedule.in_schedule(CoreSchedule::FixedUpdate)) // run the physics schedule in the fixed timestep loop
        .add_system(bevy_joint_positions) // update the bevy joint positions
        .run();
}

pub fn setup_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    build_model(&mut commands, &mut meshes, &mut materials);
    build_environment(&mut commands, &mut meshes, &mut materials);
}

// derive the Component trait for the Joint struct
#[derive(Component)]
pub struct Joint {
    position: f32,
    velocity: f32,
    acceleration: f32,
    force: f32,
    mass: f32,
}

impl Joint {
    fn get_state(&self) -> (f32, f32) {
        (self.position, self.velocity)
    }

    fn set_state(&mut self, position: f32, velocity: f32) {
        self.position = position;
        self.velocity = velocity;
    }

    fn get_dstate(&self) -> (f32, f32) {
        (self.velocity, self.acceleration)
    }

    fn set_dstate(&mut self, velocity: f32, acceleration: f32) {
        self.velocity = velocity;
        self.acceleration = acceleration;
    }
}

fn build_model(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    commands
        .spawn(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube { size: 1. })),
            material: materials.add(Color::rgb(0.8, 0.1, 0.2).into()),
            transform: Transform::from_translation(Vec3::new(0., 0., 0.0)),
            ..Default::default()
        })
        .insert(Joint {
            position: 0.5,
            velocity: 0.,
            acceleration: 0.,
            force: 0.,
            mass: 1.,
        });
}

fn spring_force(mut joint_query: Query<&mut Joint>) {
    for mut joint in joint_query.iter_mut() {
        joint.force += -10.0 * (joint.position - 3.);
    }
}

fn damping_force(mut joint_query: Query<&mut Joint>) {
    for mut joint in joint_query.iter_mut() {
        joint.force += -0.1 * joint.velocity;
    }
}

fn apply_gravity(mut joint_query: Query<&mut Joint>) {
    for mut joint in joint_query.iter_mut() {
        joint.force += -9.81 * joint.mass;
    }
}

fn calculate_acceleration(mut joint_query: Query<&mut Joint>) {
    for mut joint in joint_query.iter_mut() {
        joint.acceleration = joint.force / joint.mass;
    }
}

pub fn bevy_joint_positions(mut joint_transform_query: Query<(&mut Joint, &mut Transform)>) {
    for (joint, mut transform) in joint_transform_query.iter_mut() {
        transform.translation = Vec3::new(0., 0., joint.position);
    }
}

fn collect_joint_state(
    mut joint_query: Query<(Entity, &mut Joint)>,
    mut physics_state: ResMut<PhysicsState>,
) {
    for (entity, mut joint) in joint_query.iter_mut() {
        let (position, _velocity) = joint.get_state();
        let (velocity, acceleration) = joint.get_dstate();
        physics_state
            .state
            .insert(entity, (position, velocity, acceleration));
        joint.force = 0.;
    }
}

fn distribute_joint_state(
    mut joint_query: Query<(Entity, &mut Joint)>,
    physics_state: Res<PhysicsState>,
) {
    for (entity, mut joint) in joint_query.iter_mut() {
        if let Some((position, velocity, acceleration)) = physics_state.state.get(&entity) {
            joint.set_state(*position, *velocity);
            joint.set_dstate(*velocity, *acceleration);
        }
    }
}
