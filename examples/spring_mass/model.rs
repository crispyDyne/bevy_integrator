use bevy::prelude::*;

use crate::joint::{Joint, JointState};
use bevy_integrator::{
    environment::build_environment,
    integrator::{PhysicsState, StateMap},
};

pub fn spring_force(mut joint_query: Query<&mut Joint>) {
    for mut joint in joint_query.iter_mut() {
        joint.force += -10.0 * (joint.position - 3.);
    }
}

pub fn damping_force(mut joint_query: Query<&mut Joint>) {
    for mut joint in joint_query.iter_mut() {
        joint.force += -0.1 * joint.velocity;
    }
}

pub fn apply_gravity(mut joint_query: Query<&mut Joint>) {
    for mut joint in joint_query.iter_mut() {
        joint.force += -9.81 * joint.mass;
    }
}

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    build_model(&mut commands, &mut meshes, &mut materials);
    build_environment(&mut commands, &mut meshes, &mut materials);
}

pub fn build_model(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    let mut entity = commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Cube { size: 1. })),
        material: materials.add(Color::rgb(0.8, 0.1, 0.2).into()),
        transform: Transform::from_translation(Vec3::new(0., 0., 0.0)),
        ..Default::default()
    });
    entity.insert(Joint {
        position: 0.5,
        velocity: 0.,
        acceleration: 0.,
        force: 0.,
        mass: 1.,
    });

    let entity_id = entity.id();

    // set initial state
    let mut states: StateMap<Joint> = StateMap::new();
    states.insert(
        entity_id,
        JointState {
            position: 0.5, // position the cube at 0.5m (so it is initially sitting on the ground)
            velocity: 0.0,
        },
    );
    // set initial derivatives of state
    let mut dstates: StateMap<Joint> = StateMap::new();
    dstates.insert(
        entity_id,
        JointState {
            position: 0.0,
            velocity: 0.0,
        },
    );
    commands.insert_resource(PhysicsState::<Joint> { states, dstates });
}
