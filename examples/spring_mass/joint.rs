use bevy::prelude::*;
use std::ops::{Add, Mul};

use bevy_integrator::integrator::Stateful;

// this is an example of a stateful component that can be integrated in the physics engine
#[derive(Component, Debug)]
pub struct Joint {
    pub position: f32,
    pub velocity: f32,
    pub acceleration: f32,
    pub force: f32,
    pub mass: f32,
}

impl Stateful for Joint {
    type State = JointState;

    fn get_state(&self) -> Self::State {
        JointState {
            position: self.position,
            velocity: self.velocity,
        }
    }

    fn set_state(&mut self, state: &Self::State) {
        self.position = state.position;
        self.velocity = state.velocity;
    }

    fn get_dstate(&self) -> Self::State {
        JointState {
            position: self.velocity,
            velocity: self.acceleration,
        }
    }

    fn set_dstate(&mut self, dstate: Self::State) {
        self.velocity = dstate.position;
        self.acceleration = dstate.velocity;
    }

    fn reset(&mut self) {
        self.acceleration = 0.;
        self.force = 0.;
    }
}

#[derive(Clone, Debug)]
pub struct JointState {
    pub position: f32,
    pub velocity: f32,
}

impl Add for JointState {
    type Output = JointState;

    fn add(self, other: JointState) -> JointState {
        JointState {
            position: self.position + other.position,
            velocity: self.velocity + other.velocity,
        }
    }
}

impl Mul<f32> for JointState {
    type Output = JointState;

    fn mul(self, other: f32) -> JointState {
        JointState {
            position: self.position * other,
            velocity: self.velocity * other,
        }
    }
}

pub fn calculate_acceleration(mut joint_query: Query<&mut Joint>) {
    for mut joint in joint_query.iter_mut() {
        joint.acceleration = joint.force / joint.mass;
    }
}

pub fn bevy_joint_positions(mut joint_transform_query: Query<(&mut Joint, &mut Transform)>) {
    for (joint, mut transform) in joint_transform_query.iter_mut() {
        transform.translation = Vec3::new(0., 0., joint.position);
    }
}
