use bevy::prelude::Component;

#[derive(Component)]
pub struct Joint {
    pub position: f32,
    pub velocity: f32,
    pub acceleration: f32,
    pub force: f32,
    pub mass: f32,
}

impl Joint {
    pub fn get_state(&self) -> JointState {
        JointState {
            position: self.position,
            velocity: self.velocity,
        }
    }

    pub fn set_state(&mut self, state: &JointState) {
        self.position = state.position;
        self.velocity = state.velocity;
    }

    pub fn get_dstate(&self) -> JointState {
        JointState {
            position: self.velocity,
            velocity: self.acceleration,
        }
    }

    pub fn set_dstate(&mut self, dstate: JointState) {
        self.velocity = dstate.position;
        self.acceleration = dstate.velocity;
    }
}

#[derive(Clone, Debug)]
pub struct JointState {
    pub position: f32,
    pub velocity: f32,
}

impl std::ops::Add for &JointState {
    type Output = JointState;

    fn add(self, other: &JointState) -> JointState {
        JointState {
            position: self.position + other.position,
            velocity: self.velocity + other.velocity,
        }
    }
}

impl std::ops::Mul<f32> for &JointState {
    type Output = JointState;

    fn mul(self, other: f32) -> JointState {
        JointState {
            position: self.position * other,
            velocity: self.velocity * other,
        }
    }
}

impl std::ops::AddAssign<&JointState> for JointState {
    fn add_assign(&mut self, other: &JointState) {
        self.position += other.position;
        self.velocity += other.velocity;
    }
}
