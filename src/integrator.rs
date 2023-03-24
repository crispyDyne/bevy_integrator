use bevy::{ecs::schedule::ScheduleLabel, prelude::*, utils::HashMap};

// Define the physics schedule which will be run in the fixed timestep loop
#[derive(ScheduleLabel, Debug, Hash, PartialEq, Eq, Clone)]
pub struct PhysicsSchedule;

// Define physics system sets, which are used to group systems together, and define the order in which they are run
#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
enum PhysicsSet {
    Initialize,
    Physics,
    Finalize,
}

#[derive(Resource, Default)]
pub struct PhysicsState {
    state: HashMap<Entity, State1dof>,
    dstate: HashMap<Entity, State1dof>,
}

pub fn integrator_schedule(world: &mut World) {
    // get the initial state
    let state_0 = world.get_resource::<PhysicsState>().unwrap().state.clone();

    // get step size
    let time_step = world
        .get_resource::<FixedTime>()
        .unwrap()
        .period
        .as_secs_f32();

    // run the physics schedule
    world.run_schedule(PhysicsSchedule);

    // get the state and state derivatives after the physics schedule has run
    let physics_state = &mut world.get_resource_mut::<PhysicsState>().unwrap();
    let dstate = &physics_state.dstate.clone();
    let state = &mut physics_state.state;

    // integrate the state (Euler's method)
    for (_entity_0, state_0) in state_0.iter() {
        let state = state.get_mut(_entity_0).unwrap();
        let dstate = dstate.get(_entity_0).unwrap();
        *state = state_0 + &(dstate * time_step);
    }
}

pub fn create_physics_schedule<M>(systems: impl IntoSystemConfigs<M>) -> Schedule {
    let mut physics_schedule = Schedule::new();
    physics_schedule
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
            systems // no "chain" here, these systems can be run in any order
                .in_set(PhysicsSet::Physics),
        )
        .add_systems(
            (calculate_acceleration, collect_joint_state)
                .chain() // these systems should be run in order
                .in_set(PhysicsSet::Finalize),
        );
    physics_schedule
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

fn distribute_joint_state(
    mut joint_query: Query<(Entity, &mut Joint)>,
    physics_state: Res<PhysicsState>,
) {
    for (entity, mut joint) in joint_query.iter_mut() {
        if let Some(state) = physics_state.state.get(&entity) {
            joint.set_state(state);
        }
    }
}

fn collect_joint_state(
    mut joint_query: Query<(Entity, &mut Joint)>,
    mut physics_state: ResMut<PhysicsState>,
) {
    for (entity, mut joint) in joint_query.iter_mut() {
        physics_state.state.insert(entity, joint.get_state());
        physics_state.dstate.insert(entity, joint.get_dstate());
        joint.force = 0.;
    }
}

// simple 1 degree of freedom state struct
#[derive(Clone)]
struct State1dof {
    position: f32,
    velocity: f32,
}

impl std::ops::Add for &State1dof {
    type Output = State1dof;

    fn add(self, other: &State1dof) -> State1dof {
        State1dof {
            position: self.position + other.position,
            velocity: self.velocity + other.velocity,
        }
    }
}

impl std::ops::Mul<f32> for &State1dof {
    type Output = State1dof;

    fn mul(self, other: f32) -> State1dof {
        State1dof {
            position: self.position * other,
            velocity: self.velocity * other,
        }
    }
}

// derive the Component trait for the Joint struct
#[derive(Component)]
pub struct Joint {
    pub position: f32,
    pub velocity: f32,
    pub acceleration: f32,
    pub force: f32,
    pub mass: f32,
}

impl Joint {
    fn get_state(&self) -> State1dof {
        State1dof {
            position: self.position,
            velocity: self.velocity,
        }
    }

    fn set_state(&mut self, state: &State1dof) {
        self.position = state.position;
        self.velocity = state.velocity;
    }

    fn get_dstate(&self) -> State1dof {
        State1dof {
            position: self.velocity,
            velocity: self.acceleration,
        }
    }

    // fn set_dstate(&mut self, dstate: State1dof) {
    //     self.velocity = dstate.position;
    //     self.acceleration = dstate.velocity;
    // }
}
