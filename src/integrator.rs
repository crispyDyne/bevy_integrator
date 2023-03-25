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

#[derive(Resource, Default, Clone, Debug)]
pub struct PhysicsState {
    pub state: HashMap<Entity, State1dof>,
}
#[derive(Resource, Default, Clone, Debug)]
pub struct PhysicsStateDerivative {
    pub state: HashMap<Entity, State1dof>,
}

impl std::ops::Mul<f32> for &PhysicsStateDerivative {
    type Output = PhysicsStateDerivative;
    fn mul(self, rhs: f32) -> Self::Output {
        let mut state = HashMap::new();
        for (_entity, state_0) in self.state.iter() {
            let state_0 = state_0.clone();
            state.insert(*_entity, &state_0 * rhs);
        }
        PhysicsStateDerivative { state }
    }
}

impl std::ops::Add<&PhysicsStateDerivative> for &PhysicsState {
    type Output = PhysicsState;
    fn add(self, rhs: &PhysicsStateDerivative) -> Self::Output {
        let mut state = HashMap::new();
        for (_entity, state_0) in self.state.iter() {
            let state_0 = state_0.clone();
            let dstate = rhs.state.get(_entity).unwrap();
            state.insert(*_entity, &state_0 + dstate);
        }
        PhysicsState { state }
    }
}
impl std::ops::Add<&PhysicsStateDerivative> for &PhysicsStateDerivative {
    type Output = PhysicsStateDerivative;
    fn add(self, rhs: &PhysicsStateDerivative) -> Self::Output {
        let mut state = HashMap::new();
        for (_entity, state_0) in self.state.iter() {
            let state_0 = state_0.clone();
            let dstate = rhs.state.get(_entity).unwrap();
            state.insert(*_entity, &state_0 + dstate);
        }
        PhysicsStateDerivative { state }
    }
}
impl std::ops::Add<PhysicsStateDerivative> for PhysicsStateDerivative {
    type Output = PhysicsStateDerivative;
    fn add(self, rhs: PhysicsStateDerivative) -> Self::Output {
        let mut state = HashMap::new();
        for (_entity, state_0) in self.state.iter() {
            let state_0 = state_0.clone();
            let dstate = rhs.state.get(_entity).unwrap();
            state.insert(*_entity, &state_0 + dstate);
        }
        PhysicsStateDerivative { state }
    }
}

fn evaluate_state(world: &mut World, state: &mut PhysicsState, _t: f32) -> PhysicsStateDerivative {
    // assign the state
    let mut physics_state = world.get_resource_mut::<PhysicsState>().unwrap();
    physics_state.state = state.state.clone();

    // run the physics
    world.run_schedule(PhysicsSchedule);

    // return the state derivative
    let physics_dstate = world.get_resource::<PhysicsStateDerivative>().unwrap();
    physics_dstate.clone()
}

pub fn integrator_schedule(world: &mut World) {
    // get the initial state
    let state_0 = world.get_resource::<PhysicsState>().unwrap().clone();

    // get step size
    let time_step = world
        .get_resource::<FixedTime>()
        .unwrap()
        .period
        .as_secs_f32();

    // get the time (need to track time in the fixed timestep loop)
    let time = 0.0; // constant for now

    let state = rk4(world, state_0, time, time_step);
    let mut physics_state = world.get_resource_mut::<PhysicsState>().unwrap();
    physics_state.state = state.state;
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
        .add_system(distribute_state.in_set(PhysicsSet::Initialize))
        .add_systems(
            systems // no "chain" here, these systems can be run in any order
                .in_set(PhysicsSet::Physics),
        )
        .add_systems(
            (calculate_acceleration, collect_state_derivatives)
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

fn distribute_state(
    mut joint_query: Query<(Entity, &mut Joint)>,
    physics_state: Res<PhysicsState>,
) {
    for (entity, mut joint) in joint_query.iter_mut() {
        if let Some(state) = physics_state.state.get(&entity) {
            joint.set_state(state);
        }
    }
}

fn collect_state_derivatives(
    mut joint_query: Query<(Entity, &mut Joint)>,
    mut physics_dstate: ResMut<PhysicsStateDerivative>,
) {
    for (entity, mut joint) in joint_query.iter_mut() {
        physics_dstate.state.insert(entity, joint.get_dstate());
        joint.force = 0.;
    }
}

#[derive(Clone, Debug)]
pub struct State1dof {
    pub position: f32,
    pub velocity: f32,
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

impl std::ops::AddAssign<&State1dof> for State1dof {
    fn add_assign(&mut self, other: &State1dof) {
        self.position += other.position;
        self.velocity += other.velocity;
    }
}

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

    fn set_dstate(&mut self, dstate: State1dof) {
        self.velocity = dstate.position;
        self.acceleration = dstate.velocity;
    }
}

fn euler(world: &mut World, state: PhysicsState, t: f32, dt: f32) -> PhysicsState {
    let state_derivative = evaluate_state(world, &mut state.clone(), t);
    &state + &(&state_derivative * dt)
}

fn heun(world: &mut World, state: PhysicsState, t: f32, dt: f32) -> PhysicsState {
    let state_derivative = evaluate_state(world, &mut state.clone(), t);
    let state_derivative2 =
        evaluate_state(world, &mut (&state + &(&state_derivative * dt)), t + dt);
    &state + &(&(&state_derivative + &state_derivative2) * (dt * 0.5))
}

fn midpoint(world: &mut World, state: PhysicsState, t: f32, dt: f32) -> PhysicsState {
    let state_derivative = evaluate_state(world, &mut state.clone(), t);
    let state_derivative2 = evaluate_state(
        world,
        &mut (&state + &(&state_derivative * (dt * 0.5))),
        t + dt * 0.5,
    );
    &state + &(&state_derivative2 * dt)
}

fn rk4(world: &mut World, state: PhysicsState, t: f32, dt: f32) -> PhysicsState {
    let state_derivative = evaluate_state(world, &mut state.clone(), t);
    let state_derivative2 = evaluate_state(
        world,
        &mut (&state + &(&state_derivative * (dt * 0.5))),
        t + dt * 0.5,
    );
    let state_derivative3 = evaluate_state(
        world,
        &mut (&state + &(&state_derivative2 * (dt * 0.5))),
        t + dt * 0.5,
    );
    let state_derivative4 =
        evaluate_state(world, &mut (&state + &(&state_derivative3 * dt)), t + dt);
    let state_change =
        state_derivative + &state_derivative2 * 2. + &state_derivative3 * 2. + state_derivative4;
    &state + &(&state_change * (dt / 6.))
}
