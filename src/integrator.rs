use bevy::{ecs::schedule::ScheduleLabel, prelude::*};
use std::{
    collections::HashMap,
    ops::{Add, Mul},
};

// Define the physics schedule which will be run in the fixed timestep loop
#[derive(ScheduleLabel, Debug, Hash, PartialEq, Eq, Clone)]
pub struct PhysicsSchedule;

// Define physics system sets, which are used to group systems together, and define the order in which they are run
#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
enum PhysicsSet {
    Initialize,
    Evaluate,
    Finalize,
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
enum SolverSet {
    Pre,
    Post,
}

pub struct StateMap<T: Stateful>(pub HashMap<Entity, T::State>);

// wrapper over HashMap<Entity, T::State> to implement Add and Mul
impl<T: Stateful> StateMap<T> {
    pub fn new() -> Self {
        StateMap(HashMap::new())
    }
    pub fn get(&self, entity: &Entity) -> Option<&T::State> {
        self.0.get(entity)
    }

    pub fn insert(&mut self, entity: Entity, state: T::State) {
        self.0.insert(entity, state);
    }
}

impl<T: Stateful> Clone for StateMap<T> {
    fn clone(&self) -> Self {
        StateMap(self.0.clone())
    }
}

impl<T: Stateful> Mul<f32> for &StateMap<T> {
    type Output = StateMap<T>;

    fn mul(self, rhs: f32) -> Self::Output {
        let mut result = HashMap::new();
        for (entity, state) in self.0.iter() {
            result.insert(*entity, state.clone() * rhs);
        }
        StateMap(result)
    }
}

impl<T: Stateful> Add for &StateMap<T> {
    type Output = StateMap<T>;

    fn add(self, rhs: Self) -> Self::Output {
        let mut result = HashMap::new();
        for (entity, state) in self.0.iter() {
            result.insert(*entity, state.clone() + rhs.0.get(entity).unwrap().clone());
        }
        StateMap(result)
    }
}

fn evaluate_state<T: Stateful>(world: &mut World, state: &StateMap<T>, _t: f32) -> StateMap<T> {
    // assign the state
    world.resource_scope(
        |_world: &mut World, mut physics_state: Mut<PhysicsState<T>>| {
            physics_state.states = state.clone();
        },
    );

    // run the physics
    world.run_schedule(PhysicsSchedule);

    // return the state derivative
    let mut dstates = StateMap(HashMap::new());
    world.resource_scope(|_world: &mut World, physics_state: Mut<PhysicsState<T>>| {
        dstates = physics_state.dstates.clone();
    });
    dstates
}

pub fn integrator_schedule<T: Stateful>(world: &mut World) {
    // get the initial state
    let state_0 = world
        .get_resource::<PhysicsState<T>>()
        .unwrap()
        .states
        .clone();

    // get step size
    let time_step = world
        .get_resource::<FixedTime>()
        .unwrap()
        .period
        .as_secs_f32();

    // get the time (need to track time in the fixed timestep loop)
    let time = 0.0; // constant for now

    // get Solver resource from world
    let solver = world.get_resource::<Solver>().unwrap();

    let state = match solver {
        Solver::Euler => euler::<T>(world, &state_0, time, time_step),
        Solver::Heun => heun::<T>(world, &state_0, time, time_step),
        Solver::Midpoint => midpoint::<T>(world, &state_0, time, time_step),
        Solver::RK4 => rk4::<T>(world, &state_0, time, time_step),
    };

    let mut physics_state = world.get_resource_mut::<PhysicsState<T>>().unwrap();
    physics_state.states = state;
}

pub trait Stateful: std::fmt::Debug + 'static {
    type State: Add<Output = Self::State> + Mul<f32, Output = Self::State> + Clone + Sync + Send;
    fn get_state(&self) -> Self::State;
    fn set_state(&mut self, state: &Self::State);
    fn get_dstate(&self) -> Self::State;
    fn set_dstate(&mut self, dstate: Self::State);
    fn reset(&mut self);
}

#[derive(Resource)]
pub struct PhysicsState<T: Stateful> {
    pub states: StateMap<T>,
    pub dstates: StateMap<T>,
}

pub trait PhysicsScheduleExt {
    fn add_physics_systems<T, MInit, M, MFinal>(
        &mut self,
        systems_init: impl IntoSystemConfigs<MInit>,
        systems: impl IntoSystemConfigs<M>,
        systems_final: impl IntoSystemConfigs<MFinal>,
    ) -> &mut Self
    where
        T: Component + Stateful;
}

impl PhysicsScheduleExt for Schedule {
    fn add_physics_systems<T, MInit, M, MFinal>(
        &mut self,
        systems_init: impl IntoSystemConfigs<MInit>,
        systems: impl IntoSystemConfigs<M>,
        systems_final: impl IntoSystemConfigs<MFinal>,
    ) -> &mut Self
    where
        T: Component + Stateful,
    {
        self.configure_sets(
            (
                SolverSet::Pre,
                PhysicsSet::Initialize,
                PhysicsSet::Evaluate,
                PhysicsSet::Finalize,
                SolverSet::Post,
            )
                .chain(), // This defines the ordering of the system sets
        )
        .add_system(distribute_state::<T>.in_set(SolverSet::Pre))
        .add_systems(systems_init.in_set(PhysicsSet::Initialize))
        .add_systems(systems.in_set(PhysicsSet::Evaluate))
        .add_systems(systems_final.in_set(PhysicsSet::Finalize))
        .add_system(collect_state_derivatives::<T>.in_set(SolverSet::Post));

        self
    }
}

pub fn initialize_state<T: Component + Stateful>(
    mut commands: Commands,
    joint_query: Query<(Entity, &T)>,
) {
    let mut states = StateMap::<T>::new();
    let mut dstates = StateMap::<T>::new();
    for (entity, joint) in joint_query.iter() {
        states.insert(entity, joint.get_state());
        dstates.insert(entity, joint.get_dstate());
    }
    commands.insert_resource(PhysicsState::<T> { states, dstates });
}

fn distribute_state<T: Component + Stateful>(
    mut joint_query: Query<(Entity, &mut T)>,
    physics_state: Res<PhysicsState<T>>,
) {
    for (entity, mut joint) in joint_query.iter_mut() {
        if let Some(state) = physics_state.states.get(&entity) {
            joint.set_state(state);
            joint.reset();
        }
    }
}

fn collect_state_derivatives<T: Component + Stateful>(
    mut joint_query: Query<(Entity, &mut T)>,
    mut physics_state: ResMut<PhysicsState<T>>,
) {
    for (entity, joint) in joint_query.iter_mut() {
        let joint_state = joint.get_dstate();
        physics_state.dstates.insert(entity, joint_state);
    }
}

#[derive(Resource)]
pub enum Solver {
    Euler,
    Heun,
    Midpoint,
    RK4,
}

fn euler<T: Stateful>(world: &mut World, state: &StateMap<T>, t: f32, dt: f32) -> StateMap<T> {
    let state_derivative = evaluate_state(world, &mut state.clone(), t);
    let updated_state = state + &(&state_derivative * dt);
    updated_state
}

fn heun<T: Stateful>(world: &mut World, state: &StateMap<T>, t: f32, dt: f32) -> StateMap<T> {
    let state_derivative = evaluate_state(world, &mut state.clone(), t);
    let state_derivative2 = evaluate_state(world, &mut (state + &(&state_derivative * dt)), t + dt);
    state + &(&(&state_derivative + &state_derivative2) * (dt * 0.5))
}

fn midpoint<T: Stateful>(world: &mut World, state: &StateMap<T>, t: f32, dt: f32) -> StateMap<T> {
    let state_derivative = evaluate_state(world, &mut state.clone(), t);
    let state_derivative2 = evaluate_state(
        world,
        &mut (state + &(&state_derivative * (dt * 0.5))),
        t + dt * 0.5,
    );
    state + &(&state_derivative2 * dt)
}

fn rk4<T: Stateful>(world: &mut World, state: &StateMap<T>, t: f32, dt: f32) -> StateMap<T> {
    let state_derivative = evaluate_state(world, &mut state.clone(), t);
    let state_derivative2 = evaluate_state(
        world,
        &mut (state + &(&state_derivative * (dt * 0.5))),
        t + dt * 0.5,
    );
    let state_derivative3 = evaluate_state(
        world,
        &mut (state + &(&state_derivative2 * (dt * 0.5))),
        t + dt * 0.5,
    );
    let state_derivative4 =
        evaluate_state(world, &mut (state + &(&state_derivative3 * dt)), t + dt);
    let state_change = &(&(&state_derivative + &(&state_derivative2 * 2.))
        + &(&state_derivative3 * 2.))
        + &state_derivative4;
    state + &(&state_change * (dt / 6.))
}
