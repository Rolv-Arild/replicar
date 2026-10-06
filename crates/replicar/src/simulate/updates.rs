//! Applying a body update to a RocketSim state, and reading a car's controls from the network values.

use glam::{Mat3A, Quat, Vec3A};
use replicar_format::FrameIndex;
use rocketsim::{Arena, CarControls, CarState, PhysState};

use crate::decode::{NetworkBody, NetworkCar, NetworkFrame, NetworkValue};

/// A value is applied when it changed at this frame, or for a body that is new to the simulation (every
/// value it has, however old).
fn applies<T>(value: &NetworkValue<T>, frame: FrameIndex, new_body: bool) -> bool {
    new_body || value.frame == frame
}

/// Applies a body's values that changed at `frame` (all of them for a new body). Returns whether anything
/// was applied.
pub(super) fn apply_update(
    state: &mut PhysState,
    body: &NetworkBody,
    frame: FrameIndex,
    new_body: bool,
) -> bool {
    let mut applied = false;
    if let Some(value) = &body.position
        && applies(value, frame, new_body)
    {
        state.pos = Vec3A::from(value.value);
        applied = true;
    }
    if let Some(value) = &body.rotation
        && applies(value, frame, new_body)
    {
        let [x, y, z, w] = value.value;
        let quat = Quat::from_xyzw(x, y, z, w);
        if quat.is_finite() && quat.length_squared() > 0.5 {
            state.rot_mat = Mat3A::from_quat(quat.normalize());
            applied = true;
        }
    }
    if let Some(value) = &body.linear_velocity
        && applies(value, frame, new_body)
    {
        state.vel = Vec3A::from(value.value);
        applied = true;
    }
    if let Some(value) = &body.angular_velocity_raw
        && applies(value, frame, new_body)
    {
        state.ang_vel = Vec3A::from(value.value) * 0.01;
        applied = true;
    }
    applied
}

/// An update with `sleeping` set omits the velocities: the body is at rest, and a stale velocity of an
/// earlier update would carry it away. The simulated velocities are zeroed (inferred), unless the same
/// update has a velocity. `None` when the body has no sleeping update at `frame`, else whether a velocity
/// changed.
pub(super) fn zero_sleeping_velocity(
    state: &mut PhysState,
    body: &NetworkBody,
    frame: FrameIndex,
) -> Option<bool> {
    if !body
        .sleeping
        .as_ref()
        .is_some_and(|v| v.frame == frame && v.value)
    {
        return None;
    }
    let mut changed = false;
    if !body
        .linear_velocity
        .as_ref()
        .is_some_and(|v| v.frame == frame)
    {
        changed |= state.vel != Vec3A::ZERO;
        state.vel = Vec3A::ZERO;
    }
    if !body
        .angular_velocity_raw
        .as_ref()
        .is_some_and(|v| v.frame == frame)
    {
        changed |= state.ang_vel != Vec3A::ZERO;
        state.ang_vel = Vec3A::ZERO;
    }
    Some(changed)
}

/// The controls the network values say: throttle, steer and handbrake, boost and jump from their action
/// counters (odd while active). Pitch, yaw and roll are not in the replay.
pub(crate) fn network_controls(car: &NetworkCar) -> CarControls {
    let inputs = &car.inputs;
    CarControls {
        throttle: inputs.throttle.as_ref().map_or(0.0, |v| v.value),
        steer: inputs.steer.as_ref().map_or(0.0, |v| v.value),
        handbrake: inputs.handbrake.as_ref().is_some_and(|v| v.value),
        boost: inputs
            .boost_active_raw
            .as_ref()
            .is_some_and(|v| v.value % 2 == 1),
        jump: inputs
            .jump_active_raw
            .as_ref()
            .is_some_and(|v| v.value % 2 == 1),
        ..CarControls::default()
    }
}

/// A jump's impulse is not yet in the updates: the car is low and no update at `frame` shows it rising.
pub(super) fn jump_impulse_unseen(car: &NetworkCar, frame: FrameIndex) -> bool {
    car.body
        .position
        .as_ref()
        .is_some_and(|p| p.value[2] < 50.0)
        && !car
            .body
            .linear_velocity
            .as_ref()
            .is_some_and(|v| v.frame == frame && v.value[2] > 150.0)
}

/// A dodge's impulse is not yet in the updates: the car is airborne and has no velocity update at `frame`.
pub(super) fn dodge_impulse_unseen(car: &NetworkCar, frame: FrameIndex, state: &CarState) -> bool {
    let airborne = !state.is_on_ground || state.phys.pos.z > 50.0;
    airborne
        && !car
            .body
            .linear_velocity
            .as_ref()
            .is_some_and(|v| v.frame == frame)
}

/// The dodge torque of a car whose dodge counter turned odd at frame `g`. The replay sends the torque only
/// when it changes (a dodge in the same direction as the last has an old stamp: 22% of activations on a host
/// replay), and it can arrive a frame after the counter: the value visible a frame later is the one in
/// effect, unless the car has no later frame.
pub(crate) fn dodge_torque(
    frames: &[NetworkFrame],
    g: usize,
    car: &NetworkCar,
) -> Option<[f32; 3]> {
    let later = frames
        .get(g + 1)
        .and_then(|frame| frame.cars.iter().find(|c| c.life == car.life));
    later
        .unwrap_or(car)
        .inputs
        .dodge_torque_raw
        .as_ref()
        .or(car.inputs.dodge_torque_raw.as_ref())
        .map(|t| t.value)
}

/// RocketSim limits speeds at the start of each tick, so a state read after a step can exceed the limits
/// recorded server states obey (ROCKETSIM_NOTES.md, entry 2): apply them to the state.
pub(super) fn limit_velocities(arena: &mut Arena, car_count: usize) {
    const CAR_MAX_SPEED: f32 = 2300.0;
    const CAR_MAX_ANGULAR_SPEED: f32 = 5.5;
    const BALL_MAX_SPEED: f32 = 6000.0;
    const BALL_MAX_ANGULAR_SPEED: f32 = 6.0;
    let limit = |velocity: Vec3A, maximum: f32| {
        let speed = velocity.length();
        (speed > maximum).then(|| velocity * (maximum / speed))
    };
    for index in 0..car_count {
        let mut state = *arena.get_car_state(index);
        let linear = limit(state.phys.vel, CAR_MAX_SPEED);
        let angular = limit(state.phys.ang_vel, CAR_MAX_ANGULAR_SPEED);
        if linear.is_some() || angular.is_some() {
            state.phys.vel = linear.unwrap_or(state.phys.vel);
            state.phys.ang_vel = angular.unwrap_or(state.phys.ang_vel);
            arena.set_car_state(index, state);
        }
    }
    let mut ball = *arena.get_ball_state();
    let linear = limit(ball.phys.vel, BALL_MAX_SPEED);
    let angular = limit(ball.phys.ang_vel, BALL_MAX_ANGULAR_SPEED);
    if linear.is_some() || angular.is_some() {
        ball.phys.vel = linear.unwrap_or(ball.phys.vel);
        ball.phys.ang_vel = angular.unwrap_or(ball.phys.ang_vel);
        arena.set_ball_state(ball);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testkit::sent;

    /// A stale value does not overwrite simulated physics; a value of this frame does.
    #[test]
    fn only_values_of_this_frame_are_applied() {
        let body = NetworkBody {
            position: Some(sent([1.0, 2.0, 3.0], 7)),
            linear_velocity: Some(sent([10.0, 0.0, 0.0], 5)),
            ..NetworkBody::default()
        };
        let mut state = CarState::default().phys;
        state.vel = Vec3A::new(0.0, 99.0, 0.0);
        assert!(apply_update(&mut state, &body, FrameIndex(7), false));
        assert_eq!(state.pos, Vec3A::new(1.0, 2.0, 3.0));
        assert_eq!(state.vel, Vec3A::new(0.0, 99.0, 0.0));
        // A new body takes every value it has.
        assert!(apply_update(&mut state, &body, FrameIndex(9), true));
        assert_eq!(state.vel, Vec3A::new(10.0, 0.0, 0.0));
    }

    /// A sleeping update zeroes the velocities it omits and keeps one it carries.
    #[test]
    fn a_sleeping_update_zeroes_the_omitted_velocities() {
        let body = NetworkBody {
            sleeping: Some(sent(true, 4)),
            linear_velocity: Some(sent([5.0, 0.0, 0.0], 4)),
            ..NetworkBody::default()
        };
        let mut state = CarState::default().phys;
        state.vel = Vec3A::new(5.0, 0.0, 0.0);
        state.ang_vel = Vec3A::new(0.0, 1.0, 0.0);
        assert_eq!(
            zero_sleeping_velocity(&mut state, &body, FrameIndex(4)),
            Some(true)
        );
        assert_eq!(state.vel, Vec3A::new(5.0, 0.0, 0.0));
        assert_eq!(state.ang_vel, Vec3A::ZERO);
        assert_eq!(
            zero_sleeping_velocity(&mut state, &body, FrameIndex(5)),
            None
        );
    }
}
