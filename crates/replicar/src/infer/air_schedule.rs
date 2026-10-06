//! Air schedules: an airborne car's air controls over the interval to its next update, solved as a
//! boundary-value problem on both the rotation and the angular velocity at the end (offline).

use glam::{Mat3A, Vec3A};
use replicar_format::{FrameIndex, PlayerIndex};
use rocketsim::CarState;

use crate::air::{self, AirControls};
use crate::decode::{GameState, NetworkCar, NetworkFrame};
use crate::update_ticks::{UpdateTicks, Withheld, quaternion};

/// Below this height (UU) the car is not taken to be in free flight.
const MIN_Z: f32 = 30.0;
/// A solution must reach the end rotation within this (degrees) and angular velocity within this (rad/s).
const ROTATION_TOLERANCE_DEGREES: f32 = 3.0;
const ANGULAR_VELOCITY_TOLERANCE: f32 = 0.5;
/// The end update is looked for this many frames ahead.
const MAX_FRAMES_AHEAD: usize = 24;
/// Span solve refinements for the prior.
const SPAN_REFINEMENTS: usize = 1;

/// A car's air controls tick by tick until `end_tick`: (first sim tick, controls), in order.
#[derive(Debug, Clone, PartialEq)]
pub struct AirSchedule {
    pub player: PlayerIndex,
    pub end_tick: u64,
    pub entries: Vec<(u64, AirControls)>,
}

/// What the simulator knows when it asks for an air schedule.
pub struct AirScheduleQuery<'q> {
    /// The frame of the car's update.
    pub index: usize,
    pub car: &'q NetworkCar,
    /// The car's simulated state after the update.
    pub state: &'q CarState,
    /// The update's ticks before its frame.
    pub ticks_before: u64,
    pub player: PlayerIndex,
    /// The current sim tick.
    pub now: u64,
}

/// A car's position, rotation and angular velocity updated at `frame`.
fn rotation_at(car: &NetworkCar, frame: usize) -> Option<(Mat3A, Vec3A, f32)> {
    let at = |f: FrameIndex| f.get() == frame;
    let p = car.body.position.as_ref().filter(|v| at(v.frame))?;
    let r = car.body.rotation.as_ref().filter(|v| at(v.frame))?;
    let w = car
        .body
        .angular_velocity_raw
        .as_ref()
        .filter(|v| at(v.frame))?;
    Some((
        Mat3A::from_quat(quaternion(r.value)?),
        Vec3A::from(w.value) * 0.01,
        p.value[2],
    ))
}

/// The air schedule from the car's update at `query.index` to its next update with a rotation and angular
/// velocity. Refused on the ground, for a flipping car (whose flip is not free flight), below `MIN_Z`, for a
/// span across a dodge, a frame out of play or a withheld frame, for a span outside 2-90 ticks, and when the
/// solution does not reach the end state (a contact, a wall, an unseen flip).
pub(super) fn plan(
    frames: &[NetworkFrame],
    ticks: &UpdateTicks,
    withheld: Withheld,
    query: &AirScheduleQuery,
) -> Option<AirSchedule> {
    let AirScheduleQuery {
        index,
        car,
        state,
        ticks_before,
        player,
        now,
    } = *query;
    if state.is_on_ground || state.is_flipping {
        return None;
    }
    let first_time = f64::from(frames.first().map_or(0.0, |f| f.time));
    let timeline =
        |frame: usize| ((f64::from(frames[frame].time) - first_time) * 120.0).round() as i64;
    let (rot_a, omega_a, z_a) = rotation_at(car, index)?;
    let in_play = |frame: usize| {
        frames[frame]
            .game_state
            .as_ref()
            .is_some_and(|g| g.value == GameState::Active)
    };
    if z_a < MIN_Z || !in_play(index) {
        return None;
    }
    let same_car = |c: &&NetworkCar| c.life == car.life && c.player == car.player;
    let t_a = timeline(index) - ticks_before as i64;
    let mut end = None;
    for g in index + 1..=(index + MAX_FRAMES_AHEAD).min(frames.len() - 1) {
        if !in_play(g) || withheld.contains(g) {
            return None;
        }
        let other = frames[g].cars.iter().find(same_car)?;
        if other
            .inputs
            .dodge_active_raw
            .as_ref()
            .is_some_and(|d| d.frame.get() == g && d.value % 2 == 1)
        {
            return None;
        }
        if let Some((rot_b, omega_b, z_b)) = rotation_at(other, g) {
            // The end update's ticks as the simulator applies them: at most the frame's gap.
            let gap = (timeline(g) - timeline(g - 1)).max(0);
            let before = ticks
                .cars
                .get(&(car.life, FrameIndex(g as u32)))
                .copied()
                .or(ticks.car_median[g])
                .map_or(gap / 2, i64::from)
                .min(gap);
            end = Some((rot_b, omega_b, z_b, timeline(g) - before));
            break;
        }
    }
    let (rot_b, omega_b, z_b, t_b) = end?;
    let total = t_b - t_a;
    if z_b < MIN_Z || !(2..=90).contains(&total) {
        return None;
    }
    let total = total as u32;
    // Segments of about four ticks (a frame at 30 frames per second).
    let parts = total.div_ceil(4).max(1);
    let segments: Vec<u32> = (0..parts)
        .map(|i| total * (i + 1) / parts - total * i / parts)
        .collect();
    let constant = air::solve_span(rot_a, omega_a, omega_b, total, SPAN_REFINEMENTS);
    let prior = vec![constant; segments.len()];
    let (solved, rot_error, omega_error) =
        air::solve_bvp(rot_a, omega_a, rot_b, omega_b, &segments, &prior);
    // Written so that a NaN solution is refused too.
    if !(rot_error <= ROTATION_TOLERANCE_DEGREES.to_radians()
        && omega_error <= ANGULAR_VELOCITY_TOLERANCE)
    {
        return None;
    }
    let mut entries = Vec::with_capacity(segments.len());
    let mut tick = now + 1;
    for (controls, n) in solved.iter().zip(&segments) {
        entries.push((tick, *controls));
        tick += u64::from(*n);
    }
    Some(AirSchedule {
        player,
        end_tick: now + u64::from(total),
        entries,
    })
}
