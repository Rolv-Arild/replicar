//! Air schedules: an airborne car's air controls over the interval to its next update, solved as a
//! boundary-value problem on both the rotation and the angular velocity at the end (offline).

use std::collections::HashMap;

use glam::{Mat3A, Vec3A};
use replicar_format::{FrameIndex, PlayerIndex};
use rocketsim::{Arena, BallState, CarControls, CarState};

use super::PressInFlight;
use super::fits::seed_car;
use crate::air::{self, AirControls};
use crate::decode::{CarLife, GameState, NetworkCar, NetworkFrame};
use crate::hitbox::Hitbox;
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
/// Shifts of a flip's start tried on the flip path, in order: beyond three ticks they are rare and cost 20% of a
/// conversion (RESULTS.md, "Code review before the freeze, and what a conversion costs").
const FLIP_START_SHIFTS: [i32; 7] = [0, -1, 1, -2, 2, -3, 3];
/// RocketSim's maximum angular speed of a car (rad/s), applied to a flipping car's end state.
const MAX_ANGULAR_SPEED: f32 = 5.5;

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
    /// The player's hitbox, for the scratch arena of the flip path.
    pub hitbox: Hitbox,
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
/// velocity, with the shift of a flip's start the solution chose. A car in free flight uses the analytic
/// model; a flipping car or one with a pending press (`press`) uses RocketSim itself in `scratch` (refused
/// without one), trying shifts of the flip's start: the first that reaches the end state, else the closest.
/// Refused on the ground, below `MIN_Z`, for a span across a dodge (other than the pending press), a frame out
/// of play or a withheld frame, for a span outside 2-90 ticks, and when the solution does not reach the end
/// state (a contact, a wall, an unseen flip).
pub(super) fn plan(
    frames: &[NetworkFrame],
    ticks: &UpdateTicks,
    overrides: &HashMap<(CarLife, usize), u64>,
    withheld: Withheld,
    query: &AirScheduleQuery,
    press: Option<PressInFlight>,
    scratch: Option<&mut Arena>,
) -> Option<(AirSchedule, i32)> {
    let AirScheduleQuery {
        index,
        car,
        state,
        ticks_before,
        player,
        now,
        ..
    } = *query;
    if state.is_on_ground || ((state.is_flipping || press.is_some()) && scratch.is_none()) {
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
        let dodge_in_span = other
            .inputs
            .dodge_active_raw
            .as_ref()
            .is_some_and(|d| d.frame.get() == g && d.value % 2 == 1);
        if dodge_in_span && press.is_none() {
            return None;
        }
        if let Some((rot_b, omega_b, z_b)) = rotation_at(other, g) {
            // The end update's ticks as the simulator applies them: at most the frame's gap.
            let gap = (timeline(g) - timeline(g - 1)).max(0);
            let before = match overrides.get(&(car.life, g)) {
                Some(&fitted) => fitted as i64,
                None => ticks
                    .cars
                    .get(&(car.life, FrameIndex(g as u32)))
                    .copied()
                    .or(ticks.car_median[g])
                    .map_or(gap / 2, i64::from),
            }
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
    // Segments of about four ticks (a frame at 30 frames per second); about eight for a flipping car, whose solve
    // flies RocketSim once per control: 15% faster conversions, the same held-out accuracy (RESULTS.md, "v2:
    // search for speed and accuracy gains").
    let flipping = scratch.is_some() && (state.is_flipping || press.is_some());
    let parts = total.div_ceil(if flipping { 8 } else { 4 }).max(1);
    let segments: Vec<u32> = (0..parts)
        .map(|i| total * (i + 1) / parts - total * i / parts)
        .collect();
    let constant = air::solve_span(rot_a, omega_a, omega_b, total, SPAN_REFINEMENTS);
    let prior = vec![constant; segments.len()];
    let tolerance = ROTATION_TOLERANCE_DEGREES.to_radians();
    let (solved, rot_error, omega_error, shift) = match scratch {
        Some(scratch) if state.is_flipping || press.is_some() => {
            let mut best: Option<(Vec<AirControls>, f32, f32, i32)> = None;
            for offset in FLIP_START_SHIFTS {
                let mut start = *state;
                start.phys.rot_mat = rot_a;
                start.phys.ang_vel = omega_a;
                let mut press_tick = press.map(|d| d.start_tick);
                if let Some(tick) = press_tick.as_mut() {
                    let shifted = *tick as i64 + i64::from(offset);
                    if shifted <= now as i64 || shifted > now as i64 + i64::from(total) {
                        continue;
                    }
                    *tick = shifted as u64;
                } else {
                    let shifted = start.flip_time + offset as f32 / 120.0;
                    if shifted < 0.0 {
                        continue;
                    }
                    start.flip_time = shifted;
                }
                let mut forward = |segments: &[air::Segment]| {
                    flip_forward(scratch, &start, now, press, press_tick, segments)
                };
                let (solved, rot_error, omega_error) =
                    air::solve_bvp_with(&mut forward, rot_b, omega_b, &segments, &prior);
                let within = rot_error <= tolerance && omega_error <= ANGULAR_VELOCITY_TOLERANCE;
                let better = best.as_ref().is_none_or(|b| {
                    rot_error / tolerance + omega_error / ANGULAR_VELOCITY_TOLERANCE
                        < b.1 / tolerance + b.2 / ANGULAR_VELOCITY_TOLERANCE
                });
                if better {
                    best = Some((solved, rot_error, omega_error, offset));
                }
                if within {
                    break;
                }
            }
            best?
        }
        _ => {
            let (solved, rot_error, omega_error) =
                air::solve_bvp(rot_a, omega_a, rot_b, omega_b, &segments, &prior);
            (solved, rot_error, omega_error, 0)
        }
    };
    // Written so that a NaN solution is refused too.
    if !(rot_error <= tolerance && omega_error <= ANGULAR_VELOCITY_TOLERANCE) {
        return None;
    }
    let mut entries = Vec::with_capacity(segments.len());
    let mut tick = now + 1;
    for (controls, n) in solved.iter().zip(&segments) {
        entries.push((tick, *controls));
        tick += u64::from(*n);
    }
    Some((
        AirSchedule {
            player,
            end_tick: now + u64::from(total),
            entries,
        },
        shift,
    ))
}

/// RocketSim as the forward model of a flipping car: the car alone in the scratch arena from `start`, the ball
/// parked far from it (a contact there would not be the real one), the press at `press_tick`, then the
/// segments' air controls. Returns the end rotation and the angular velocity limited to RocketSim's maximum.
fn flip_forward(
    scratch: &mut Arena,
    start: &CarState,
    now: u64,
    press: Option<PressInFlight>,
    press_tick: Option<u64>,
    segments: &[air::Segment],
) -> (Mat3A, Vec3A) {
    let mut parked = BallState::default();
    parked.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
    if (start.phys.pos - parked.phys.pos).length() < 600.0 {
        parked.phys.pos = Vec3A::new(3000.0, 4000.0, 300.0);
    }
    scratch.set_ball_state(parked);
    seed_car(scratch, *start, now);
    let mut tick = now;
    for &(controls, n) in segments {
        for _ in 0..n {
            tick += 1;
            let mut c = CarControls {
                pitch: controls.pitch,
                yaw: controls.yaw,
                roll: controls.roll,
                ..CarControls::default()
            };
            if let (Some(d), Some(at)) = (press, press_tick)
                && tick == at
            {
                c.jump = true;
                c.pitch = d.pitch;
                c.yaw = d.yaw;
                c.roll = 0.0;
            }
            scratch.set_car_controls(0, c);
            scratch.step_tick();
        }
    }
    let end = scratch.get_car_state(0);
    let mut omega = end.phys.ang_vel;
    let speed = omega.length();
    if speed > MAX_ANGULAR_SPEED {
        omega *= MAX_ANGULAR_SPEED / speed;
    }
    (end.phys.rot_mat, omega)
}
