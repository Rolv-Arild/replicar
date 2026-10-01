//! Car/ball tick alignment from the ball packets (offline).
//!
//! The ball packets say when a car touched the ball (`ball_evidence`) and what the ball did
//! afterwards (the velocity in the next packet). The car and the ball are placed on the timeline with
//! separately inferred packet lags, so their relative timing is only known to a tick or two, and a
//! tick is 8-25 UU of closing distance at the speeds of a hit: the simulated car may graze or miss
//! the ball, or hit it with the wrong impulse. For each contact interval the car packet before the
//! hit is placed -3..=+3 ticks off in a scratch arena, the hit is simulated to the second ball
//! packet, and the shift whose ball velocity is closest to that packet is kept when it beats the
//! unshifted one clearly. The kept shifts move the lags of those car packets; the converter then
//! runs again with them. The second ball packet is a future observation: this is a fit, labelled
//! as such, not a prediction.

use std::collections::HashMap;

use rocketsim::{Arena, ArenaConfig, BallState, CarControls, CarState, GameMode, Team};

use crate::conversion::{
    ConvertError, ConvertOptions, PacketLags, convert_observations_with, hitbox_config, infer_packet_lags,
    quaternion, step_tick_with_hit_impulse,
};
use crate::observations::{Body, ObservedReplay};

/// Shifts tried, in ticks the car packet is later (+) or earlier (-) than its lag says.
const SHIFTS: std::ops::RangeInclusive<i64> = -3..=3;
/// A shift is kept only when it lowers the ball velocity residual (UU/s) by at least this.
const MIN_IMPROVEMENT: f32 = 30.0;
/// The best shift must reproduce the ball's velocity this closely (UU/s) to be believed.
const MAX_RESIDUAL: f32 = 100.0;
/// The exported frame the short simulation starts from is at most this many ticks before the hit.
const MAX_LEAD_TICKS: i64 = 12;
/// Shifts within this (UU/s) of the best are equivalent; the smallest one wins.
const TIE: f32 = 10.0;

/// What the alignment did, for diagnostics.
#[derive(Debug, Default, Clone)]
pub struct AlignmentSummary {
    pub contacts: usize,
    pub fitted: usize,
    pub shifted: usize,
    pub shifts: HashMap<i64, usize>,
}

fn phys(body: &Body, frame: usize) -> Option<([f32; 3], [f32; 3], glam::Mat3A, [f32; 3])> {
    let pos = body.position.as_ref().filter(|v| v.frame == frame)?.value;
    let vel = body.linear_velocity.as_ref().filter(|v| v.frame == frame)?.value;
    let rot = body.rotation_xyzw.as_ref().filter(|v| v.frame == frame)?.value;
    let ang = body
        .angular_velocity_replay_units
        .as_ref()
        .filter(|v| v.frame == frame)
        .map_or([0.0; 3], |v| v.value);
    let q = quaternion(rot)?;
    Some((pos, vel, glam::Mat3A::from_quat(q), ang))
}

/// The lags with the car packets before contacts moved by their fitted shifts.
pub fn aligned_lags(
    observations: &ObservedReplay,
    options: &ConvertOptions,
) -> Result<(PacketLags, AlignmentSummary), ConvertError> {
    let mut lags = infer_packet_lags(observations, options);
    let mut summary = AlignmentSummary::default();
    // A lag-free replay (every lag zero) has nothing to align.
    if lags.car_actor.values().all(|l| *l == 0.0) {
        return Ok((lags, summary));
    }
    // First pass: the normal conversion, for its contacts and exported poses.
    let mut first = options.clone();
    first.align_contacts = false;
    first.external_packet_lags = None;
    let mut frames = Vec::new();
    let summary_pass = convert_observations_with(observations, &first, |converted, _, _| {
        frames.push(converted.clone());
        Ok(())
    })?;
    let frame_data = &observations.frames;
    let first_time = f64::from(frame_data.first().map_or(0.0, |f| f.time));
    let timeline = |f: usize| ((f64::from(frame_data[f].time) - first_time) * 120.0).round() as i64;
    let mut ball_arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    arena.add_car(Team::Blue, rocketsim::CarBodyConfig::OCTANE);
    let mut current_config = "octane".to_string();
    for converted in &frames {
        for contact in &converted.ball_contacts {
            let Some(slot) = contact.car_slot else { continue };
            let fb = converted.replay_frame;
            let fa = contact.frame_a;
            summary.contacts += 1;
            let Some(slot_info) = summary_pass.car_slots.iter().find(|s| s.slot == slot) else {
                continue;
            };
            let Some(car_actor) = frame_data[fb]
                .cars
                .iter()
                .find(|c| c.player_key.as_deref() == Some(slot_info.player_key.as_str()))
            else {
                continue;
            };
            // The last exported frame before the hit: the converter's own reconstruction of the car
            // and the ball at that tick (both on its timeline), the closest start for a short
            // simulation to the hit.
            let hit_tick = contact.tick as i64;
            let Some(start_frame) = (fa.saturating_sub(1)..=fb)
                .rev()
                .find(|&f| timeline(f) < hit_tick && hit_tick - timeline(f) <= MAX_LEAD_TICKS)
            else {
                continue;
            };
            let (Some(exported), Some(ball_b)) = (
                frames.get(start_frame).and_then(|f| f.state.cars.get(slot)),
                frame_data[fb].ball.as_ref().and_then(|b| phys(b, fb)),
            ) else {
                continue;
            };
            let exported_ball = frames[start_frame].state.ball;
            let tick_s = timeline(start_frame);
            let tick_b = contact.tick_to as i64;
            if tick_b <= tick_s || tick_b - tick_s > 40 {
                continue;
            }
            if slot_info.hitbox != current_config {
                arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
                arena.add_car(Team::Blue, hitbox_config(&slot_info.hitbox));
                current_config = slot_info.hitbox.clone();
            }
            let controls: CarControls = exported.1.controls;
            let parked = {
                let mut b = BallState::default();
                b.phys.pos = glam::Vec3A::new(0.0, 0.0, 1800.0);
                b
            };
            let mut residuals: Vec<(i64, f32)> = Vec::new();
            for shift in SHIFTS {
                // The car state of the frame placed `shift` ticks later; whichever object is placed
                // earlier runs alone until the other starts.
                if shift >= tick_b - tick_s {
                    continue;
                }
                arena.set_car_controls(0, controls);
                if shift >= 0 {
                    ball_arena.set_ball_state(exported_ball);
                    for _ in 0..shift {
                        ball_arena.step_tick();
                    }
                    arena.set_ball_state(*ball_arena.get_ball_state());
                    arena.set_car_state(0, exported.1);
                } else {
                    arena.set_ball_state(parked);
                    arena.set_car_state(0, exported.1);
                    for _ in 0..(-shift) {
                        arena.step_tick();
                    }
                    arena.set_ball_state(exported_ball);
                }
                for _ in (tick_s + shift.max(0))..tick_b {
                    step_tick_with_hit_impulse(&mut arena, options.apply_hit_extra_impulse);
                }
                let v = arena.get_ball_state().phys.vel;
                residuals.push((shift, (v - glam::Vec3A::from(ball_b.1)).length()));
            }
            let Some(&(_, r0)) = residuals.iter().find(|(s, _)| *s == 0) else { continue };
            let best = residuals.iter().map(|r| r.1).fold(f32::INFINITY, f32::min);
            summary.fitted += 1;
            if std::env::var_os("ALIGN_DEBUG").is_some() {
                eprintln!(
                    "CONTACT car_key={} lead={} residuals={:?}",
                    slot_info.player_key,
                    hit_tick - tick_s,
                    residuals.iter().map(|r| (r.0, r.1.round())).collect::<Vec<_>>()
                );
            }
            // Only a shift that reproduces the ball's velocity is believed.
            if best > MAX_RESIDUAL {
                continue;
            }
            if r0 - best < MIN_IMPROVEMENT {
                continue;
            }
            let chosen = residuals
                .iter()
                .filter(|(_, r)| *r <= best + TIE)
                .min_by_key(|(s, _)| s.abs())
                .map(|(s, _)| *s)
                .unwrap_or(0);
            if chosen == 0 {
                continue;
            }
            summary.shifted += 1;
            *summary.shifts.entry(chosen).or_default() += 1;
            // The car packets that ended before the hit moved with it: the last fresh one is
            // `chosen` ticks later than its lag said (a smaller lag).
            let packet_frame = (fa.saturating_sub(3)..=fb).rev().find(|&g| {
                let Some(car) = frame_data[g].cars.iter().find(|c| {
                    c.actor_id == car_actor.actor_id && c.actor_created_frame == car_actor.actor_created_frame
                }) else {
                    return false;
                };
                car.body.position.as_ref().is_some_and(|p| p.frame == g)
                    && lags.car_actor.get(&(car.actor_id, car.actor_created_frame, g)).is_some_and(|l| {
                        timeline(g) - l.round() as i64 <= hit_tick
                    })
            });
            if let Some(g) = packet_frame {
                if let Some(entry) = lags
                    .car_actor
                    .get_mut(&(car_actor.actor_id, car_actor.actor_created_frame, g))
                {
                    *entry = (*entry - chosen as f32).max(0.0);
                }
            }
        }
    }
    if std::env::var_os("ALIGN_DEBUG").is_some() {
        eprintln!("contact alignment: {summary:?}");
    }
    Ok((lags, summary))
}
