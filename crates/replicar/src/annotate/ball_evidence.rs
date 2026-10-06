//! Ball contacts from the ball's own updates. Two ball updates a few ticks apart are exact server states: if
//! nothing touched the ball in between, RocketSim rolling the ball alone from the first reproduces the second
//! (free flight to 0.1 UU/s, bounces included). A velocity the rollout cannot reach at any elapsed tick the
//! update ticks allow is a contact (a touch, or a goal post or wreck the ball-only arena lacks).

use glam::{Mat3A, Vec3A};
use replicar_format::FrameIndex;
use rocketsim::{Arena, ArenaConfig, BallState, GameMode};

use crate::decode::{GameState, NetworkBody, NetworkFrame};
use crate::update_ticks::{UpdateTicks, Withheld, quaternion};

/// A velocity difference above this (UU/s) between the second update and the ball-only rollout is a contact.
/// On the two remote-client games (server truth) quiet intervals reproduce to 0.02 UU/s (p99) and the weakest
/// 1% of touches differ by 46-150 UU/s, so every threshold from 2 to 40 finds all touches; 10 is in the
/// middle.
pub const CONTACT_VELOCITY_THRESHOLD: f32 = 10.0;

/// One interval between two consecutive ball updates.
#[derive(Debug, Clone)]
pub struct BallInterval {
    pub frame_a: usize,
    pub frame_b: usize,
    /// The updates' ticks on the replay timeline.
    pub tick_a: i64,
    pub tick_b: i64,
    /// The elapsed ticks at which the rollout comes closest to the second update.
    pub best_ticks: i64,
    /// The smallest velocity difference (UU/s) between the rollout and the second update, and the position
    /// difference (UU) there.
    pub velocity_residual: f32,
    pub position_residual: f32,
    /// The ball-only rollout from the first update, tick by tick: where the ball would be without a contact.
    pub path: Vec<[f32; 3]>,
}

fn ball_state(body: &NetworkBody, frame: usize) -> Option<BallState> {
    let at = |f: FrameIndex| f.get() == frame;
    let pos = body.position.as_ref().filter(|v| at(v.frame))?;
    let vel = body.linear_velocity.as_ref().filter(|v| at(v.frame))?;
    let mut state = BallState::default();
    state.phys.pos = Vec3A::from(pos.value);
    state.phys.vel = Vec3A::from(vel.value);
    if let Some(ang) = body.angular_velocity_raw.as_ref().filter(|v| at(v.frame)) {
        state.phys.ang_vel = Vec3A::from(ang.value) * 0.01;
    }
    if let Some(rot) = body.rotation.as_ref().filter(|v| at(v.frame))
        && let Some(q) = quaternion(rot.value)
    {
        state.phys.rot_mat = Mat3A::from_quat(q);
    }
    Some(state)
}

/// Rolls the ball alone between every two consecutive ball updates (at most three frames apart, all frames in
/// play, neither withheld) and reports how far the second update is from the rollout. Offline: uses the
/// second update. RocketSim's meshes must be loaded.
#[must_use]
pub fn ball_intervals(
    frames: &[NetworkFrame],
    ticks: &UpdateTicks,
    withheld: Withheld,
) -> Vec<BallInterval> {
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    let first_time = f64::from(frames.first().map_or(0.0, |f| f.time));
    let timeline =
        |frame: usize| ((f64::from(frames[frame].time) - first_time) * 120.0).round() as i64;
    let in_play = |frame: usize| {
        frames[frame]
            .game_state
            .as_ref()
            .is_some_and(|s| s.value == GameState::Active)
    };
    let updated: Vec<usize> = (0..frames.len())
        .filter(|&f| {
            frames[f]
                .ball
                .as_ref()
                .is_some_and(|b| ball_state(b, f).is_some())
        })
        .collect();
    let mut out = Vec::new();
    for pair in updated.windows(2) {
        let (fa, fb) = (pair[0], pair[1]);
        if fb - fa > 3 || !(fa..=fb).all(in_play) || withheld.contains(fa) || withheld.contains(fb)
        {
            continue;
        }
        let (Some(a), Some(b)) = (
            frames[fa].ball.as_ref().and_then(|x| ball_state(x, fa)),
            frames[fb].ball.as_ref().and_then(|x| ball_state(x, fb)),
        ) else {
            continue;
        };
        // The elapsed ticks: within one of the estimate from both updates' ticks, and every count the frame
        // windows allow as the fallback (the interval is quiet if any allowed count reproduces the update).
        let window_lo = timeline(fb.saturating_sub(1)) - timeline(fa);
        let window_hi = timeline(fb) - timeline(fa.saturating_sub(1));
        let estimate = match (ticks.ball[fa], ticks.ball[fb]) {
            (Some(before_a), Some(before_b)) => {
                let estimate =
                    (timeline(fb) - i64::from(before_b)) - (timeline(fa) - i64::from(before_a));
                Some((estimate - 1, estimate + 1))
            }
            _ => None,
        };
        let max_d = window_hi.max(estimate.map_or(0, |r| r.1));
        if !(1..=60).contains(&max_d) {
            continue;
        }
        arena.set_ball_state(a);
        // (velocity residual, position residual, position) per elapsed tick.
        let mut states: Vec<(f32, f32, [f32; 3])> = Vec::new();
        for _ in 1..=max_d {
            arena.step_tick();
            let state = arena.get_ball_state();
            states.push((
                (state.phys.vel - b.phys.vel).length(),
                (state.phys.pos - b.phys.pos).length(),
                state.phys.pos.to_array(),
            ));
        }
        let pick = |lo: i64, hi: i64| -> Option<(i64, f32, f32)> {
            (lo.max(1)..=hi.min(max_d))
                .map(|d| (d, states[d as usize - 1].0, states[d as usize - 1].1))
                .min_by(|x, y| x.1.total_cmp(&y.1))
        };
        let mut best = estimate.and_then(|(lo, hi)| pick(lo, hi));
        if best.is_none_or(|(_, v, _)| v > CONTACT_VELOCITY_THRESHOLD)
            && let Some(wide) = pick(window_lo, window_hi)
            && best.is_none_or(|(_, v, _)| wide.1 < v)
        {
            best = Some(wide);
        }
        let Some((best_ticks, velocity_residual, position_residual)) = best else {
            continue;
        };
        // The updates' ticks: the frame's tick minus the update's ticks before it, or half the frame window.
        // A contact interval has no elapsed count that fits, so its path is as long as the ticks say.
        let half_window = |f: usize| (timeline(f) - timeline(f.saturating_sub(1))) / 2;
        let tick_a = timeline(fa) - ticks.ball[fa].map_or(half_window(fa), i64::from);
        let tick_b = timeline(fb) - ticks.ball[fb].map_or(half_window(fb), i64::from);
        let path_len = if velocity_residual > CONTACT_VELOCITY_THRESHOLD {
            (tick_b - tick_a).clamp(1, max_d)
        } else {
            best_ticks
        };
        out.push(BallInterval {
            frame_a: fa,
            frame_b: fb,
            tick_a,
            tick_b,
            best_ticks,
            velocity_residual,
            position_residual,
            path: states[..path_len as usize].iter().map(|s| s.2).collect(),
        });
    }
    out
}
