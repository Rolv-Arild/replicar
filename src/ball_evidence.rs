//! Ball-packet evidence of touches. Two fresh ball packets a few ticks apart are exact server
//! states; if nothing touched the ball in between, RocketSim rolling the ball alone from the first
//! packet reproduces the second (free flight to 0.1 UU/s, bounces included). A velocity the
//! rollout cannot reach at any elapsed tick the packet lags allow is a touch (or another contact the
//! ball-only arena lacks: a goal post, a demolished car).

use std::path::Path;

use rocketsim::{Arena, ArenaConfig, BallState, GameMode, Mat3A, Vec3A};

use crate::conversion::{ConvertOptions, PacketLags};
use crate::observations::ObservedReplay;

/// One interval between two consecutive fresh ball packets.
#[derive(Debug, Clone)]
pub struct BallInterval {
    pub frame_a: usize,
    pub frame_b: usize,
    /// Physical ticks of the two packets on the replay timeline (frame tick minus packet lag).
    pub tick_a: i64,
    pub tick_b: i64,
    /// The elapsed tick count (within one of the lags' estimate) at which the rollout is closest.
    pub best_ticks: i64,
    /// Smallest velocity difference (UU/s) between the rollout and the second packet.
    pub velocity_residual: f32,
    /// Position difference (UU) at that elapsed count.
    pub position_residual: f32,
    pub position_a: [f32; 3],
    pub position_b: [f32; 3],
    /// The ball-only rollout from the first packet, tick by tick (ticks 1..=best_ticks): where the
    /// ball would be without a touch.
    pub path: Vec<[f32; 3]>,
}

/// A velocity difference above this (UU/s) between the second packet and the ball-only rollout is
/// a contact. Measured on the two remote-client games (server truth): the rollout reproduces
/// quiet intervals to 0.02 UU/s (p99; walls and floor on a 10 fps client 4 UU/s), and the weakest
/// 1% of true touches differ by 46-150 UU/s, so every threshold between 2 and 40 finds all 140-221
/// touches per replay; 10 is in the middle of that range.
pub const CONTACT_VELOCITY_THRESHOLD: f32 = 10.0;

fn ball_state(body: &crate::observations::Body, frame: usize) -> Option<BallState> {
    let pos = body.position.as_ref().filter(|v| v.frame == frame)?;
    let vel = body.linear_velocity.as_ref().filter(|v| v.frame == frame)?;
    let mut state = BallState::default();
    state.phys.pos = Vec3A::from(pos.value);
    state.phys.vel = Vec3A::from(vel.value);
    if let Some(ang) = body.angular_velocity_replay_units.as_ref().filter(|v| v.frame == frame) {
        state.phys.ang_vel = Vec3A::from(ang.value) * 0.01;
    }
    if let Some(rot) = body.rotation_xyzw.as_ref().filter(|v| v.frame == frame) {
        if let Some(q) = crate::conversion::quaternion(rot.value) {
            state.phys.rot_mat = Mat3A::from_quat(q);
        }
    }
    Some(state)
}

/// Rolls the ball alone between every pair of consecutive fresh ball packets (at most three frames
/// apart, both with an inferred lag, all frames active) and reports how far the second packet is
/// from the rollout. Offline: uses the second packet.
pub fn ball_intervals(
    observations: &ObservedReplay,
    lags: &PacketLags,
    options: &ConvertOptions,
) -> Result<Vec<BallInterval>, Box<dyn std::error::Error>> {
    rocketsim::init(Path::new(&options.collision_meshes), true)?;
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    let frames = &observations.frames;
    let first_time = f64::from(frames.first().map_or(0.0, |f| f.time));
    let timeline = |frame: usize| ((f64::from(frames[frame].time) - first_time) * 120.0).round() as i64;
    let active = |frame: usize| {
        frames[frame].game_state.as_ref().is_some_and(|s| s.value == "Active")
    };
    let fresh: Vec<usize> = (0..frames.len())
        .filter(|&f| {
            frames[f]
                .ball
                .as_ref()
                .is_some_and(|b| ball_state(b, f).is_some())
        })
        .collect();
    let mut out = Vec::new();
    for pair in fresh.windows(2) {
        let (fa, fb) = (pair[0], pair[1]);
        let withheld = |f: usize| {
            options
                .withheld_frames
                .as_ref()
                .is_some_and(|w| w.get(f).copied().unwrap_or(false))
        };
        if fb - fa > 3 || !(fa..=fb).all(&active) || withheld(fa) || withheld(fb) {
            continue;
        }
        let (Some(a), Some(b)) = (
            frames[fa].ball.as_ref().and_then(|x| ball_state(x, fa)),
            frames[fb].ball.as_ref().and_then(|x| ball_state(x, fb)),
        ) else {
            continue;
        };
        // Elapsed ticks between the packets: from their inferred lags when both have one (then
        // within one tick of the estimate), always allowing every count the frame windows permit
        // (a packet's lag is in [0, its frame window]) as the fallback: the interval is quiet if any
        // allowed count reproduces the second packet.
        let window_lo = timeline(fb.saturating_sub(1)) - timeline(fa);
        let window_hi = timeline(fb) - timeline(fa.saturating_sub(1));
        let lag_range = match (lags.ball[fa], lags.ball[fb]) {
            (Some(lag_a), Some(lag_b)) => {
                let estimate = (timeline(fb) - lag_b.round() as i64) - (timeline(fa) - lag_a.round() as i64);
                Some((estimate - 1, estimate + 1, estimate))
            }
            _ => None,
        };
        let max_d = window_hi.max(lag_range.map_or(0, |r| r.1));
        if max_d < 1 || max_d > 60 {
            continue;
        }
        arena.set_ball_state(a);
        let mut states: Vec<(f32, f32, [f32; 3])> = Vec::new(); // (velocity residual, position residual, position) per elapsed tick
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
        let mut best = lag_range.and_then(|(lo, hi, _)| pick(lo, hi));
        if best.is_none_or(|(_, v, _)| v > CONTACT_VELOCITY_THRESHOLD) {
            if let Some(wide) = pick(window_lo, window_hi) {
                if best.is_none_or(|(_, v, _)| wide.1 < v) {
                    best = Some(wide);
                }
            }
        }
        let Some((best_ticks, velocity_residual, position_residual)) = best else {
            continue;
        };
        // Physical ticks of the packets: the frame time minus the inferred lag, or half the frame
        // window when a packet has none. A touched interval has no elapsed count that fits, so its
        // path is as long as the lags say.
        let half_window = |f: usize| (timeline(f) - timeline(f.saturating_sub(1))) / 2;
        let tick_a = timeline(fa) - lags.ball[fa].map_or(half_window(fa), |l| l.round() as i64);
        let tick_b = timeline(fb) - lags.ball[fb].map_or(half_window(fb), |l| l.round() as i64);
        let path_len = if velocity_residual > CONTACT_VELOCITY_THRESHOLD {
            (tick_b - tick_a).clamp(1, max_d)
        } else {
            best_ticks
        };
        let path: Vec<[f32; 3]> = states[..path_len as usize].iter().map(|s| s.2).collect();
        out.push(BallInterval {
            frame_a: fa,
            frame_b: fb,
            tick_a,
            tick_b,
            best_ticks,
            velocity_residual,
            position_residual,
            position_a: a.phys.pos.to_array(),
            position_b: b.phys.pos.to_array(),
            path,
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observations::{Body, Frame, Header, Source, Value};
    use rocketsim::Vec3A;

    fn value<T>(value: T, frame: usize) -> Option<Value<T>> {
        Some(Value {
            value,
            frame,
            source: Source::Replay,
        })
    }

    fn ball_frame(index: usize, time: f32, state: &BallState) -> Frame {
        Frame {
            index,
            time,
            delta: 1.0 / 30.0,
            ball: Some(Body {
                position: value(state.phys.pos.to_array(), index),
                linear_velocity: value(state.phys.vel.to_array(), index),
                ..Body::default()
            }),
            cars: Vec::new(),
            players: Vec::new(),
            team_scores: [None, None],
            seconds_remaining: None,
            overtime: None,
            game_state: value("Active".to_string(), index),
            events: Vec::new(),
            pad_pickups: Vec::new(),
        }
    }

    /// A ball rolled alone for four ticks between packets is a quiet interval; the same ball with a
    /// kick of 600 UU/s applied half way is a contact.
    #[test]
    fn a_kick_between_two_ball_packets_is_a_contact() {
        let options = ConvertOptions::default();
        rocketsim::init(Path::new(&options.collision_meshes), true).unwrap();
        let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
        let mut start = BallState::default();
        start.phys.pos = Vec3A::new(500.0, 200.0, 900.0);
        start.phys.vel = Vec3A::new(700.0, -300.0, 200.0);
        let mut run = |kick: Option<Vec3A>| -> Vec<BallState> {
            arena.set_ball_state(start);
            let mut states = vec![*arena.get_ball_state()];
            for tick in 1..=8 {
                if tick == 3 {
                    if let Some(kick) = kick {
                        let mut b = *arena.get_ball_state();
                        b.phys.vel += kick;
                        arena.set_ball_state(b);
                    }
                }
                arena.step_tick();
                states.push(*arena.get_ball_state());
            }
            states
        };
        for (kick, expect_contact) in [(None, false), (Some(Vec3A::new(0.0, 600.0, 100.0)), true)] {
            let states = run(kick);
            // Packets at ticks 0 and 4 of an 8 tick timeline at 30 fps (4 ticks per frame).
            let frames = vec![ball_frame(0, 0.0, &states[0]), ball_frame(1, 4.0 / 120.0, &states[4])];
            let replay = ObservedReplay {
                header: Header {
                    game_type: "TAGame.Replay_Soccar_TA".to_string(),
                    levels: Vec::new(),
                    final_team_scores: [None, None],
                },
                frames,
                diagnostics: Default::default(),
            };
            let lags = PacketLags {
                ball: vec![Some(0.0), Some(0.0)],
                cars: vec![None, None],
                ..PacketLags::default()
            };
            let intervals = ball_intervals(&replay, &lags, &options).unwrap();
            assert_eq!(intervals.len(), 1);
            let contact = intervals[0].velocity_residual > CONTACT_VELOCITY_THRESHOLD;
            assert_eq!(contact, expect_contact, "residual {}", intervals[0].velocity_residual);
        }
    }
}
