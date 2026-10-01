//! The match clock and its lifecycle, reconstructed from the replay's integer clock, game state,
//! overtime flag and ball packets (offline: uses frames after the one it describes).
//!
//! Rules (checked against the server on the remote-client games): the clock starts at 5:00 and stays
//! there until the first touch of a kickoff (the server's phase 2 ends at the first touch at all 18
//! kickoffs); it then counts down in real time, stops while a goal is celebrated and replayed and
//! during the next countdown and kickoff, and resumes at the next first touch. At 0 the game waits
//! for the ball to touch the ground; then the leader wins or, when level, overtime starts: the
//! clock counts up from 0 from its first touch until someone scores.
//!
//! The replay shows the clock as an integer, the ceiling of the true value (regulation counts down,
//! the overtime counter counts up and is also a ceiling). Within one running stretch the true clock
//! is linear in the 120 Hz timeline (one second per 120 ticks), so every change of the integer
//! brackets one tick between two frames, and the intersection of the brackets of a stretch pins the
//! clock to a few ticks. The fractional clock is that line; a client replays the clock late by its
//! replication delay, which this inherits.

use serde::Serialize;

use crate::observations::ObservedReplay;

/// The scoreboard of one frame.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ScoreboardFrame {
    /// `regulation` or `overtime`.
    pub period: &'static str,
    /// `pregame`, `countdown`, `kickoff` (clock held until the first touch), `running`,
    /// `expired` (regulation at 0, waiting for the ball to touch the ground), `decided` (the ball
    /// has touched the ground after expiry: the game ends or goes to overtime), `goal_pause`
    /// (celebration, replay), `other`.
    pub clock_state: &'static str,
    /// Regulation clock in seconds, fractional while running, held while frozen; 0 after expiry.
    pub seconds_remaining: Option<f32>,
    /// Overtime time played in seconds (counts up), fractional while running.
    pub overtime_seconds: Option<f32>,
}

const TICKS_PER_SECOND: f64 = 120.0;
/// A fresh ball packet below this height (UU, centre; the ball's radius is 91.25) is a floor contact.
const GROUND_HEIGHT: f32 = 100.0;

struct Run {
    first: usize,
    last: usize,
    overtime: bool,
}

/// Reconstructs the scoreboard of every frame.
pub fn reconstruct(observations: &ObservedReplay) -> Vec<ScoreboardFrame> {
    let frames = &observations.frames;
    let first_time = f64::from(frames.first().map_or(0.0, |f| f.time));
    let timeline = |f: usize| (f64::from(frames[f].time) - first_time) * TICKS_PER_SECOND;
    let state_of = |f: usize| frames[f].game_state.as_ref().map(|s| s.value.as_str());
    let clock = |f: usize| frames[f].seconds_remaining.as_ref().map(|v| v.value);
    let mut overtime = vec![false; frames.len()];
    let mut seen_overtime = false;
    for (f, frame) in frames.iter().enumerate() {
        if frame.overtime.as_ref().is_some_and(|v| v.value) {
            seen_overtime = true;
        }
        overtime[f] = seen_overtime;
    }
    let mut out: Vec<ScoreboardFrame> = (0..frames.len())
        .map(|f| ScoreboardFrame {
            period: if overtime[f] { "overtime" } else { "regulation" },
            clock_state: match state_of(f) {
                Some("Countdown") => "countdown",
                Some("PostGoalScored") | Some("ReplayPlayback") => "goal_pause",
                Some("Active") => "kickoff",
                None | Some("WaitingForPlayers") | Some("PreGame") => "pregame",
                _ => "other",
            },
            seconds_remaining: None,
            overtime_seconds: None,
        })
        .collect();
    // Runs of Active frames in one period.
    let mut runs: Vec<Run> = Vec::new();
    for f in 0..frames.len() {
        if state_of(f) != Some("Active") {
            continue;
        }
        match runs.last_mut() {
            Some(run) if run.last + 1 == f && run.overtime == overtime[f] => run.last = f,
            _ => runs.push(Run {
                first: f,
                last: f,
                overtime: overtime[f],
            }),
        }
    }
    // The value the clock is frozen at when a run starts: where the previous regulation run ended.
    let mut regulation_frozen: Option<f64> = None;
    for run in &runs {
        // Split the run where the integer clock does something other than step by one.
        let mut start = run.first;
        let mut pieces: Vec<(usize, usize)> = Vec::new();
        for f in run.first..run.last {
            let step_ok = match (clock(f), clock(f + 1)) {
                (Some(a), Some(b)) => {
                    if run.overtime {
                        b == a || b == a + 1
                    } else {
                        b == a || b + 1 == a
                    }
                }
                _ => true,
            };
            if !step_ok {
                pieces.push((start, f));
                start = f + 1;
            }
        }
        pieces.push((start, run.last));
        for (first, last) in pieces {
            // Brackets of the line's offset from every change of the integer clock.
            let mut lo = f64::NEG_INFINITY;
            let mut hi = f64::INFINITY;
            let mut count = 0usize;
            let mut mids: Vec<f64> = Vec::new();
            for f in first + 1..=last {
                let (Some(prev), Some(now)) = (clock(f - 1), clock(f)) else {
                    continue;
                };
                if prev == now {
                    continue;
                }
                let (t_lo, t_hi) = (timeline(f - 1), timeline(f));
                // Regulation: x(t) = a - t/120 crossed `now` downward at c in (t_lo, t_hi]:
                // a = now + c/120. Overtime: x(t) = (t - t0)/120 crossed `now - 1` upward:
                // t0 = c - 120 (now - 1).
                let (b_lo, b_hi) = if run.overtime {
                    let k = f64::from(now - 1) * TICKS_PER_SECOND;
                    (t_lo - k, t_hi - k)
                } else {
                    let a0 = f64::from(now);
                    (a0 + t_lo / TICKS_PER_SECOND, a0 + t_hi / TICKS_PER_SECOND)
                };
                lo = lo.max(b_lo);
                hi = hi.min(b_hi);
                mids.push(0.5 * (b_lo + b_hi));
                count += 1;
            }
            let fit = if count == 0 {
                None
            } else if lo <= hi {
                Some(0.5 * (lo + hi))
            } else {
                mids.sort_by(|a, b| a.total_cmp(b));
                Some(mids[mids.len() / 2])
            };
            if run.overtime {
                // Seconds played: 0 until the first touch, then counting up.
                let start_tick = fit.unwrap_or_else(|| timeline(first));
                for f in first..=last {
                    let x = ((timeline(f) - start_tick) / TICKS_PER_SECOND).max(0.0);
                    out[f].overtime_seconds = Some(x as f32);
                    out[f].clock_state = if x > 0.0 { "running" } else { "kickoff" };
                }
                continue;
            }
            // Regulation: held at the frozen value until the line falls below it.
            let frozen = regulation_frozen.or_else(|| clock(first).map(f64::from));
            let mut end_value = frozen;
            let mut expired_at: Option<usize> = None;
            for f in first..=last {
                let line = fit.map(|a| a - timeline(f) / TICKS_PER_SECOND);
                let (x, running) = match (line, frozen) {
                    (Some(line), Some(frozen)) => {
                        if line >= frozen {
                            (frozen, false)
                        } else {
                            (line, true)
                        }
                    }
                    (Some(line), None) => (line, true),
                    (None, Some(frozen)) => (frozen, false),
                    (None, None) => continue,
                };
                if x <= 0.0 {
                    out[f].seconds_remaining = Some(0.0);
                    out[f].clock_state = "expired";
                    expired_at.get_or_insert(f);
                } else {
                    out[f].seconds_remaining = Some(x as f32);
                    out[f].clock_state = if running { "running" } else { "kickoff" };
                }
                end_value = Some(x.max(0.0));
            }
            regulation_frozen = end_value.or(regulation_frozen);
            // The ball touching the ground after expiry decides the game.
            if let Some(expired) = expired_at {
                let decided = (expired..=last).find(|&f| {
                    frames[f]
                        .ball
                        .as_ref()
                        .and_then(|b| b.position.as_ref())
                        .is_some_and(|p| p.frame == f && p.value[2] < GROUND_HEIGHT)
                });
                if let Some(decided) = decided {
                    for state in &mut out[decided..=last] {
                        state.clock_state = "decided";
                    }
                }
            }
        }
    }
    // A replay that ends while waiting for the ball ends at the final whistle: its last frame is
    // the decision (3 ticks before the server's end phase on the host replay of game 1).
    if let Some(last) = out.last_mut() {
        if last.clock_state == "expired" {
            last.clock_state = "decided";
        }
    }
    // Frozen frames between runs keep the value the clock stopped at.
    let mut held: Option<f32> = None;
    let mut held_overtime: Option<f32> = None;
    for f in 0..frames.len() {
        let paused = matches!(out[f].clock_state, "goal_pause" | "countdown");
        if overtime[f] {
            if let Some(v) = out[f].overtime_seconds {
                held_overtime = Some(v);
            } else if paused {
                out[f].overtime_seconds = held_overtime.or(Some(0.0));
            }
        } else if let Some(v) = out[f].seconds_remaining {
            held = Some(v);
        } else if paused {
            out[f].seconds_remaining = held.or_else(|| clock(f).map(|v| v as f32));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observations::{Frame, Header, Source, Value};

    fn value<T>(value: T, frame: usize) -> Option<Value<T>> {
        Some(Value {
            value,
            frame,
            source: Source::Replay,
        })
    }

    fn frame(index: usize, time: f32, state: &str, clock: i32, overtime: bool) -> Frame {
        Frame {
            index,
            time,
            delta: 1.0 / 30.0,
            ball: None,
            cars: Vec::new(),
            players: Vec::new(),
            team_scores: [None, None],
            seconds_remaining: value(clock, index),
            overtime: overtime.then(|| Value {
                value: true,
                frame: index,
                source: Source::Replay,
            }),
            game_state: value(state.to_string(), index),
            events: Vec::new(),
            pad_pickups: Vec::new(),
        }
    }

    /// A true clock that starts at 10 s when the kickoff touch happens at t = 1.0 s, runs to a goal at
    /// 4.3 s (clock 6.7 s), pauses, restarts at the next touch at t = 9.0 s, and expires at 15.7 s; the
    /// replay shows ceil of it at 30 frames per second.
    #[test]
    fn the_clock_is_recovered_to_a_few_ticks_with_its_pauses() {
        let mut frames = Vec::new();
        for i in 0..600 {
            let t = i as f32 / 30.0;
            let (state, remaining): (&str, f64) = if t < 1.0 {
                ("Active", 10.0)
            } else if t < 4.3 {
                ("Active", 10.0 - f64::from(t - 1.0))
            } else if t < 9.0 {
                ("PostGoalScored", 6.7)
            } else {
                ("Active", (6.7 - f64::from(t - 9.0)).max(0.0))
            };
            frames.push(frame(i, t, state, remaining.ceil() as i32, false));
        }
        let replay = ObservedReplay {
            header: Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames,
            diagnostics: Default::default(),
        };
        let sb = reconstruct(&replay);
        let truth = |t: f32| -> f64 {
            if t < 1.0 {
                10.0
            } else if t < 4.3 {
                10.0 - f64::from(t - 1.0)
            } else if t < 9.0 {
                6.7
            } else {
                (6.7 - f64::from(t - 9.0)).max(0.0)
            }
        };
        let mut worst = 0.0f64;
        for (i, s) in sb.iter().enumerate() {
            let t = i as f32 / 30.0;
            let x = f64::from(s.seconds_remaining.expect("clock"));
            worst = worst.max((x - truth(t)).abs());
        }
        assert!(worst < 0.05, "worst clock error {worst}");
        // States: held before the touch, running, paused, running again, expired at the end.
        assert_eq!(sb[10].clock_state, "kickoff");
        assert_eq!(sb[60].clock_state, "running");
        assert_eq!(sb[200].clock_state, "goal_pause");
        assert_eq!(sb[300].clock_state, "running");
        assert_eq!(sb[598].clock_state, "expired");
        // The last frame of a replay that ends in expiry is the decision.
        assert_eq!(sb[599].clock_state, "decided");
        assert_eq!(sb.last().unwrap().seconds_remaining, Some(0.0));
    }

    /// Overtime counts up from 0 at its kickoff touch (here t = 2.0 s) and the replay shows the ceiling.
    #[test]
    fn the_overtime_clock_counts_up_from_the_touch() {
        let mut frames = Vec::new();
        for i in 0..450 {
            let t = i as f32 / 30.0;
            let played = f64::from((t - 2.0).max(0.0));
            let (state, shown) = if t < 0.5 { ("Countdown", 0) } else { ("Active", played.ceil() as i32) };
            frames.push(frame(i, t, state, shown, true));
        }
        let replay = ObservedReplay {
            header: Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames,
            diagnostics: Default::default(),
        };
        let sb = reconstruct(&replay);
        assert_eq!(sb[10].clock_state, "countdown");
        assert_eq!(sb[30].clock_state, "kickoff");
        for (i, s) in sb.iter().enumerate().skip(30) {
            let t = i as f32 / 30.0;
            let truth = (t - 2.0).max(0.0);
            let x = s.overtime_seconds.expect("overtime clock");
            assert!((x - truth).abs() < 0.05, "frame {i}: {x} vs {truth}");
            assert_eq!(s.period, "overtime");
        }
        assert_eq!(sb[300].clock_state, "running");
    }
}
