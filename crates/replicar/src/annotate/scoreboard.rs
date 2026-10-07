//! The match clock and its phase, reconstructed from the replay's integer clock, game state, overtime flag and
//! ball updates (offline: uses frames after the one it describes).
//!
//! Rules (checked against the server on the remote-client games): the clock starts at 5:00 and stays there
//! until the first touch of a kickoff; it then counts down in real time, stops while a goal is celebrated and
//! replayed and during the next countdown and kickoff, and resumes at the next first touch. At 0 the game
//! waits for the ball to touch the ground; then the leader wins or, when level, overtime starts: the clock
//! counts up from 0 from its first touch until someone scores.
//!
//! The replay shows the clock as an integer, the ceiling of the true value (regulation counts down, the
//! overtime counter counts up and is also a ceiling). Within one running stretch the true clock is linear in
//! the replay ticks (one second per 120), so every change of the integer brackets one tick between two
//! frames, and the intersection of the brackets of a stretch pins the clock to a few ticks. A client replays
//! the clock late by its replication delay, which this inherits.

pub use replicar_format::{ClockPhase, Period};
use rocketsim::ArenaEvent;

use crate::decode::{GameState, NetworkFrame};
use crate::simulate::SimEvent;

const TICKS_PER_SECOND: f64 = 120.0;
/// A ball update below this height (UU, centre; the ball's radius is 91.25) is a floor contact.
const GROUND_HEIGHT: f32 = 100.0;

/// The scoreboard of one frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Scoreboard {
    pub period: Period,
    pub clock_phase: ClockPhase,
    /// The regulation clock in seconds, fractional while running, held while frozen; 0 after expiry.
    pub seconds_remaining: Option<f32>,
    /// Overtime played in seconds (counts up), fractional while running.
    pub overtime_seconds: Option<f32>,
}

/// One stretch of in-play frames in one period.
struct Run {
    first: usize,
    last: usize,
    overtime: bool,
}

/// The scoreboard of every frame.
#[must_use]
pub fn reconstruct(frames: &[NetworkFrame]) -> Vec<Scoreboard> {
    let first_time = f64::from(frames.first().map_or(0.0, |f| f.time));
    let timeline = |f: usize| (f64::from(frames[f].time) - first_time) * TICKS_PER_SECOND;
    let state_of = |f: usize| frames[f].game_state.as_ref().map(|s| &s.value);
    let active = |f: usize| state_of(f) == Some(&GameState::Active);
    let clock = |f: usize| frames[f].seconds_remaining.as_ref().map(|v| v.value);
    let mut overtime = vec![false; frames.len()];
    let mut seen_overtime = false;
    for (f, frame) in frames.iter().enumerate() {
        if frame.overtime.as_ref().is_some_and(|v| v.value) {
            seen_overtime = true;
        }
        overtime[f] = seen_overtime;
    }
    let mut out: Vec<Scoreboard> = (0..frames.len())
        .map(|f| Scoreboard {
            period: if overtime[f] {
                Period::Overtime
            } else {
                Period::Regulation
            },
            clock_phase: match state_of(f) {
                Some(GameState::Countdown) => ClockPhase::Countdown,
                Some(GameState::PostGoalScored | GameState::ReplayPlayback) => {
                    ClockPhase::GoalPause
                }
                Some(GameState::Active) => ClockPhase::Kickoff,
                None | Some(GameState::WaitingForPlayers | GameState::PreGame) => {
                    ClockPhase::Pregame
                }
                Some(GameState::Other(_)) => ClockPhase::Other,
            },
            seconds_remaining: None,
            overtime_seconds: None,
        })
        .collect();
    let mut runs: Vec<Run> = Vec::new();
    for f in (0..frames.len()).filter(|&f| active(f)) {
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
                (Some(a), Some(b)) if run.overtime => b == a || b == a + 1,
                (Some(a), Some(b)) => b == a || b + 1 == a,
                _ => true,
            };
            if !step_ok {
                pieces.push((start, f));
                start = f + 1;
            }
        }
        pieces.push((start, run.last));
        for (first, last) in pieces {
            let fit = fit_line(first, last, run.overtime, &clock, &timeline);
            if run.overtime {
                // Seconds played: 0 until the first touch, then counting up. Without a change of the integer
                // there is no evidence the clock moved: held at 0.
                let Some(start_tick) = fit else {
                    for state in &mut out[first..=last] {
                        state.overtime_seconds = Some(0.0);
                        state.clock_phase = ClockPhase::Kickoff;
                    }
                    continue;
                };
                for f in first..=last {
                    let x = ((timeline(f) - start_tick) / TICKS_PER_SECOND).max(0.0);
                    out[f].overtime_seconds = Some(x as f32);
                    out[f].clock_phase = if x > 0.0 {
                        ClockPhase::Running
                    } else {
                        ClockPhase::Kickoff
                    };
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
                    (Some(line), Some(frozen)) if line >= frozen => (frozen, false),
                    (Some(line), _) => (line, true),
                    (None, Some(frozen)) => (frozen, false),
                    (None, None) => continue,
                };
                if x <= 0.0 {
                    out[f].seconds_remaining = Some(0.0);
                    out[f].clock_phase = ClockPhase::Expired;
                    expired_at.get_or_insert(f);
                } else {
                    out[f].seconds_remaining = Some(x as f32);
                    out[f].clock_phase = if running {
                        ClockPhase::Running
                    } else {
                        ClockPhase::Kickoff
                    };
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
                        .is_some_and(|p| p.frame.get() == f && p.value[2] < GROUND_HEIGHT)
                });
                if let Some(decided) = decided {
                    for state in &mut out[decided..=last] {
                        state.clock_phase = ClockPhase::Decided;
                    }
                }
            }
        }
    }
    // A replay that ends while waiting for the ball ends at the final whistle: its last frame is the decision
    // (3 ticks before the server's end phase on the host replay of game 1).
    if let Some(last) = out.last_mut()
        && last.clock_phase == ClockPhase::Expired
    {
        last.clock_phase = ClockPhase::Decided;
    }
    // Frozen frames between runs keep the value the clock stopped at.
    let mut held: Option<f32> = None;
    let mut held_overtime: Option<f32> = None;
    for f in 0..frames.len() {
        let paused = matches!(
            out[f].clock_phase,
            ClockPhase::GoalPause | ClockPhase::Countdown
        );
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

/// The clock line of one running stretch from every change of the integer clock: the regulation clock's
/// value at replay tick 0, or the overtime clock's start tick. `None` without a change.
fn fit_line(
    first: usize,
    last: usize,
    overtime: bool,
    clock: &impl Fn(usize) -> Option<i32>,
    timeline: &impl Fn(usize) -> f64,
) -> Option<f64> {
    let mut lo = f64::NEG_INFINITY;
    let mut hi = f64::INFINITY;
    let mut mids: Vec<f64> = Vec::new();
    for f in first + 1..=last {
        let (Some(prev), Some(now)) = (clock(f - 1), clock(f)) else {
            continue;
        };
        if prev == now {
            continue;
        }
        let (t_lo, t_hi) = (timeline(f - 1), timeline(f));
        // Regulation: x(t) = a - t/120 crossed `now` downward at c in (t_lo, t_hi]: a = now + c/120.
        // Overtime: x(t) = (t - t0)/120 crossed `now - 1` upward: t0 = c - 120 (now - 1).
        let (b_lo, b_hi) = if overtime {
            let k = f64::from(now - 1) * TICKS_PER_SECOND;
            (t_lo - k, t_hi - k)
        } else {
            let a0 = f64::from(now);
            (a0 + t_lo / TICKS_PER_SECOND, a0 + t_hi / TICKS_PER_SECOND)
        };
        lo = lo.max(b_lo);
        hi = hi.min(b_hi);
        mids.push(0.5 * (b_lo + b_hi));
    }
    if mids.is_empty() {
        None
    } else if lo <= hi {
        Some(0.5 * (lo + hi))
    } else {
        mids.sort_by(f64::total_cmp);
        Some(mids[mids.len() / 2])
    }
}

/// Follows the simulation after expiry: the simulated ball's first floor contact decides the game, also when
/// no ball update caught it. Use one per simulation, in frame order.
#[derive(Debug, Default)]
pub struct Decider {
    decided: bool,
}

impl Decider {
    /// The frame's scoreboard with the simulation's floor contacts (`events`) applied.
    #[must_use]
    pub fn apply(&mut self, mut scoreboard: Scoreboard, events: &[SimEvent]) -> Scoreboard {
        if matches!(
            scoreboard.clock_phase,
            ClockPhase::Expired | ClockPhase::Decided
        ) {
            if events.iter().any(
                |e| matches!(&e.event, ArenaEvent::BallHitWorld(h) if h.contact_normal.z > 0.9),
            ) {
                self.decided = true;
            }
            if self.decided {
                scoreboard.clock_phase = ClockPhase::Decided;
            }
        } else {
            self.decided = false;
        }
        scoreboard
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testkit::{frame, sent};

    fn clock_frame(index: u32, state: GameState, clock: i32, overtime: bool) -> NetworkFrame {
        let mut f = frame(index, index as f32 / 30.0, 1.0 / 30.0);
        f.seconds_remaining = Some(sent(clock, index));
        f.overtime = overtime.then(|| sent(true, index));
        f.game_state = Some(sent(state, index));
        f
    }

    /// A true clock that starts at 10 s when the kickoff touch happens at t = 1.0 s, runs to a goal at 4.3 s
    /// (clock 6.7 s), pauses, restarts at the next touch at t = 9.0 s, and expires at 15.7 s; the replay shows
    /// its ceiling at 30 frames per second.
    #[test]
    fn the_clock_is_recovered_to_a_few_ticks_with_its_pauses() {
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
        let frames: Vec<NetworkFrame> = (0..600)
            .map(|i| {
                let t = i as f32 / 30.0;
                let state = if (4.3..9.0).contains(&t) {
                    GameState::PostGoalScored
                } else {
                    GameState::Active
                };
                clock_frame(i, state, truth(t).ceil() as i32, false)
            })
            .collect();
        let sb = reconstruct(&frames);
        let worst = sb
            .iter()
            .enumerate()
            .map(|(i, s)| {
                (f64::from(s.seconds_remaining.expect("clock")) - truth(i as f32 / 30.0)).abs()
            })
            .fold(0.0, f64::max);
        assert!(worst < 0.05, "worst clock error {worst}");
        assert_eq!(sb[10].clock_phase, ClockPhase::Kickoff);
        assert_eq!(sb[60].clock_phase, ClockPhase::Running);
        assert_eq!(sb[200].clock_phase, ClockPhase::GoalPause);
        assert_eq!(sb[300].clock_phase, ClockPhase::Running);
        assert_eq!(sb[598].clock_phase, ClockPhase::Expired);
        // The last frame of a replay that ends in expiry is the decision.
        assert_eq!(sb[599].clock_phase, ClockPhase::Decided);
        assert_eq!(sb[599].seconds_remaining, Some(0.0));
    }

    /// Overtime counts up from 0 at its kickoff touch (here t = 2.0 s) and the replay shows the ceiling.
    #[test]
    fn the_overtime_clock_counts_up_from_the_touch() {
        let frames: Vec<NetworkFrame> = (0..450)
            .map(|i| {
                let t = i as f32 / 30.0;
                let played = f64::from((t - 2.0).max(0.0));
                if t < 0.5 {
                    clock_frame(i, GameState::Countdown, 0, true)
                } else {
                    clock_frame(i, GameState::Active, played.ceil() as i32, true)
                }
            })
            .collect();
        let sb = reconstruct(&frames);
        assert_eq!(sb[10].clock_phase, ClockPhase::Countdown);
        assert_eq!(sb[30].clock_phase, ClockPhase::Kickoff);
        for (i, s) in sb.iter().enumerate().skip(30) {
            let truth = (i as f32 / 30.0 - 2.0).max(0.0);
            let x = s.overtime_seconds.expect("overtime clock");
            assert!((x - truth).abs() < 0.05, "frame {i}: {x} vs {truth}");
            assert_eq!(s.period, Period::Overtime);
        }
        assert_eq!(sb[300].clock_phase, ClockPhase::Running);
    }
}
