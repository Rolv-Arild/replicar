//! Play segments and the `future` group (docs/glossary.md, "Play segment", "Future group").
//!
//! A frame is in play when the replay's game state is `Active`. Segment k (0-based, in replay order) starts at
//! the first frame in play of a kickoff and ends at the first frame that reports a goal (that frame is in the
//! segment), or at the last frame in play when no goal follows. Frames outside play, and the frames still in
//! play after a goal frame until play stops, are in no segment. The overtime kickoff starts a new segment
//! though no goal precedes it. This is v1's episode rule.
//!
//! How a segment ends, and the time until then, are FUTURE-DERIVED: read from the frames after the one they
//! describe, and only from the frame's own segment.

pub use replicar_format::SegmentEnd;
use replicar_format::Team;

use crate::annotate::scoreboard::{ClockPhase, Scoreboard};
use crate::decode::{GameState, NetworkEvent, NetworkFrame};

/// A frame's play segment and its future.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SegmentFrame {
    pub segment: u32,
    /// Future-derived: how the segment ends.
    pub future_segment_end: SegmentEnd,
    /// Future-derived: replay time from this frame to the segment's last frame (0 there).
    pub future_seconds_until_segment_end: f32,
}

/// One play segment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Segment {
    pub first: usize,
    pub last: usize,
    pub end: SegmentEnd,
}

/// The team that scored the frame's first goal (a goal report names the team scored on).
fn goal_of(frame: &NetworkFrame) -> Option<Team> {
    frame.events.iter().find_map(|event| match event {
        NetworkEvent::GoalScoredOn { team } => Some(match team {
            Team::Blue => Team::Orange,
            Team::Orange => Team::Blue,
        }),
        _ => None,
    })
}

/// Every play segment of the replay, in order. `scoreboard` is `scoreboard::reconstruct` of the same frames.
#[must_use]
pub fn segments(frames: &[NetworkFrame], scoreboard: &[Scoreboard]) -> Vec<Segment> {
    let in_play = |f: usize| {
        frames[f]
            .game_state
            .as_ref()
            .is_some_and(|s| s.value == GameState::Active)
    };
    // (first, last, the scoring team when it ended with a goal).
    let mut found: Vec<(usize, usize, Option<Team>)> = Vec::new();
    let mut open: Option<(usize, usize)> = None;
    // After a goal frame the rest of that stretch of play belongs to no segment.
    let mut after_goal = false;
    for f in 0..frames.len() {
        let play = in_play(f);
        if let Some(team) = goal_of(&frames[f]) {
            let segment = match open.take() {
                Some((first, _)) => Some(first),
                None if play && !after_goal => Some(f),
                None => None,
            };
            if let Some(first) = segment {
                found.push((first, f, Some(team)));
            }
            after_goal = true;
        } else if play {
            if after_goal {
                continue;
            }
            match &mut open {
                Some((_, last)) => *last = f,
                None => open = Some((f, f)),
            }
        } else {
            if let Some((first, last)) = open.take() {
                found.push((first, last, None));
            }
            after_goal = false;
        }
    }
    if let Some((first, last)) = open.take() {
        found.push((first, last, None));
    }
    found
        .into_iter()
        .map(|(first, last, goal)| Segment {
            first,
            last,
            end: match goal {
                Some(Team::Blue) => SegmentEnd::BlueGoal,
                Some(Team::Orange) => SegmentEnd::OrangeGoal,
                None if matches!(
                    scoreboard.get(last).map(|s| s.clock_phase),
                    Some(ClockPhase::Expired | ClockPhase::Decided)
                ) =>
                {
                    SegmentEnd::TimeExpired
                }
                None if last + 1 == frames.len() => SegmentEnd::ReplayEnded,
                None => SegmentEnd::Other,
            },
        })
        .collect()
}

/// Each frame's segment and future; `None` outside play segments.
#[must_use]
pub fn segment_frames(frames: &[NetworkFrame], segments: &[Segment]) -> Vec<Option<SegmentFrame>> {
    let mut out = vec![None; frames.len()];
    for (index, segment) in segments.iter().enumerate() {
        let end = f64::from(frames[segment.last].time);
        for f in segment.first..=segment.last {
            out[f] = Some(SegmentFrame {
                segment: index as u32,
                future_segment_end: segment.end,
                future_seconds_until_segment_end: (end - f64::from(frames[f].time)) as f32,
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::annotate::scoreboard::reconstruct;
    use crate::testkit::{frame, sent};

    /// Frames at 30 per second with the given game states; goals reported where `goal` names a team scored
    /// on.
    fn frames(states: &[(GameState, Option<Team>)]) -> Vec<NetworkFrame> {
        states
            .iter()
            .enumerate()
            .map(|(i, (state, goal))| {
                let mut f = frame(i as u32, i as f32 / 30.0, 1.0 / 30.0);
                f.game_state = Some(sent(state.clone(), i as u32));
                f.seconds_remaining = Some(sent(100, i as u32));
                if let Some(team) = goal {
                    f.events.push(NetworkEvent::GoalScoredOn { team: *team });
                }
                f
            })
            .collect()
    }

    #[test]
    fn a_segment_runs_from_the_kickoff_to_its_goal_frame_and_no_goal_is_said_so() {
        use GameState::{Active, Countdown, PostGoalScored};
        let frames = frames(&[
            (Countdown, None),
            (Active, None),
            (Active, None),
            // Orange is scored on: blue scores; the frame is the segment's last.
            (Active, Some(Team::Orange)),
            // Still in play after the goal: no segment.
            (Active, None),
            (PostGoalScored, Some(Team::Orange)),
            (Countdown, None),
            (Active, None),
            (Active, None),
            // Play stops without a goal and with the clock running: other.
            (Countdown, None),
            (Active, None),
            (Active, None),
        ]);
        let scoreboard = reconstruct(&frames);
        let found = segments(&frames, &scoreboard);
        assert_eq!(
            found,
            [
                Segment {
                    first: 1,
                    last: 3,
                    end: SegmentEnd::BlueGoal
                },
                Segment {
                    first: 7,
                    last: 8,
                    end: SegmentEnd::Other
                },
                Segment {
                    first: 10,
                    last: 11,
                    end: SegmentEnd::ReplayEnded
                },
            ]
        );
        let per_frame = segment_frames(&frames, &found);
        assert_eq!(per_frame[0], None);
        assert_eq!(per_frame[4], None);
        assert_eq!(per_frame[5], None);
        let first = per_frame[1].expect("in the first segment");
        assert_eq!(first.segment, 0);
        assert!((first.future_seconds_until_segment_end - 2.0 / 30.0).abs() < 1e-6);
        assert_eq!(
            per_frame[3].map(|f| f.future_seconds_until_segment_end),
            Some(0.0)
        );
        assert_eq!(per_frame[8].map(|f| f.segment), Some(1));
    }

    #[test]
    fn a_segment_that_ends_with_the_clock_expired_is_time_expired() {
        use GameState::{Active, Countdown};
        let mut frames = frames(&[
            (Active, None),
            (Active, None),
            (Active, None),
            (Countdown, None),
        ]);
        for (i, f) in frames.iter_mut().enumerate() {
            f.seconds_remaining = Some(sent(if i < 2 { 1 } else { 0 }, i as u32));
        }
        let scoreboard = reconstruct(&frames);
        assert!(matches!(
            scoreboard[2].clock_phase,
            ClockPhase::Expired | ClockPhase::Decided
        ));
        assert_eq!(
            segments(&frames, &scoreboard),
            [Segment {
                first: 0,
                last: 2,
                end: SegmentEnd::TimeExpired
            }]
        );
    }
}
