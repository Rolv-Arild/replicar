//! Convenience training labels, kept apart from the state and the observations.
//!
//! Most of them are FUTURE-DERIVED on purpose: a frame's episode, its time to the end of the episode and
//! the next goal are read from frames after it (the replay's observed goal events). They are outputs
//! only; nothing here feeds the conversion, the state or the scoreboard, so the rule that final scores
//! never leak into earlier frames of the state still holds. The exception is `update_age_seconds`,
//! which is observed (from the frame and earlier ones only).
//!
//! Episodes. Frame `f` is in play when the replay's game state is `Active` (the scoreboard's
//! `kickoff`, `running`, `expired` and `decided` clock states). Episode k (0-based, counted in replay
//! order) starts at the first in-play frame of a kickoff and ends at the first frame carrying an observed
//! `goal_scored_on` event (the goal frame, which is in the episode and has 0 s remaining), or at the last
//! in-play frame when no goal follows (regulation expiring, which is the replay's last frame when the
//! replay ends in play). Frames outside play (pregame, countdown, goal pause and replay, anything after
//! the goal frame until the next kickoff) have no episode: the label is null, never zero, and they are
//! not assigned to the previous or the next episode. The overtime kickoff starts a new episode even
//! though no goal precedes it.
//!
//! Goals. The scoring team is the opposite of the team a `goal_scored_on` event names (the team scored
//! on). The next goal of frame `f` is the first goal event at a frame `>= f`, whether or not `f` is in
//! the same episode (a frame of a goal pause looks forward to the goal of the next episode), so the goal
//! frame itself has `seconds_until_next_goal` 0. Times are differences of the frames' `replay_time`
//! (the goal is dated by the frame that reports it, which a client sees late by its replication delay).

use serde::Serialize;

use crate::conversion::ConvertedFrame;
use crate::observations::{Car, Event, Frame, ObservedReplay, primary_linked_cars};

/// The labels of one replay frame (`labels` in a JSONL frame, `label_*` columns in Parquet).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FrameLabels {
    /// Future-derived. The goal-to-goal segment of this frame (0-based); null outside play.
    pub episode: Option<u32>,
    /// Future-derived. Replay time from this frame to the end of its episode (its goal frame, or the last
    /// in-play frame); null outside an episode.
    pub episode_seconds_remaining: Option<f32>,
    /// Future-derived. The team (0 blue, 1 orange) that scores the next goal at or after this frame, from
    /// the observed goal events; null when no goal follows.
    pub next_scoring_team: Option<u8>,
    /// Future-derived. Replay time from this frame to that goal's frame; null when no goal follows.
    pub seconds_until_next_goal: Option<f32>,
    /// Observed, not future-derived. Per car slot: this frame's time minus the time of the frame that
    /// carried the last rigid-body packet of the slot's car (0 when the packet is in this frame). Null for
    /// a slot with no car in this frame's state and before the car's first packet (its current actor has
    /// none yet, a spawn pose holds it).
    pub update_age_seconds: Vec<Option<f32>>,
}

/// Header-level labels, final and future-derived (`labels` in the JSONL header).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct HeaderLabels {
    /// Always true: every value below is read from the end of the replay.
    pub future_derived: bool,
    /// Blue and orange score on the replay's own last frame; null for a team whose score the replay never
    /// showed. A team that never scored may read 0 from `inferred_match_start`
    /// (`final_score_sources`). Not the header properties.
    pub final_score: [Option<i32>; 2],
    /// `replay` or `inferred_match_start` for each final score (`observations::Source`).
    pub final_score_sources: [Option<&'static str>; 2],
    /// 0 or 1: the team with the higher final score; null for a draw and when either score is unknown.
    pub winning_team: Option<u8>,
    /// Number of episodes (`FrameLabels::episode` runs over `0..episodes`).
    pub episodes: u32,
    /// Observed goal events per scoring team (blue, orange).
    pub observed_goals: [u32; 2],
}

/// The scoring team of a goal event, which names the team scored on.
fn scoring_team(scored_on: u8) -> u8 {
    1 - scored_on.min(1)
}

/// The first goal event of a frame, as (scoring team).
fn goal_of(frame: &Frame) -> Option<u8> {
    frame.events.iter().find_map(|event| match event {
        Event::GoalScoredOn { team } => Some(scoring_team(*team)),
        _ => None,
    })
}

fn in_play(frame: &Frame) -> bool {
    frame.game_state.as_ref().is_some_and(|state| state.value == "Active")
}

/// Labels that need the whole replay, computed once.
pub struct ReplayLabels {
    episode: Vec<Option<u32>>,
    /// The frame index each frame's episode ends at.
    episode_end: Vec<Option<usize>>,
    /// The first goal at or after each frame: (frame index, scoring team).
    next_goal: Vec<Option<(usize, u8)>>,
    episodes: u32,
}

impl ReplayLabels {
    pub fn new(observed: &ObservedReplay) -> Self {
        let frames = &observed.frames;
        let count = frames.len();
        let goals: Vec<Option<u8>> = frames.iter().map(goal_of).collect();
        let mut next_goal = vec![None; count];
        let mut upcoming = None;
        for index in (0..count).rev() {
            if let Some(team) = goals[index] {
                upcoming = Some((index, team));
            }
            next_goal[index] = upcoming;
        }
        let mut episode = vec![None; count];
        let mut episode_end = vec![None; count];
        let mut episodes = 0u32;
        // The open episode: its index and the first and last frame assigned to it so far.
        let mut open: Option<(u32, usize, usize)> = None;
        // After a goal frame the rest of that run of in-play frames belongs to no episode.
        let mut after_goal = false;
        fn close(episode_end: &mut [Option<usize>], (_, first, _): (u32, usize, usize), end: usize) {
            for slot in &mut episode_end[first..=end] {
                *slot = Some(end);
            }
        }
        for index in 0..count {
            let play = in_play(&frames[index]);
            if goals[index].is_some() {
                let target = match open.take() {
                    Some(segment) => Some(segment),
                    None if play && !after_goal => {
                        episodes += 1;
                        Some((episodes - 1, index, index))
                    }
                    None => None,
                };
                if let Some(segment) = target {
                    episode[index] = Some(segment.0);
                    close(&mut episode_end, segment, index);
                }
                after_goal = true;
            } else if play {
                if after_goal {
                    continue;
                }
                match &mut open {
                    Some(segment) => segment.2 = index,
                    None => {
                        episodes += 1;
                        open = Some((episodes - 1, index, index));
                    }
                }
                episode[index] = open.map(|segment| segment.0);
            } else {
                if let Some(segment) = open.take() {
                    let end = segment.2;
                    close(&mut episode_end, segment, end);
                }
                after_goal = false;
            }
        }
        if let Some(segment) = open.take() {
            let end = segment.2;
            close(&mut episode_end, segment, end);
        }
        Self {
            episode,
            episode_end,
            next_goal,
            episodes,
        }
    }

    pub fn episodes(&self) -> u32 {
        self.episodes
    }

    /// The labels of frame `index`. `converted` supplies the slots of the car actors and which slots have a
    /// car in the state; `slots` is the width of the per-slot values (the conversion's number of car slots).
    pub fn frame(
        &self,
        observed: &ObservedReplay,
        index: usize,
        converted: &ConvertedFrame,
        slots: usize,
    ) -> FrameLabels {
        let mut present = vec![false; slots];
        for (info, _) in &converted.state.cars {
            if let Some(flag) = present.get_mut(info.idx) {
                *flag = true;
            }
        }
        self.frame_from(&observed.frames, index, &converted.car_actor_slots, &present)
    }

    fn frame_from(
        &self,
        frames: &[Frame],
        index: usize,
        car_actor_slots: &[(i32, usize)],
        present: &[bool],
    ) -> FrameLabels {
        let time = |frame: usize| f64::from(frames[frame].time);
        let until = |end: usize| (time(end) - time(index)) as f32;
        FrameLabels {
            episode: self.episode[index],
            episode_seconds_remaining: self.episode_end[index].map(until),
            next_scoring_team: self.next_goal[index].map(|(_, team)| team),
            seconds_until_next_goal: self.next_goal[index].map(|(goal, _)| until(goal)),
            update_age_seconds: update_ages(frames, index, car_actor_slots, present),
        }
    }
}

/// Per car slot, the age of the slot's car's last rigid-body packet at frame `index`, in seconds of
/// replay time (observed: only that frame and earlier ones are used). `car_actor_slots` maps each car
/// actor of the frame to its slot; the slot's car is the primary linked car of that slot. Null when the
/// slot has no car in the state (`present`), no primary car, or that car has had no packet yet.
fn update_ages(
    frames: &[Frame],
    index: usize,
    car_actor_slots: &[(i32, usize)],
    present: &[bool],
) -> Vec<Option<f32>> {
    let frame = &frames[index];
    slot_primary_cars(frame, car_actor_slots, present)
        .into_iter()
        .map(|car| {
            car?.body
                .position
                .as_ref()
                .filter(|packet| packet.frame <= index)
                .map(|packet| (f64::from(frame.time) - f64::from(frames[packet.frame].time)) as f32)
        })
        .collect()
}

/// The car each slot shows in a frame: the primary linked car whose actor maps to the slot in
/// `car_actor_slots`. `None` for a slot that is not `present` in the state or has no such car.
pub(crate) fn slot_primary_cars<'a>(
    frame: &'a Frame,
    car_actor_slots: &[(i32, usize)],
    present: &[bool],
) -> Vec<Option<&'a Car>> {
    let mut cars = vec![None; present.len()];
    for car in primary_linked_cars(frame) {
        let Some(&(_, slot)) = car_actor_slots.iter().find(|(actor, _)| *actor == car.actor_id) else {
            continue;
        };
        if present.get(slot).copied().unwrap_or(false) {
            cars[slot] = Some(car);
        }
    }
    cars
}

/// The header labels: final score and winner from the replay's last observed scoreboard.
pub fn header_labels(observed: &ObservedReplay) -> HeaderLabels {
    let final_scores = observed.frames.last().map(|frame| &frame.team_scores);
    let mut final_score = [None; 2];
    let mut final_score_sources = [None; 2];
    if let Some(scores) = final_scores {
        for (team, score) in scores.iter().enumerate() {
            if let Some(score) = score {
                final_score[team] = Some(score.value);
                final_score_sources[team] = Some(match score.source {
                    crate::observations::Source::Replay => "replay",
                    crate::observations::Source::InferredMatchStart => "inferred_match_start",
                });
            }
        }
    }
    let winning_team = match final_score {
        [Some(blue), Some(orange)] if blue > orange => Some(0),
        [Some(blue), Some(orange)] if orange > blue => Some(1),
        _ => None,
    };
    let mut observed_goals = [0u32; 2];
    for frame in &observed.frames {
        if let Some(team) = goal_of(frame) {
            observed_goals[usize::from(team)] += 1;
        }
    }
    HeaderLabels {
        future_derived: true,
        final_score,
        final_score_sources,
        winning_team,
        episodes: ReplayLabels::new(observed).episodes(),
        observed_goals,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observations::{Body, Car, Header, Inputs, Source, Value};

    fn value<T>(value: T, frame: usize) -> Option<Value<T>> {
        Some(Value {
            value,
            frame,
            source: Source::Replay,
        })
    }

    /// A frame at 10 frames per second (time = index / 10) in `state`, with `goal` (the team scored on)
    /// reported in it.
    fn frame(index: usize, state: &str, goal: Option<u8>) -> Frame {
        Frame {
            index,
            time: index as f32 / 10.0,
            delta: 0.1,
            ball: None,
            cars: Vec::new(),
            players: Vec::new(),
            team_scores: [None, None],
            seconds_remaining: None,
            overtime: None,
            game_state: value(state.to_string(), index),
            events: goal.map(|team| Event::GoalScoredOn { team }).into_iter().collect(),
            pad_pickups: Vec::new(),
        }
    }

    fn replay(frames: Vec<Frame>) -> ObservedReplay {
        ObservedReplay {
            header: Header {
                game_type: "TAGame.Replay_Soccar_TA".to_string(),
                levels: Vec::new(),
                final_team_scores: [None, None],
            },
            frames,
            diagnostics: Default::default(),
        }
    }

    /// pregame 0-1, countdown 2-3, play 4-9, goal on orange (blue scores) reported at 10, goal pause 10-12,
    /// countdown 13, play 14-18 with the goal on blue (orange scores) reported at 19 (still in the goal
    /// pause), countdown 20, play 21-24 with no goal (the replay ends).
    fn two_goal_replay() -> ObservedReplay {
        let mut frames = Vec::new();
        let mut push = |state: &str, goal: Option<u8>| frames.push(frame(frames.len(), state, goal));
        push("WaitingForPlayers", None);
        push("PreGame", None);
        push("Countdown", None);
        push("Countdown", None);
        for _ in 0..6 {
            push("Active", None);
        }
        push("PostGoalScored", Some(1));
        push("ReplayPlayback", None);
        push("ReplayPlayback", None);
        push("Countdown", None);
        for _ in 0..5 {
            push("Active", None);
        }
        push("PostGoalScored", Some(0));
        push("Countdown", None);
        for _ in 0..4 {
            push("Active", None);
        }
        replay(frames)
    }

    #[test]
    fn episodes_run_from_a_kickoff_through_the_goal_frame() {
        let observed = two_goal_replay();
        let labels = ReplayLabels::new(&observed);
        let episodes: Vec<Option<u32>> = labels.episode.clone();
        let expect = |range: std::ops::Range<usize>, value: Option<u32>| {
            for index in range {
                assert_eq!(episodes[index], value, "frame {index}");
            }
        };
        expect(0..4, None); // pregame and countdown
        expect(4..11, Some(0)); // play and the goal frame
        expect(11..14, None); // goal pause and countdown
        expect(14..20, Some(1));
        expect(20..21, None);
        expect(21..25, Some(2)); // no goal: runs to the last frame
        assert_eq!(labels.episodes(), 3);
        let at = |index: usize| labels.frame_from(&observed.frames, index, &[], &[]);
        // The goal frame has 0 s remaining, the kickoff frame its whole episode.
        assert_eq!(at(10).episode_seconds_remaining, Some(0.0));
        assert!((at(4).episode_seconds_remaining.unwrap() - 0.6).abs() < 1e-6);
        assert_eq!(at(3).episode_seconds_remaining, None);
        assert_eq!(at(12).episode_seconds_remaining, None);
        // Without a goal the episode ends at the replay's last frame.
        assert!((at(21).episode_seconds_remaining.unwrap() - 0.3).abs() < 1e-6);
        assert_eq!(at(24).episode_seconds_remaining, Some(0.0));
    }

    #[test]
    fn the_next_goal_is_looked_up_from_the_goal_events() {
        let observed = two_goal_replay();
        let labels = ReplayLabels::new(&observed);
        let at = |index: usize| labels.frame_from(&observed.frames, index, &[], &[]);
        // Scored on team 1 means team 0 scored; frames before it, and the goal frame itself, look to it.
        for index in 0..=10 {
            assert_eq!(at(index).next_scoring_team, Some(0), "frame {index}");
        }
        assert_eq!(at(10).seconds_until_next_goal, Some(0.0));
        assert!((at(0).seconds_until_next_goal.unwrap() - 1.0).abs() < 1e-6);
        // After it, the goal pause already looks to the next episode's goal (orange scores).
        for index in 11..=19 {
            assert_eq!(at(index).next_scoring_team, Some(1), "frame {index}");
        }
        assert!((at(11).seconds_until_next_goal.unwrap() - 0.8).abs() < 1e-6);
        // No further goal: null, never zero.
        for index in 20..25 {
            let labels = at(index);
            assert_eq!(labels.next_scoring_team, None, "frame {index}");
            assert_eq!(labels.seconds_until_next_goal, None, "frame {index}");
        }
    }

    #[test]
    fn a_tied_regulation_gives_the_overtime_kickoff_its_own_episode() {
        let mut frames = Vec::new();
        let mut push = |state: &str, goal: Option<u8>| frames.push(frame(frames.len(), state, goal));
        for _ in 0..3 {
            push("Active", None); // regulation play to expiry (0-2)
        }
        push("Countdown", None); // overtime countdown
        for _ in 0..3 {
            push("Active", None); // overtime play (4-6)
        }
        push("PostGoalScored", Some(0)); // golden goal (7), the replay ends in the goal pause
        push("ReplayPlayback", None);
        let observed = replay(frames);
        let labels = ReplayLabels::new(&observed);
        assert_eq!(
            labels.episode,
            [Some(0), Some(0), Some(0), None, Some(1), Some(1), Some(1), Some(1), None]
        );
        // The first episode has no goal: it ends at its last in-play frame; frames of it still look to the
        // overtime goal.
        let first = labels.frame_from(&observed.frames, 0, &[], &[]);
        assert!((first.episode_seconds_remaining.unwrap() - 0.2).abs() < 1e-6);
        assert_eq!(first.next_scoring_team, Some(1));
        assert!((first.seconds_until_next_goal.unwrap() - 0.7).abs() < 1e-6);
    }

    #[test]
    fn a_goal_with_no_play_before_it_opens_no_episode() {
        // A replay that starts in a goal pause: the goal belongs to no episode (null), and the in-play
        // frames of the same run after a goal frame are not an episode either.
        let observed = replay(vec![
            frame(0, "PostGoalScored", Some(0)),
            frame(1, "Active", None),
            frame(2, "Countdown", None),
            frame(3, "Active", None),
        ]);
        let labels = ReplayLabels::new(&observed);
        assert_eq!(labels.episode, [None, None, None, Some(0)]);
        assert_eq!(labels.episodes(), 1);
        let first = labels.frame_from(&observed.frames, 0, &[], &[]);
        assert_eq!((first.next_scoring_team, first.seconds_until_next_goal), (Some(1), Some(0.0)));
        // No goal after frame 0: the later frames have no next goal.
        assert_eq!(labels.frame_from(&observed.frames, 3, &[], &[]).next_scoring_team, None);
    }

    #[test]
    fn a_replay_with_no_frame_in_play_has_only_nulls() {
        let observed = replay(vec![frame(0, "Countdown", None), frame(1, "PreGame", None)]);
        let labels = ReplayLabels::new(&observed);
        assert_eq!(labels.episode, [None, None]);
        assert_eq!(labels.episodes(), 0);
        let one = labels.frame_from(&observed.frames, 1, &[], &[]);
        assert_eq!(
            (one.episode, one.episode_seconds_remaining, one.next_scoring_team, one.seconds_until_next_goal),
            (None, None, None, None)
        );
    }

    fn car(actor_id: i32, key: &str, packet_frame: Option<usize>) -> Car {
        Car {
            actor_id,
            actor_created_frame: 0,
            player_key: Some(key.to_string()),
            player_link_active: true,
            team: Some(0),
            body_product_id: None,
            body: Body {
                position: packet_frame.and_then(|frame| value([0.0; 3], frame)),
                ..Body::default()
            },
            boost: None,
            boost_raw: None,
            inputs: Inputs::default(),
            spawn_pose: None,
        }
    }

    #[test]
    fn the_update_age_is_the_time_since_the_cars_last_packet() {
        let mut frames: Vec<Frame> = (0..6).map(|index| frame(index, "Active", None)).collect();
        // Slot 0 (actor 10): packets at frames 1 and 4. Slot 1 (actor 11): none yet at frame 4, one at 5.
        // Slot 2 (actor 12) is not in the state. Actor 13 is a shadowed older car of slot 0.
        frames[4].cars = vec![car(10, "a", Some(4)), car(11, "b", None), car(12, "c", Some(0))];
        frames[3].cars = vec![car(10, "a", Some(1)), car(11, "b", None)];
        frames[5].cars = vec![car(10, "a", Some(4)), car(11, "b", Some(5)), car(13, "a", Some(0))];
        frames[5].cars[2].player_link_active = false;
        let actor_slots = [(10, 0), (11, 1), (12, 2), (13, 0)];
        let present = [true, true, false];
        let labels = ReplayLabels::new(&replay(frames.clone()));
        let at = |index: usize| labels.frame_from(&frames, index, &actor_slots, &present).update_age_seconds;
        let near = |value: Option<f32>, expected: Option<f32>| match (value, expected) {
            (Some(a), Some(b)) => assert!((a - b).abs() < 1e-6, "{a} vs {b}"),
            (a, b) => assert_eq!(a, b),
        };
        // Frame 3: slot 0 last updated at frame 1 (0.2 s ago), slot 1 has no packet yet, slot 2 has no car.
        let ages = at(3);
        near(ages[0], Some(0.2));
        assert_eq!((ages[1], ages[2]), (None, None));
        // Frame 4: a packet in this frame is age 0; the absent slot stays null although its actor has a packet.
        let ages = at(4);
        near(ages[0], Some(0.0));
        assert_eq!((ages[1], ages[2]), (None, None));
        // Frame 5: the unlinked shadow does not replace the slot's primary car.
        let ages = at(5);
        near(ages[0], Some(0.1));
        near(ages[1], Some(0.0));
        // A frame with no cars: every slot null.
        assert_eq!(at(0), vec![None, None, None]);
    }

    #[test]
    fn the_header_labels_read_the_last_observed_scoreboard() {
        let mut frames = vec![frame(0, "Active", None), frame(1, "Active", None), frame(2, "PostGoalScored", Some(1))];
        frames[0].team_scores = [value(9, 0), value(9, 0)]; // earlier scores are not the final ones
        frames[2].team_scores = [value(2, 2), value(1, 0)];
        let labels = header_labels(&replay(frames.clone()));
        assert!(labels.future_derived);
        assert_eq!(labels.final_score, [Some(2), Some(1)]);
        assert_eq!(labels.final_score_sources, [Some("replay"), Some("replay")]);
        assert_eq!(labels.winning_team, Some(0));
        assert_eq!((labels.episodes, labels.observed_goals), (1, [1, 0]));
        // Orange ahead, a draw, and an unknown score.
        frames[2].team_scores = [value(1, 2), value(3, 2)];
        assert_eq!(header_labels(&replay(frames.clone())).winning_team, Some(1));
        frames[2].team_scores = [value(3, 2), value(3, 2)];
        assert_eq!(header_labels(&replay(frames.clone())).winning_team, None);
        frames[2].team_scores = [value(3, 2), None];
        let unknown = header_labels(&replay(frames.clone()));
        assert_eq!((unknown.final_score, unknown.winning_team), ([Some(3), None], None));
        frames[2].team_scores = [
            value(1, 2),
            Some(Value {
                value: 0,
                frame: 0,
                source: Source::InferredMatchStart,
            }),
        ];
        let shutout = header_labels(&replay(frames));
        assert_eq!(shutout.final_score_sources, [Some("replay"), Some("inferred_match_start")]);
        assert_eq!(shutout.winning_team, Some(0));
        assert_eq!(header_labels(&replay(Vec::new())).final_score, [None, None]);
    }
}
