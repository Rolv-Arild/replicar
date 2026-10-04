//! Per-frame packet freshness: which bodies got a fresh rigid-body packet in a frame and how old the last
//! one is. Observed or inferred values, not future-derived labels, and outputs only: nothing here feeds the
//! conversion or the state.
//!
//! * `ball_fresh` and `car_fresh`: the converter applied a fresh rigid-body packet at this frame (the body's
//!   position was updated in the frame: `ConvertedFrame::ball_fresh` and `fresh_car_slots`). `car_fresh` is
//!   null for a slot with no car in the frame's state and for a slot whose primary car cannot be resolved
//!   in the frame (between actor lifetimes), where it is unknown. It is a SUPERSET of the `packet_lags`
//!   rows: it also counts fresh packets in frames the converter does not simulate (goal pause, countdown,
//!   the first kickoff frame), which have no lag record.
//! * `ball_update_age_seconds` and `car_update_age_seconds` (per slot): frame time minus the time of the
//!   frame that carried the body's last packet (frame cadence, not packet time; 0 at a fresh frame); null
//!   before the first packet, for a slot with no resolved car, and for a respawned car until its first
//!   packet.
//! * `ball_packet_age_ticks` and `car_packet_age_ticks`: the frame's timeline tick minus the inferred server
//!   tick of the last applied packet, where that tick is the timeline tick of the packet's frame minus the
//!   packet's lag, carried forward (normally 0 to 4 at a fresh frame, larger when the frame gap exceeds 4
//!   ticks: the train maximum is 9 for cars and 5 for the ball; it grows between packets). This USES THE OFFLINE
//!   LAG INFERENCE (`packet_lags`): it is null before the first packet, when no lag was inferred for the
//!   packet (inference off, a frame the converter does not simulate, or the `default` source, which is only
//!   half the frame window and not an inference), and for a car from its respawn (a new actor) until its
//!   first packet. The lag source of a packet is in the `packet_lags` table.

use serde::Serialize;

use crate::conversion::{AppliedPacketLag, ConvertedFrame};
use crate::labels::slot_primary_cars;
use crate::observations::{Frame, ObservedReplay};

/// The freshness of one replay frame (`freshness` in a JSONL frame; typed columns in Parquet).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FrameFreshness {
    /// A fresh ball packet was applied at this frame.
    pub ball_fresh: bool,
    /// Per car slot: a fresh packet of the slot's primary car was applied at this frame; null for a slot with
    /// no car in this frame's state or whose primary car cannot be resolved in it (unknown).
    pub car_fresh: Vec<Option<bool>>,
    /// Frame time minus the time of the frame with the ball's last packet; null before the first.
    pub ball_update_age_seconds: Option<f32>,
    /// Per car slot, frame time minus the time of the frame with the slot's car's last packet; null when
    /// unknown (see the module docs).
    pub car_update_age_seconds: Vec<Option<f32>>,
    /// Offline-inferred age of the ball's last packet in server ticks (see the module docs); null when unknown.
    pub ball_packet_age_ticks: Option<u32>,
    /// Per car slot, the same for the slot's car; null when unknown or the slot has no car.
    pub car_packet_age_ticks: Vec<Option<u32>>,
}

/// The inferred server tick of an applied packet: the frame's timeline tick minus the recorded lag, or
/// `None` without a lag record or when it is the `default` (uninferred) one.
fn server_tick(timeline_tick: u64, lag: Option<&AppliedPacketLag>) -> Option<i64> {
    let lag = lag.filter(|lag| lag.source != "default")?;
    Some(timeline_tick as i64 - lag.ticks as i64)
}

/// Carries the last packet's server tick forward from frame to frame; use one per conversion, in frame order.
pub struct FreshnessTracker {
    /// The ball's last applied packet: its inferred server tick (`None`: no lag known), or no packet yet.
    ball: Option<Option<i64>>,
    /// Per slot: the actor lifetime (actor id, creation frame) of the last packet and its inferred server tick.
    cars: Vec<Option<((i32, usize), Option<i64>)>>,
}

impl FreshnessTracker {
    pub fn new(slots: usize) -> Self {
        Self {
            ball: None,
            cars: vec![None; slots],
        }
    }

    pub fn frame(
        &mut self,
        observed: &ObservedReplay,
        index: usize,
        converted: &ConvertedFrame,
    ) -> FrameFreshness {
        let mut present = vec![false; self.cars.len()];
        for (info, _) in &converted.state.cars {
            if let Some(flag) = present.get_mut(info.idx) {
                *flag = true;
            }
        }
        self.step(
            &observed.frames,
            index,
            converted.timeline_tick,
            converted.ball_fresh,
            &converted.fresh_car_slots,
            &converted.packet_lags,
            &converted.car_actor_slots,
            &present,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn step(
        &mut self,
        frames: &[Frame],
        index: usize,
        timeline_tick: u64,
        ball_fresh: bool,
        fresh_car_slots: &[usize],
        packet_lags: &[AppliedPacketLag],
        car_actor_slots: &[(i32, usize)],
        present: &[bool],
    ) -> FrameFreshness {
        let frame = &frames[index];
        let age =
            |server: Option<i64>| server.map(|tick| (timeline_tick as i64 - tick).max(0) as u32);
        if ball_fresh {
            let lag = packet_lags.iter().find(|lag| lag.actor_id.is_none());
            self.ball = Some(server_tick(timeline_tick, lag));
        }
        let ball_update_age_seconds = frame
            .ball
            .as_ref()
            .and_then(|body| body.position.as_ref())
            .filter(|packet| packet.frame <= index)
            .map(|packet| (f64::from(frame.time) - f64::from(frames[packet.frame].time)) as f32);
        let slot_cars = slot_primary_cars(frame, car_actor_slots, present);
        let mut car_fresh = Vec::with_capacity(present.len());
        let mut car_update_age_seconds = Vec::with_capacity(present.len());
        let mut car_packet_age_ticks = Vec::with_capacity(present.len());
        for (slot, car) in slot_cars.iter().enumerate() {
            let Some(car) = car else {
                // No car, or a car that cannot be resolved (between actor lifetimes): unknown, not stale.
                car_fresh.push(None);
                car_update_age_seconds.push(None);
                car_packet_age_ticks.push(None);
                continue;
            };
            let lifetime = (car.actor_id, car.actor_created_frame);
            // A car of another lifetime (a respawn) does not inherit the previous car's packet.
            if self.cars[slot].is_some_and(|(last, _)| last != lifetime) {
                self.cars[slot] = None;
            }
            let fresh = fresh_car_slots.contains(&slot);
            if fresh {
                let lag = packet_lags
                    .iter()
                    .find(|lag| lag.actor_id == Some(car.actor_id));
                self.cars[slot] = Some((lifetime, server_tick(timeline_tick, lag)));
            }
            car_fresh.push(Some(fresh));
            car_update_age_seconds.push(
                car.body
                    .position
                    .as_ref()
                    .filter(|packet| packet.frame <= index)
                    .map(|packet| {
                        (f64::from(frame.time) - f64::from(frames[packet.frame].time)) as f32
                    }),
            );
            car_packet_age_ticks.push(age(self.cars[slot].and_then(|(_, tick)| tick)));
        }
        FrameFreshness {
            ball_fresh,
            car_fresh,
            ball_update_age_seconds,
            car_update_age_seconds,
            ball_packet_age_ticks: age(self.ball.flatten()),
            car_packet_age_ticks,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observations::{Body, Car, Inputs, Source, Value};

    fn value<T>(value: T, frame: usize) -> Option<Value<T>> {
        Some(Value {
            value,
            frame,
            source: Source::Replay,
        })
    }

    fn car(actor_id: i32, created: usize, key: &str, packet_frame: Option<usize>) -> Car {
        Car {
            actor_id,
            actor_created_frame: created,
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

    fn frame(index: usize, ball_packet: Option<usize>, cars: Vec<Car>) -> Frame {
        Frame {
            index,
            time: index as f32 / 30.0,
            delta: 1.0 / 30.0,
            ball: ball_packet.map(|packet| Body {
                position: value([0.0; 3], packet),
                ..Body::default()
            }),
            cars,
            players: Vec::new(),
            team_scores: [None, None],
            seconds_remaining: None,
            overtime: None,
            game_state: None,
            events: Vec::new(),
            pad_pickups: Vec::new(),
        }
    }

    fn lag(actor_id: Option<i32>, ticks: u64, source: &'static str) -> AppliedPacketLag {
        AppliedPacketLag {
            actor_id,
            ticks,
            source,
        }
    }

    // Four ticks per frame, a car slot 0 (actor 10) and a ball. Frames 0 to 5 at timeline ticks 0, 4, ..., 20.
    fn ticks(index: usize) -> u64 {
        index as u64 * 4
    }

    #[test]
    fn ages_in_ticks_carry_the_inferred_server_tick_forward() {
        let actor_slots = [(10, 0), (11, 1)];
        let present = [true, true];
        let mut tracker = FreshnessTracker::new(2);
        let mut frames = Vec::new();
        // Ball packets at frames 1 and 4; car 10 packets at frames 1 and 3; car 11 (slot 1) never has one.
        for index in 0..6 {
            let ball = [None, Some(1), Some(1), Some(1), Some(4), Some(4)][index];
            let car10 = [None, Some(1), Some(1), Some(3), Some(3), Some(3)][index];
            frames.push(frame(
                index,
                ball,
                vec![car(10, 0, "a", car10), car(11, 0, "b", None)],
            ));
        }
        let mut seen = Vec::new();
        for index in 0..6 {
            let ball_fresh = matches!(index, 1 | 4);
            let fresh: &[usize] = if index == 1 || index == 3 { &[0] } else { &[] };
            let mut lags = Vec::new();
            if ball_fresh {
                lags.push(lag(None, if index == 1 { 3 } else { 1 }, "chain"));
            }
            if fresh.contains(&0) {
                lags.push(lag(Some(10), if index == 1 { 2 } else { 0 }, "chain"));
            }
            seen.push(tracker.step(
                &frames,
                index,
                ticks(index),
                ball_fresh,
                fresh,
                &lags,
                &actor_slots,
                &present,
            ));
        }
        // Before the first packet everything is null; a body that never updates stays null (not 0).
        assert_eq!(seen[0].ball_packet_age_ticks, None);
        assert_eq!(seen[0].car_packet_age_ticks, [None, None]);
        assert_eq!(seen[0].ball_update_age_seconds, None);
        // Frame 1: ball lag 3 and car lag 2 give ages 3 and 2 (inside 0..=4); the age grows by 4 per frame.
        assert_eq!(seen[1].ball_packet_age_ticks, Some(3));
        assert_eq!(seen[1].car_packet_age_ticks[0], Some(2));
        assert_eq!(seen[2].ball_packet_age_ticks, Some(7));
        assert_eq!(seen[2].car_packet_age_ticks[0], Some(6));
        // Frame 3: the car's new packet resets its age to its lag; the ball's keeps growing.
        assert_eq!(seen[3].car_packet_age_ticks[0], Some(0));
        assert_eq!(seen[3].ball_packet_age_ticks, Some(11));
        assert_eq!(seen[4].ball_packet_age_ticks, Some(1));
        assert_eq!(seen[5].ball_packet_age_ticks, Some(5));
        assert_eq!(seen[5].car_packet_age_ticks[0], Some(8));
        assert_eq!(
            seen.iter()
                .map(|f| f.car_packet_age_ticks[1])
                .collect::<Vec<_>>(),
            vec![None; 6]
        );
        // Masks: fresh exactly where a packet was applied; every slot with a car has a value.
        assert_eq!(
            seen.iter().map(|f| f.ball_fresh).collect::<Vec<_>>(),
            [false, true, false, false, true, false]
        );
        assert_eq!(
            seen.iter().map(|f| f.car_fresh[0]).collect::<Vec<_>>(),
            [
                Some(false),
                Some(true),
                Some(false),
                Some(true),
                Some(false),
                Some(false)
            ]
        );
        assert!(seen.iter().all(|f| f.car_fresh[1] == Some(false)));
        // The car's frame-time age: packets at frames 1 and 3, so 0 at them and one frame (1/30 s) per frame
        // after; slot 1 has no packet and stays null.
        let car_age: Vec<Option<f32>> = seen.iter().map(|f| f.car_update_age_seconds[0]).collect();
        assert_eq!(&car_age[..2], [None, Some(0.0)]);
        assert!((car_age[2].unwrap() - 1.0 / 30.0).abs() < 1e-6 && car_age[3] == Some(0.0));
        assert!((car_age[5].unwrap() - 2.0 / 30.0).abs() < 1e-6);
        assert!(seen.iter().all(|f| f.car_update_age_seconds[1].is_none()));
        // The frame-time age of the ball: one frame (1/30 s) after a packet in the previous frame, 0 at one.
        assert_eq!(seen[1].ball_update_age_seconds, Some(0.0));
        assert!((seen[2].ball_update_age_seconds.unwrap() - 1.0 / 30.0).abs() < 1e-6);
        assert!((seen[3].ball_update_age_seconds.unwrap() - 2.0 / 30.0).abs() < 1e-6);
    }

    #[test]
    fn a_packet_without_an_inferred_lag_leaves_the_tick_age_null() {
        let actor_slots = [(10, 0)];
        let present = [true];
        let frames: Vec<Frame> = (0..4)
            .map(|index| frame(index, Some(index), vec![car(10, 0, "a", Some(index))]))
            .collect();
        let mut tracker = FreshnessTracker::new(1);
        let mut at = |index: usize, lags: &[AppliedPacketLag]| {
            tracker.step(
                &frames,
                index,
                ticks(index),
                true,
                &[0],
                lags,
                &actor_slots,
                &present,
            )
        };
        // No lag record at all (inference off, or a frame that is not simulated).
        let first = at(0, &[]);
        assert!(first.ball_fresh && first.car_fresh == [Some(true)]);
        assert_eq!(
            (
                first.ball_packet_age_ticks,
                first.car_packet_age_ticks.clone()
            ),
            (None, vec![None])
        );
        // The `default` source is half the frame window, not an inferred lag.
        let second = at(1, &[lag(None, 2, "default"), lag(Some(10), 2, "default")]);
        assert_eq!(
            (
                second.ball_packet_age_ticks,
                second.car_packet_age_ticks.clone()
            ),
            (None, vec![None])
        );
        // A fitted or chained lag is one; a lag of 0 is a value.
        let third = at(2, &[lag(None, 0, "chain"), lag(Some(10), 3, "dodge_fit")]);
        assert_eq!(
            (
                third.ball_packet_age_ticks,
                third.car_packet_age_ticks.clone()
            ),
            (Some(0), vec![Some(3)])
        );
        // A later fresh packet without a lag does not keep the previous packet's age.
        let fourth = at(3, &[]);
        assert_eq!(
            (fourth.ball_packet_age_ticks, fourth.car_packet_age_ticks),
            (None, vec![None])
        );
    }

    #[test]
    fn a_respawned_car_does_not_inherit_the_previous_cars_packet_and_an_absent_slot_is_null() {
        let actor_slots = [(10, 0), (20, 0)];
        let mut tracker = FreshnessTracker::new(2);
        let frames = vec![
            frame(0, None, vec![car(10, 0, "a", Some(0))]),
            // The player's new car (actor 20) exists and has no packet yet.
            frame(1, None, vec![car(20, 1, "a", None)]),
            frame(2, None, vec![car(20, 1, "a", Some(2))]),
        ];
        let present = [true, false]; // slot 1 has no car in the state
        let first = tracker.step(
            &frames,
            0,
            0,
            false,
            &[0],
            &[lag(Some(10), 1, "chain")],
            &actor_slots,
            &present,
        );
        assert_eq!(first.car_packet_age_ticks, [Some(1), None]);
        // Absent slot: car_fresh and the age are null, not false or 0.
        assert_eq!(first.car_fresh, [Some(true), None]);
        assert_eq!(first.car_update_age_seconds[1], None);
        let second = tracker.step(&frames, 1, 4, false, &[], &[], &actor_slots, &present);
        assert_eq!(second.car_packet_age_ticks, [None, None]);
        assert_eq!(second.car_fresh, [Some(false), None]);
        let third = tracker.step(
            &frames,
            2,
            8,
            false,
            &[0],
            &[lag(Some(20), 2, "frame_median")],
            &actor_slots,
            &present,
        );
        assert_eq!(third.car_packet_age_ticks, [Some(2), None]);
    }

    #[test]
    fn a_slot_whose_primary_car_is_unresolved_is_unknown_not_stale() {
        // Slot 0 is in the state but the frame has no car that maps to it (between actor lifetimes).
        let frames = vec![
            frame(0, None, vec![car(10, 0, "a", Some(0))]),
            frame(1, None, Vec::new()),
        ];
        let mut tracker = FreshnessTracker::new(1);
        let present = [true];
        let first = tracker.step(
            &frames,
            0,
            0,
            false,
            &[0],
            &[lag(Some(10), 1, "chain")],
            &[(10, 0)],
            &present,
        );
        assert_eq!(first.car_fresh, [Some(true)]);
        let second = tracker.step(&frames, 1, 4, false, &[], &[], &[], &present);
        assert_eq!(second.car_fresh, [None]);
        assert_eq!(
            (
                second.car_update_age_seconds.clone(),
                second.car_packet_age_ticks
            ),
            (vec![None], vec![None])
        );
    }
}
