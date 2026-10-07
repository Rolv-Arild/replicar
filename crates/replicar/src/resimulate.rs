//! The `resimulation` group to and from what the simulator needs: the update ticks it applies and the
//! recorded inference (docs/glossary.md, "Resimulate").

use std::collections::BTreeMap;

use replicar_format::FrameIndex;
use replicar_format::resimulation::{
    CarTicks, Choice as Stored, Dodge, FrameTicks, Resimulation, ScheduleEntry,
};

use crate::air::AirControls;
use crate::decode::{ActorId, CarLife};
use crate::infer::recorded::{Choice, Question, Recording};
use crate::infer::{AirSchedule, DodgePlan, GroundChoice, GroundSchedule};
use crate::update_ticks::UpdateTicks;
use crate::{Error, PlayerIndex};

const QUESTIONS: [(Question, &str); 8] = [
    (Question::AirControls, "air_controls"),
    (Question::FlipPitch, "flip_pitch"),
    (Question::DodgeStart, "dodge_start"),
    (Question::AirSchedule, "air_schedule"),
    (Question::GroundSchedule, "ground_schedule"),
    (Question::TicksOverride, "ticks_override"),
    (Question::DodgeHandled, "dodge_handled"),
    (Question::ControlShift, "control_shift"),
];

fn question_name(question: Question) -> &'static str {
    QUESTIONS
        .iter()
        .find(|(q, _)| *q == question)
        .map_or("", |(_, name)| name)
}

fn question_of(name: &str) -> Option<Question> {
    QUESTIONS.iter().find(|(_, n)| *n == name).map(|(q, _)| *q)
}

fn dodge(plan: &DodgePlan) -> Dodge {
    Dodge {
        activation_frame: plan.activation_frame as u32,
        start_offset: plan.start_offset,
        duration: plan.duration,
        pitch: plan.pitch,
        yaw: plan.yaw,
        cancel: plan.cancel,
        first_update: plan.first_update.map(|(f, t)| (f as u32, t)),
    }
}

fn plan(dodge: &Dodge) -> DodgePlan {
    DodgePlan {
        activation_frame: dodge.activation_frame as usize,
        start_offset: dodge.start_offset,
        duration: dodge.duration,
        pitch: dodge.pitch,
        yaw: dodge.yaw,
        cancel: dodge.cancel,
        first_update: dodge.first_update.map(|(f, t)| (f as usize, t)),
    }
}

/// The group of a run: its update ticks (`None`: every update at its frame's tick) and its recording.
#[must_use]
pub fn to_group(ticks: Option<&UpdateTicks>, recording: &Recording) -> Resimulation {
    let mut group = Resimulation::default();
    if let Some(ticks) = ticks {
        for (frame, (ball, median)) in ticks.ball.iter().zip(&ticks.car_median).enumerate() {
            if ball.is_some() || median.is_some() {
                group.ticks.push(FrameTicks {
                    frame: frame as u32,
                    ball: *ball,
                    car_median: *median,
                });
            }
        }
        group.car_ticks = ticks
            .cars
            .iter()
            .map(|(&(life, frame), &ticks)| CarTicks {
                frame: frame.0,
                actor: life.actor.0,
                created: life.created.0,
                ticks,
            })
            .collect();
    }
    let mut previous: Option<(crate::infer::recorded::Key, &Choice)> = None;
    for (&key, (frame, choice)) in &recording.choices {
        // The same answer to the next asking of the same question about the same car life extends the run.
        if let Some(((life, question, ordinal), last)) = previous
            && let Some(stored) = group.choices.last_mut()
            && (life, question) == (key.0, key.1)
            && key.2 == ordinal + 1
            && last == choice
        {
            stored.repeat += 1;
            previous = Some((key, choice));
            continue;
        }
        previous = Some((key, choice));
        let (life, question, ordinal) = key;
        group.choices.push({
            let mut stored = Stored {
                frame: *frame,
                actor: life.actor.0,
                created: life.created.0,
                question: question_name(question).to_owned(),
                ordinal,
                ..Stored::default()
            };
            match choice {
                Choice::AirControls(c) => {
                    stored.values = [Some(c.pitch), Some(c.yaw), Some(c.roll)]
                }
                Choice::FlipPitch(p) => stored.values[0] = Some(*p),
                Choice::DodgeStart(p) => stored.dodge = Some(dodge(p)),
                Choice::AirSchedule(schedule, shift) => {
                    stored.player = Some(schedule.player.0);
                    stored.end_tick = Some(schedule.end_tick);
                    stored.integer = Some(i64::from(*shift));
                    stored.entries = schedule
                        .entries
                        .iter()
                        .map(|(tick, c)| ScheduleEntry {
                            tick: *tick,
                            pitch: Some(c.pitch),
                            yaw: Some(c.yaw),
                            roll: Some(c.roll),
                            ..ScheduleEntry::default()
                        })
                        .collect();
                }
                Choice::GroundSchedule(choice) => {
                    stored.end_tick = Some(choice.schedule.end_tick);
                    stored.integer = choice.schedule.shift;
                    stored.dodge = choice.dodge.as_ref().map(dodge);
                    stored.entries = choice
                        .schedule
                        .entries
                        .iter()
                        .map(
                            |&(tick, throttle, steer, handbrake, boost, jump)| ScheduleEntry {
                                tick,
                                throttle: Some(throttle),
                                steer: Some(steer),
                                handbrake: Some(handbrake),
                                boost: Some(boost),
                                jump,
                                ..ScheduleEntry::default()
                            },
                        )
                        .collect();
                }
                Choice::TicksOverride(t) => stored.integer = Some(*t as i64),
                Choice::DodgeHandled => {}
                Choice::ControlShift(s) => stored.integer = Some(*s),
            }
            stored
        });
    }
    group
}

fn invalid(what: &str) -> Error {
    Error::Resimulation(format!("the resimulation group has an invalid {what}"))
}

/// The update ticks and recording of a group, for `frames` frames. The ticks are `None` when the group has
/// none and `with_ticks` is false (the run applied every update at its frame's tick).
pub fn from_group(
    group: &Resimulation,
    frames: usize,
    with_ticks: bool,
) -> Result<(Option<UpdateTicks>, Recording), Error> {
    let ticks = with_ticks.then(|| {
        let mut ticks = UpdateTicks {
            ball: vec![None; frames],
            car_median: vec![None; frames],
            ..UpdateTicks::default()
        };
        for t in &group.ticks {
            if let Some(f) = ticks.ball.get_mut(t.frame as usize) {
                *f = t.ball;
                ticks.car_median[t.frame as usize] = t.car_median;
            }
        }
        ticks.cars = group
            .car_ticks
            .iter()
            .map(|t| {
                let life = CarLife {
                    actor: ActorId(t.actor),
                    created: FrameIndex(t.created),
                };
                ((life, FrameIndex(t.frame)), t.ticks)
            })
            .collect();
        ticks
    });
    let mut choices = BTreeMap::new();
    for stored in &group.choices {
        let question = question_of(&stored.question).ok_or_else(|| invalid("question"))?;
        let life = CarLife {
            actor: ActorId(stored.actor),
            created: FrameIndex(stored.created),
        };
        let value = |k: usize| stored.values[k].ok_or_else(|| invalid("value"));
        let integer = || stored.integer.ok_or_else(|| invalid("integer"));
        let choice = match question {
            Question::AirControls => Choice::AirControls(AirControls {
                pitch: value(0)?,
                yaw: value(1)?,
                roll: value(2)?,
            }),
            Question::FlipPitch => Choice::FlipPitch(value(0)?),
            Question::DodgeStart => {
                Choice::DodgeStart(plan(stored.dodge.as_ref().ok_or_else(|| invalid("dodge"))?))
            }
            Question::AirSchedule => Choice::AirSchedule(
                AirSchedule {
                    player: PlayerIndex(stored.player.ok_or_else(|| invalid("player"))?),
                    end_tick: stored.end_tick.ok_or_else(|| invalid("end tick"))?,
                    entries: stored
                        .entries
                        .iter()
                        .map(|e| {
                            Ok((
                                e.tick,
                                AirControls {
                                    pitch: e.pitch.ok_or_else(|| invalid("pitch"))?,
                                    yaw: e.yaw.ok_or_else(|| invalid("yaw"))?,
                                    roll: e.roll.ok_or_else(|| invalid("roll"))?,
                                },
                            ))
                        })
                        .collect::<Result<_, Error>>()?,
                },
                i32::try_from(integer()?).map_err(|_| invalid("shift"))?,
            ),
            Question::GroundSchedule => Choice::GroundSchedule(GroundChoice {
                schedule: GroundSchedule {
                    end_tick: stored.end_tick.ok_or_else(|| invalid("end tick"))?,
                    entries: stored
                        .entries
                        .iter()
                        .map(|e| {
                            Ok((
                                e.tick,
                                e.throttle.ok_or_else(|| invalid("throttle"))?,
                                e.steer.ok_or_else(|| invalid("steer"))?,
                                e.handbrake.ok_or_else(|| invalid("handbrake"))?,
                                e.boost.ok_or_else(|| invalid("boost"))?,
                                e.jump,
                            ))
                        })
                        .collect::<Result<_, Error>>()?,
                    shift: stored.integer,
                },
                dodge: stored.dodge.as_ref().map(plan),
            }),
            Question::TicksOverride => {
                Choice::TicksOverride(u64::try_from(integer()?).map_err(|_| invalid("ticks"))?)
            }
            Question::DodgeHandled => Choice::DodgeHandled,
            Question::ControlShift => Choice::ControlShift(integer()?),
        };
        for ordinal in stored.ordinal..=stored.ordinal.saturating_add(stored.repeat) {
            choices.insert((life, question, ordinal), (stored.frame, choice.clone()));
        }
    }
    Ok((
        ticks,
        Recording {
            choices,
            asked: BTreeMap::new(),
        },
    ))
}
