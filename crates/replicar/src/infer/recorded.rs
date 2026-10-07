//! Recording an inference's answers and answering from the recording (docs/v2-plan.md, section 4.2, "One trait
//! with two implementations"). The simulator asks the same questions in the same order whenever it gets the
//! same answers, so a run answered from its own recording is the same run, without fitting.
//!
//! An answer is keyed by the car life it is about, the question, and how many times that question was asked
//! about that car life before. Only answers that change something are kept: a question whose answer is "no
//! choice" (`None`, `false`) is absent from the recording and answered so again.

use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap};

use rocketsim::CarControls;

use super::{
    AirSchedule, AirScheduleQuery, DodgePlan, FitQuery, GroundChoice, Inference, PressInFlight,
};
use crate::air::AirControls;
use crate::decode::{CarLife, NetworkCar};

/// The simulator's questions (the methods of `Inference`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Question {
    AirControls,
    FlipPitch,
    DodgeStart,
    AirSchedule,
    GroundSchedule,
    TicksOverride,
    DodgeHandled,
    ControlShift,
}

/// An answer that changes something.
#[derive(Debug, Clone, PartialEq)]
pub enum Choice {
    AirControls(AirControls),
    FlipPitch(f32),
    DodgeStart(DodgePlan),
    /// The schedule and the shift of the flip's start it chose.
    AirSchedule(AirSchedule, i32),
    GroundSchedule(GroundChoice),
    TicksOverride(u64),
    DodgeHandled,
    ControlShift(i64),
}

/// What identifies an answer: the car life, the question, and how many times it was asked before about
/// that car life.
pub type Key = (CarLife, Question, u32);

/// The answers of one run, with the replay frame each was given in.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Recording {
    pub choices: BTreeMap<Key, (u32, Choice)>,
    /// How many questions were asked, by question.
    pub asked: BTreeMap<Question, usize>,
}

/// Counts the questions per car life.
#[derive(Debug, Default)]
struct Counter(RefCell<HashMap<(CarLife, Question), u32>>);

impl Counter {
    /// The key of the next question `question` about `life`.
    fn next(&self, life: CarLife, question: Question) -> Key {
        let mut counts = self.0.borrow_mut();
        let count = counts.entry((life, question)).or_default();
        let key = (life, question, *count);
        *count += 1;
        key
    }
}

/// Answers with `inner` and records the answers.
pub struct Recorder<'i> {
    inner: &'i mut dyn Inference,
    counter: Counter,
    /// The frame of the latest question that names one (`control_shift` does not).
    frame: std::cell::Cell<u32>,
    recording: RefCell<Recording>,
}

impl<'i> Recorder<'i> {
    pub fn new(inner: &'i mut dyn Inference) -> Self {
        Self {
            inner,
            counter: Counter::default(),
            frame: std::cell::Cell::new(0),
            recording: RefCell::new(Recording::default()),
        }
    }

    #[must_use]
    pub fn into_recording(self) -> Recording {
        self.recording.into_inner()
    }

    /// Records the answer to the next question `question` about `life`, asked in frame `frame` (`None`: the
    /// latest frame named); `None` is not kept.
    fn record<T>(
        &self,
        life: CarLife,
        question: Question,
        frame: Option<usize>,
        answer: T,
        choice: impl FnOnce(&T) -> Option<Choice>,
    ) -> T {
        if let Some(frame) = frame {
            self.frame.set(u32::try_from(frame).unwrap_or(u32::MAX));
        }
        let key = self.counter.next(life, question);
        let mut recording = self.recording.borrow_mut();
        *recording.asked.entry(question).or_default() += 1;
        if let Some(choice) = choice(&answer) {
            recording.choices.insert(key, (self.frame.get(), choice));
        }
        answer
    }
}

impl Inference for Recorder<'_> {
    fn air_controls(
        &mut self,
        index: usize,
        car: &NetworkCar,
        controls: &CarControls,
    ) -> Option<AirControls> {
        let answer = self.inner.air_controls(index, car, controls);
        self.record(car.life, Question::AirControls, Some(index), answer, |a| {
            a.map(Choice::AirControls)
        })
    }

    fn flip_pitch(&mut self, query: &FitQuery, airborne: bool, pressing: bool) -> Option<f32> {
        let answer = self.inner.flip_pitch(query, airborne, pressing);
        self.record(
            query.car.life,
            Question::FlipPitch,
            Some(query.index),
            answer,
            |a| a.map(Choice::FlipPitch),
        )
    }

    fn dodge_start(&mut self, query: &FitQuery) -> Option<DodgePlan> {
        let answer = self.inner.dodge_start(query);
        self.record(
            query.car.life,
            Question::DodgeStart,
            Some(query.index),
            answer,
            |a| a.map(Choice::DodgeStart),
        )
    }

    fn air_schedule(
        &mut self,
        query: &AirScheduleQuery,
        press: Option<PressInFlight>,
    ) -> Option<(AirSchedule, i32)> {
        let answer = self.inner.air_schedule(query, press);
        self.record(
            query.car.life,
            Question::AirSchedule,
            Some(query.index),
            answer,
            |a| {
                a.clone()
                    .map(|(schedule, shift)| Choice::AirSchedule(schedule, shift))
            },
        )
    }

    fn ground_schedule(&mut self, query: &FitQuery, dodge_pending: bool) -> Option<GroundChoice> {
        let answer = self.inner.ground_schedule(query, dodge_pending);
        self.record(
            query.car.life,
            Question::GroundSchedule,
            Some(query.index),
            answer,
            |a| a.clone().map(Choice::GroundSchedule),
        )
    }

    fn ticks_override(&self, life: CarLife, index: usize) -> Option<u64> {
        let answer = self.inner.ticks_override(life, index);
        self.record(life, Question::TicksOverride, Some(index), answer, |a| {
            a.map(Choice::TicksOverride)
        })
    }

    fn dodge_handled(&self, life: CarLife, index: usize) -> bool {
        let answer = self.inner.dodge_handled(life, index);
        self.record(life, Question::DodgeHandled, Some(index), answer, |a| {
            a.then_some(Choice::DodgeHandled)
        })
    }

    fn control_shift(&self, life: CarLife) -> Option<i64> {
        let answer = self.inner.control_shift(life);
        self.record(life, Question::ControlShift, None, answer, |a| {
            a.map(Choice::ControlShift)
        })
    }
}

/// Answers from a recording.
pub struct RecordedInference<'r> {
    recording: &'r Recording,
    counter: Counter,
}

impl<'r> RecordedInference<'r> {
    #[must_use]
    pub fn new(recording: &'r Recording) -> Self {
        Self {
            recording,
            counter: Counter::default(),
        }
    }

    fn answer(&self, life: CarLife, question: Question) -> Option<&'r Choice> {
        self.recording
            .choices
            .get(&self.counter.next(life, question))
            .map(|(_, choice)| choice)
    }
}

impl Inference for RecordedInference<'_> {
    fn air_controls(
        &mut self,
        _index: usize,
        car: &NetworkCar,
        _controls: &CarControls,
    ) -> Option<AirControls> {
        match self.answer(car.life, Question::AirControls) {
            Some(Choice::AirControls(controls)) => Some(*controls),
            _ => None,
        }
    }

    fn flip_pitch(&mut self, query: &FitQuery, _airborne: bool, _pressing: bool) -> Option<f32> {
        match self.answer(query.car.life, Question::FlipPitch) {
            Some(Choice::FlipPitch(pitch)) => Some(*pitch),
            _ => None,
        }
    }

    fn dodge_start(&mut self, query: &FitQuery) -> Option<DodgePlan> {
        match self.answer(query.car.life, Question::DodgeStart) {
            Some(Choice::DodgeStart(plan)) => Some(*plan),
            _ => None,
        }
    }

    fn air_schedule(
        &mut self,
        query: &AirScheduleQuery,
        _press: Option<PressInFlight>,
    ) -> Option<(AirSchedule, i32)> {
        match self.answer(query.car.life, Question::AirSchedule) {
            Some(Choice::AirSchedule(schedule, shift)) => Some((schedule.clone(), *shift)),
            _ => None,
        }
    }

    fn ground_schedule(&mut self, query: &FitQuery, _dodge_pending: bool) -> Option<GroundChoice> {
        match self.answer(query.car.life, Question::GroundSchedule) {
            Some(Choice::GroundSchedule(choice)) => Some(choice.clone()),
            _ => None,
        }
    }

    fn ticks_override(&self, life: CarLife, _index: usize) -> Option<u64> {
        match self.answer(life, Question::TicksOverride) {
            Some(Choice::TicksOverride(ticks)) => Some(*ticks),
            _ => None,
        }
    }

    fn dodge_handled(&self, life: CarLife, _index: usize) -> bool {
        matches!(
            self.answer(life, Question::DodgeHandled),
            Some(Choice::DodgeHandled)
        )
    }

    fn control_shift(&self, life: CarLife) -> Option<i64> {
        match self.answer(life, Question::ControlShift) {
            Some(Choice::ControlShift(shift)) => Some(*shift),
            _ => None,
        }
    }
}
