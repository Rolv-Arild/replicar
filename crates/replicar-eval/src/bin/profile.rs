//! Where a default conversion's time goes: decode, update ticks, contact alignment, and the simulation with the
//! time inside each inference question (a wrapper around the fitted inference), per replay and in total.
//!
//! usage: `profile <replay or folder>... [--final-assessment]`

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::{Duration, Instant};

use replicar::decode::{CarLife, NetworkCar};
use replicar::infer::{
    AirSchedule, AirScheduleQuery, DodgePlan, FitQuery, GroundChoice, Inference, PressInFlight,
};
use replicar::rocketsim::CarControls;
use replicar_eval::collect_replays;

/// Times every question of the inner inference.
struct Timed<'i> {
    inner: &'i mut dyn Inference,
    times: RefCell<BTreeMap<&'static str, (Duration, usize)>>,
}

impl Timed<'_> {
    fn time<T>(&self, name: &'static str, f: impl FnOnce() -> T) -> T {
        let start = Instant::now();
        let out = f();
        let mut times = self.times.borrow_mut();
        let entry = times.entry(name).or_default();
        entry.0 += start.elapsed();
        entry.1 += 1;
        out
    }
}

impl Inference for Timed<'_> {
    fn air_controls(
        &mut self,
        index: usize,
        car: &NetworkCar,
        controls: &CarControls,
    ) -> Option<replicar::air::AirControls> {
        let start = Instant::now();
        let out = self.inner.air_controls(index, car, controls);
        let mut times = self.times.borrow_mut();
        let e = times.entry("air_controls").or_default();
        e.0 += start.elapsed();
        e.1 += 1;
        out
    }
    fn flip_pitch(&mut self, query: &FitQuery, airborne: bool, pressing: bool) -> Option<f32> {
        let start = Instant::now();
        let out = self.inner.flip_pitch(query, airborne, pressing);
        let mut times = self.times.borrow_mut();
        let e = times.entry("flip_pitch").or_default();
        e.0 += start.elapsed();
        e.1 += 1;
        out
    }
    fn dodge_start(&mut self, query: &FitQuery) -> Option<DodgePlan> {
        let start = Instant::now();
        let out = self.inner.dodge_start(query);
        let mut times = self.times.borrow_mut();
        let e = times.entry("dodge_start").or_default();
        e.0 += start.elapsed();
        e.1 += 1;
        out
    }
    fn air_schedule(
        &mut self,
        query: &AirScheduleQuery,
        press: Option<PressInFlight>,
    ) -> Option<(AirSchedule, i32)> {
        let start = Instant::now();
        let out = self.inner.air_schedule(query, press);
        let mut times = self.times.borrow_mut();
        let e = times.entry("air_schedule").or_default();
        e.0 += start.elapsed();
        e.1 += 1;
        out
    }
    fn ground_schedule(&mut self, query: &FitQuery, dodge_pending: bool) -> Option<GroundChoice> {
        let start = Instant::now();
        let out = self.inner.ground_schedule(query, dodge_pending);
        let mut times = self.times.borrow_mut();
        let e = times.entry("ground_schedule").or_default();
        e.0 += start.elapsed();
        e.1 += 1;
        out
    }
    fn ticks_override(&self, life: CarLife, index: usize) -> Option<u64> {
        self.time("ticks_override", || self.inner.ticks_override(life, index))
    }
    fn dodge_handled(&self, life: CarLife, index: usize) -> bool {
        self.time("dodge_handled", || self.inner.dodge_handled(life, index))
    }
    fn control_shift(&self, life: CarLife) -> Option<i64> {
        self.time("control_shift", || self.inner.control_shift(life))
    }
}

fn main() -> ExitCode {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let final_assessment = args.iter().any(|a| a == "--final-assessment");
    args.retain(|a| a != "--final-assessment");
    let roots: Vec<PathBuf> = args.iter().map(PathBuf::from).collect();
    let Ok(replays) = collect_replays(&roots, final_assessment) else {
        eprintln!("error: cannot list the replays");
        return ExitCode::FAILURE;
    };
    let Ok(meshes) = replicar::Meshes::load("collision_meshes") else {
        eprintln!("error: no meshes");
        return ExitCode::FAILURE;
    };
    let mut totals: BTreeMap<String, Duration> = BTreeMap::new();
    let mut calls: BTreeMap<String, usize> = BTreeMap::new();
    for path in &replays {
        let mut stage = BTreeMap::new();
        let bytes = std::fs::read(path).expect("readable replay");
        let start = Instant::now();
        let network =
            replicar::decode::decode(&replicar::parse(&bytes).expect("parse")).expect("decode");
        stage.insert("1 decode".to_owned(), start.elapsed());
        let start = Instant::now();
        let withheld = replicar::update_ticks::Withheld::default();
        let ticks = replicar::update_ticks::infer(&network, true, withheld);
        stage.insert("2 update ticks".to_owned(), start.elapsed());
        let start = Instant::now();
        let options = replicar::infer::InferenceOptions::default();
        let simulation_options = replicar::simulate::SimulationOptions::default();
        let ticks = replicar::align::align_contacts(
            &network,
            &ticks,
            &meshes,
            options,
            &simulation_options,
        )
        .expect("align")
        .0;
        stage.insert("3 contact alignment".to_owned(), start.elapsed());
        let mut fitted =
            replicar::infer::FittedInference::new(&network, Some(&ticks), options, withheld);
        let mut timed = Timed {
            inner: &mut fitted,
            times: RefCell::new(BTreeMap::new()),
        };
        let mut annotator = replicar::annotate::Annotator::new(replicar::annotate::ball_intervals(
            &network.frames,
            &ticks,
            withheld,
        ));
        let start = Instant::now();
        let mut annotate = Duration::ZERO;
        replicar::simulate::simulate(
            &network,
            Some(&ticks),
            &mut timed,
            &meshes,
            simulation_options,
            |frame| {
                let s = Instant::now();
                let _ = annotator.annotate(&frame);
                annotate += s.elapsed();
            },
        )
        .expect("simulate");
        let total = start.elapsed();
        let questions: Duration = timed.times.borrow().values().map(|t| t.0).sum();
        stage.insert(
            "4 simulation (rest)".to_owned(),
            total - questions - annotate,
        );
        stage.insert(
            "5 annotation (with ball intervals' rollouts at construction excluded)".to_owned(),
            annotate,
        );
        for (name, (time, count)) in timed.times.borrow().iter() {
            stage.insert(format!("6 inference: {name}"), *time);
            *calls.entry(format!("6 inference: {name}")).or_default() += count;
        }
        let sum: Duration = stage.values().sum();
        println!("{:.2} s  {}", sum.as_secs_f64(), path.display());
        for (name, time) in stage {
            *totals.entry(name).or_default() += time;
        }
    }
    let sum: Duration = totals.values().sum();
    println!(
        "total {:.1} s over {} replays",
        sum.as_secs_f64(),
        replays.len()
    );
    for (name, time) in &totals {
        println!(
            "  {:5.1}%  {:7.2} s  {:>9} calls  {name}",
            100.0 * time.as_secs_f64() / sum.as_secs_f64(),
            time.as_secs_f64(),
            calls.get(name).map_or(String::new(), |c| c.to_string())
        );
    }
    ExitCode::SUCCESS
}
