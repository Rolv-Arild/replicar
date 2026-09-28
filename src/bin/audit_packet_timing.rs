//! Compare raw replay update gaps with motion-derived intervals and an independent rotation check.

use std::collections::{BTreeMap, HashMap};
use std::env;
use std::error::Error;
use std::fs::{self, File};
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};

use glam::{Quat, Vec3};
use replay_to_rocketsim::conversion::{OfflineIntervalEstimate, estimate_car_packet_interval};
use replay_to_rocketsim::observations::{self, Body, Car};
use serde::Serialize;

fn replay_paths(root: &Path) -> Result<Vec<PathBuf>, Box<dyn Error>> {
    if root.is_file() {
        return Ok(vec![root.to_owned()]);
    }
    let mut paths = Vec::new();
    for size in ["1v1", "2v2", "3v3"] {
        for entry in fs::read_dir(root.join(size))? {
            let path = entry?.path();
            if path.extension().is_some_and(|ext| ext == "replay") {
                paths.push(path);
            }
        }
    }
    paths.sort();
    Ok(paths)
}

#[derive(Clone, Copy)]
struct Sample {
    frame: usize,
    time: f32,
    pos: Vec3,
    vel: Option<Vec3>,
    rot: Option<Quat>,
    ang_vel: Option<Vec3>,
    dodge_active: bool,
}

fn fresh_sample(body: &Body, frame: usize, time: f32, dodge_active: bool) -> Option<Sample> {
    let pos = Vec3::from_array(body.position.as_ref().filter(|v| v.frame == frame)?.value);
    if !pos.is_finite() {
        return None;
    }
    let vel = body
        .linear_velocity
        .as_ref()
        .filter(|v| v.frame == frame)
        .map(|v| Vec3::from_array(v.value))
        .filter(|v| v.is_finite());
    let rot = body
        .rotation_xyzw
        .as_ref()
        .filter(|v| v.frame == frame)
        .map(|v| Quat::from_xyzw(v.value[0], v.value[1], v.value[2], v.value[3]))
        .filter(|q| q.is_finite() && q.length_squared() > 0.5)
        .map(Quat::normalize);
    let ang_vel = body
        .angular_velocity_replay_units
        .as_ref()
        .filter(|v| v.frame == frame)
        .map(|v| Vec3::from_array(v.value) * 0.01)
        .filter(|v| v.is_finite());
    Some(Sample {
        frame,
        time,
        pos,
        vel,
        rot,
        ang_vel,
        dodge_active,
    })
}

fn car_sample(car: &Car, frame: usize, time: f32) -> Option<Sample> {
    let dodge_active = car
        .inputs
        .dodge_active_raw
        .as_ref()
        .is_some_and(|v| v.frame == frame && v.value % 2 == 1);
    fresh_sample(&car.body, frame, time, dodge_active)
}

fn projected_interval(prior: Sample, next: Sample) -> Option<OfflineIntervalEstimate> {
    let (Some(v0), Some(v1)) = (prior.vel, next.vel) else {
        return None;
    };
    estimate_car_packet_interval(
        prior.pos.to_array(),
        next.pos.to_array(),
        v0.to_array(),
        v1.to_array(),
        next.time - prior.time,
    )
}

fn quantiles(mut values: Vec<f32>) -> Option<Quantiles> {
    values.retain(|v| v.is_finite());
    if values.is_empty() {
        return None;
    }
    values.sort_by(f32::total_cmp);
    let get = |p: f64| values[((values.len() - 1) as f64 * p).round() as usize];
    Some(Quantiles {
        count: values.len(),
        p50: get(0.5),
        p90: get(0.9),
        p99: get(0.99),
    })
}

#[derive(Serialize)]
struct Quantiles {
    count: usize,
    p50: f32,
    p90: f32,
    p99: f32,
}

#[derive(Default)]
struct PairStats {
    update_gap_frames: BTreeMap<usize, usize>,
    elapsed_seconds: Vec<f32>,
    projected_scale: Vec<f32>,
    rounded_tick_histogram: BTreeMap<u32, usize>,
    nominal_rotation_error_degrees: Vec<f32>,
    half_rotation_error_degrees: Vec<f32>,
    projected_rotation_error_degrees: Vec<f32>,
    nominal_position_on_past_scale_pairs_uu: Vec<f32>,
    past_scale_position_error_uu: Vec<f32>,
    nominal_rotation_on_past_scale_pairs_degrees: Vec<f32>,
    past_scale_rotation_error_degrees: Vec<f32>,
    nominal_position_on_gap_model_pairs_uu: Vec<f32>,
    gap_model_position_error_uu: Vec<f32>,
    nominal_rotation_on_gap_model_pairs_degrees: Vec<f32>,
    gap_model_rotation_error_degrees: Vec<f32>,
}

#[derive(Serialize)]
struct PairSummary {
    update_gap_frames: BTreeMap<usize, usize>,
    elapsed_seconds: Option<Quantiles>,
    projected_scale: Option<Quantiles>,
    rounded_tick_histogram: BTreeMap<u32, usize>,
    nominal_rotation_error_degrees: Option<Quantiles>,
    half_rotation_error_degrees: Option<Quantiles>,
    projected_rotation_error_degrees: Option<Quantiles>,
    nominal_position_on_past_scale_pairs_uu: Option<Quantiles>,
    past_scale_position_error_uu: Option<Quantiles>,
    nominal_rotation_on_past_scale_pairs_degrees: Option<Quantiles>,
    past_scale_rotation_error_degrees: Option<Quantiles>,
    nominal_position_on_gap_model_pairs_uu: Option<Quantiles>,
    gap_model_position_error_uu: Option<Quantiles>,
    nominal_rotation_on_gap_model_pairs_degrees: Option<Quantiles>,
    gap_model_rotation_error_degrees: Option<Quantiles>,
}

impl PairStats {
    fn add(
        &mut self,
        prior: Sample,
        next: Sample,
        compare_rotation: bool,
        past_scale: Option<f32>,
        gap_model_scale: Option<f32>,
    ) {
        let gap = next.frame - prior.frame;
        if gap == 0 {
            return;
        }
        *self.update_gap_frames.entry(gap).or_default() += 1;
        let dt = next.time - prior.time;
        if !dt.is_finite() || dt <= 0.0 || dt > 0.5 {
            return;
        }
        self.elapsed_seconds.push(dt);
        if compare_rotation {
            if let (Some(v0), Some(scale)) = (
                prior.vel,
                gap_model_scale.filter(|s| (0.25..=2.5).contains(s)),
            ) {
                self.nominal_position_on_gap_model_pairs_uu
                    .push((prior.pos + v0 * dt - next.pos).length());
                self.gap_model_position_error_uu
                    .push((prior.pos + v0 * dt * scale - next.pos).length());
                if let (Some(q0), Some(q1), Some(w0)) = (prior.rot, next.rot, prior.ang_vel) {
                    if prior.pos.z > 100.0
                        && next.pos.z > 100.0
                        && !prior.dodge_active
                        && !next.dodge_active
                    {
                        let error = |duration: f32| {
                            let predicted = Quat::from_scaled_axis(w0 * duration) * q0;
                            2.0 * predicted.dot(q1).abs().clamp(0.0, 1.0).acos().to_degrees()
                        };
                        self.nominal_rotation_on_gap_model_pairs_degrees
                            .push(error(dt));
                        self.gap_model_rotation_error_degrees
                            .push(error(dt * scale));
                    }
                }
            }
            if let (Some(v0), Some(scale)) =
                (prior.vel, past_scale.filter(|s| (0.25..=2.5).contains(s)))
            {
                self.nominal_position_on_past_scale_pairs_uu
                    .push((prior.pos + v0 * dt - next.pos).length());
                self.past_scale_position_error_uu
                    .push((prior.pos + v0 * dt * scale - next.pos).length());
                if let (Some(q0), Some(q1), Some(w0)) = (prior.rot, next.rot, prior.ang_vel) {
                    if prior.pos.z > 100.0
                        && next.pos.z > 100.0
                        && !prior.dodge_active
                        && !next.dodge_active
                    {
                        let error = |duration: f32| {
                            let predicted = Quat::from_scaled_axis(w0 * duration) * q0;
                            2.0 * predicted.dot(q1).abs().clamp(0.0, 1.0).acos().to_degrees()
                        };
                        self.nominal_rotation_on_past_scale_pairs_degrees
                            .push(error(dt));
                        self.past_scale_rotation_error_degrees
                            .push(error(dt * scale));
                    }
                }
            }
        }
        let Some(interval) = projected_interval(prior, next) else {
            return;
        };
        self.projected_scale.push(interval.scale);
        *self
            .rounded_tick_histogram
            .entry(interval.effective_ticks)
            .or_default() += 1;

        if !compare_rotation
            || prior.pos.z <= 100.0
            || next.pos.z <= 100.0
            || prior.dodge_active
            || next.dodge_active
        {
            return;
        }
        let (Some(q0), Some(q1), Some(w0), Some(w1)) =
            (prior.rot, next.rot, prior.ang_vel, next.ang_vel)
        else {
            return;
        };
        let mean_w = (w0 + w1) * 0.5;
        let angle_error = |seconds: f32| {
            let predicted = Quat::from_scaled_axis(mean_w * seconds) * q0;
            2.0 * predicted.dot(q1).abs().clamp(0.0, 1.0).acos().to_degrees()
        };
        self.nominal_rotation_error_degrees.push(angle_error(dt));
        self.half_rotation_error_degrees.push(angle_error(dt * 0.5));
        self.projected_rotation_error_degrees
            .push(angle_error(interval.effective_seconds));
    }

    fn summary(self) -> PairSummary {
        PairSummary {
            update_gap_frames: self.update_gap_frames,
            elapsed_seconds: quantiles(self.elapsed_seconds),
            projected_scale: quantiles(self.projected_scale),
            rounded_tick_histogram: self.rounded_tick_histogram,
            nominal_rotation_error_degrees: quantiles(self.nominal_rotation_error_degrees),
            half_rotation_error_degrees: quantiles(self.half_rotation_error_degrees),
            projected_rotation_error_degrees: quantiles(self.projected_rotation_error_degrees),
            nominal_position_on_past_scale_pairs_uu: quantiles(
                self.nominal_position_on_past_scale_pairs_uu,
            ),
            past_scale_position_error_uu: quantiles(self.past_scale_position_error_uu),
            nominal_rotation_on_past_scale_pairs_degrees: quantiles(
                self.nominal_rotation_on_past_scale_pairs_degrees,
            ),
            past_scale_rotation_error_degrees: quantiles(self.past_scale_rotation_error_degrees),
            nominal_position_on_gap_model_pairs_uu: quantiles(
                self.nominal_position_on_gap_model_pairs_uu,
            ),
            gap_model_position_error_uu: quantiles(self.gap_model_position_error_uu),
            nominal_rotation_on_gap_model_pairs_degrees: quantiles(
                self.nominal_rotation_on_gap_model_pairs_degrees,
            ),
            gap_model_rotation_error_degrees: quantiles(self.gap_model_rotation_error_degrees),
        }
    }
}

#[derive(Default)]
struct Stats {
    car: BTreeMap<String, PairStats>,
    ball: BTreeMap<String, PairStats>,
}

impl Stats {
    fn add_car(
        &mut self,
        prior: Sample,
        next: Sample,
        past_scale: Option<f32>,
        gap_model_scale: Option<f32>,
    ) {
        self.car.entry("all".to_owned()).or_default().add(
            prior,
            next,
            true,
            past_scale,
            gap_model_scale,
        );
        let gap = next.frame - prior.frame;
        let label = if gap <= 3 {
            gap.to_string()
        } else {
            "4+".to_owned()
        };
        self.car
            .entry(label)
            .or_default()
            .add(prior, next, true, past_scale, gap_model_scale);
        let contact = if prior.pos.z < 50.0 && next.pos.z < 50.0 {
            "ground"
        } else if prior.pos.z > 100.0 && next.pos.z > 100.0 {
            "air"
        } else {
            "transition"
        };
        self.car.entry(contact.to_owned()).or_default().add(
            prior,
            next,
            true,
            past_scale,
            gap_model_scale,
        );
    }

    fn add_ball(&mut self, prior: Sample, next: Sample) {
        self.ball
            .entry("all".to_owned())
            .or_default()
            .add(prior, next, false, None, None);
        let gap = next.frame - prior.frame;
        let label = if gap <= 3 {
            gap.to_string()
        } else {
            "4+".to_owned()
        };
        self.ball
            .entry(label)
            .or_default()
            .add(prior, next, false, None, None);
    }

    fn summary(self) -> StatsSummary {
        StatsSummary {
            car: self
                .car
                .into_iter()
                .map(|(k, v)| (k, v.summary()))
                .collect(),
            ball: self
                .ball
                .into_iter()
                .map(|(k, v)| (k, v.summary()))
                .collect(),
        }
    }
}

#[derive(Serialize)]
struct StatsSummary {
    car: BTreeMap<String, PairSummary>,
    ball: BTreeMap<String, PairSummary>,
}

#[derive(Serialize)]
struct Report {
    split: String,
    replay_count: usize,
    method: &'static str,
    gap_model_source: Option<String>,
    by_game_size: BTreeMap<String, StatsSummary>,
    by_replay: BTreeMap<String, StatsSummary>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args_os().skip(1);
    let root =
        PathBuf::from(args.next().ok_or(
            "usage: audit_packet_timing <replay_or_split> [report.json] [train_report.json]",
        )?);
    let output = args.next().map(PathBuf::from);
    let model_path = args.next().map(PathBuf::from);
    if args.next().is_some() {
        return Err(
            "usage: audit_packet_timing <replay_or_split> [report.json] [train_report.json]".into(),
        );
    }
    let gap_model: Option<serde_json::Value> = if let Some(path) = &model_path {
        Some(serde_json::from_slice(&fs::read(path)?)?)
    } else {
        None
    };
    let paths = replay_paths(&root)?;
    let mut by_size: BTreeMap<String, Stats> = BTreeMap::new();
    let mut by_replay = BTreeMap::new();
    for path in &paths {
        let size = path
            .parent()
            .and_then(|p| p.file_name())
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_else(|| "single".to_owned());
        let parsed = replay_to_rocketsim::parse_replay(&fs::read(path)?)?;
        let replay = observations::extract(&parsed).ok_or("network frames unavailable")?;
        let stats = by_size.entry(size.clone()).or_default();
        let mut replay_stats = Stats::default();
        let mut prior_cars: HashMap<(i32, usize), (Sample, Option<f32>)> = HashMap::new();
        let mut prior_ball: Option<Sample> = None;
        for frame in &replay.frames {
            if !frame
                .game_state
                .as_ref()
                .is_some_and(|state| state.value == "Active")
            {
                prior_cars.clear();
                prior_ball = None;
                continue;
            }
            if let Some(next) = frame
                .ball
                .as_ref()
                .and_then(|body| fresh_sample(body, frame.index, frame.time, false))
            {
                if let Some(prior) = prior_ball.replace(next) {
                    stats.add_ball(prior, next);
                    replay_stats.add_ball(prior, next);
                }
            }
            for car in observations::primary_linked_cars(frame) {
                let Some(next) = car_sample(car, frame.index, frame.time) else {
                    continue;
                };
                let key = (car.actor_id, car.actor_created_frame);
                if let Some(&(prior, past_scale)) = prior_cars.get(&key) {
                    let gap = next.frame - prior.frame;
                    let gap_model_scale = gap_model.as_ref().and_then(|model| {
                        (gap <= 3).then(|| gap.to_string()).and_then(|gap_label| {
                            model["by_game_size"][&size]["car"][&gap_label]
                                    ["projected_scale"]["p50"]
                                    .as_f64()
                                    .map(|v| v as f32)
                        })
                    });
                    stats.add_car(prior, next, past_scale, gap_model_scale);
                    replay_stats.add_car(prior, next, past_scale, gap_model_scale);
                    prior_cars.insert(
                        key,
                        (next, projected_interval(prior, next).map(|v| v.scale)),
                    );
                } else {
                    prior_cars.insert(key, (next, None));
                }
            }
        }
        let replay_key = if root.is_file() {
            path.file_name().unwrap().to_string_lossy().into_owned()
        } else {
            path.strip_prefix(&root)?.display().to_string()
        };
        by_replay.insert(replay_key, replay_stats.summary());
    }
    let report = Report {
        split: root.display().to_string(),
        replay_count: paths.len(),
        method: "raw gaps count fresh position packets from the same active actor lifetime; motion projection uses fresh position and velocity at both endpoints; rounded ticks are inferred, not packet timestamps; rotation check uses target orientation only for scoring",
        gap_model_source: model_path.map(|path| path.display().to_string()),
        by_game_size: by_size
            .into_iter()
            .map(|(size, stats)| (size, stats.summary()))
            .collect(),
        by_replay,
    };
    match output {
        Some(path) => serde_json::to_writer_pretty(BufWriter::new(File::create(path)?), &report)?,
        None => serde_json::to_writer_pretty(io::stdout().lock(), &report)?,
    }
    io::stdout().flush()?;
    Ok(())
}
