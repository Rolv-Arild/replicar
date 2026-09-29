//! Diagnose one-step angular errors in the 50–100 UU car altitude band.
//! All contact and distance labels are simulator evidence, not replay truth.

use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::conversion::{
    ConversionOutput, ConvertOptions, PositionResidual, convert_bytes,
};
use rocketsim::ArenaEvent;
use serde::Serialize;

#[derive(Default)]
struct Paired {
    sim: Vec<f32>,
    hold: Vec<f32>,
    regret: Vec<f32>,
}

impl Paired {
    fn add(&mut self, sim: f32, hold: f32) {
        if sim.is_finite() && hold.is_finite() {
            self.sim.push(sim);
            self.hold.push(hold);
            self.regret.push(sim - hold);
        }
    }

    fn summary(&self) -> PairSummary {
        PairSummary {
            sim: quantiles(&self.sim),
            hold: quantiles(&self.hold),
            regret: quantiles(&self.regret),
            sim_worse_count: self.regret.iter().filter(|&&v| v > 0.0).count(),
        }
    }
}

#[derive(Serialize)]
struct Quantiles {
    count: usize,
    p50: Option<f32>,
    p90: Option<f32>,
    p99: Option<f32>,
}

fn quantiles(values: &[f32]) -> Quantiles {
    let mut sorted = values.to_vec();
    sorted.sort_by(f32::total_cmp);
    let at = |p: f64| {
        (!sorted.is_empty()).then(|| sorted[((sorted.len() - 1) as f64 * p).round() as usize])
    };
    Quantiles {
        count: sorted.len(),
        p50: at(0.5),
        p90: at(0.9),
        p99: at(0.99),
    }
}

#[derive(Serialize)]
struct PairSummary {
    sim: Quantiles,
    hold: Quantiles,
    regret: Quantiles,
    sim_worse_count: usize,
}

#[derive(Serialize)]
struct Outlier {
    path: String,
    frame: usize,
    time: f32,
    actor_id: i32,
    player_key: Option<String>,
    altitude_z: f32,
    origin_z: f32,
    origin_age_seconds: f32,
    angular_packet_gap_frames: usize,
    angular_packet_gap_seconds: f32,
    labels: Vec<String>,
    sim_error: f32,
    hold_error: f32,
    regret: f32,
}

#[derive(Serialize)]
struct Report {
    schema_version: u32,
    split_directory: String,
    metric: &'static str,
    boxcars_version: &'static str,
    rocketsim_revision: &'static str,
    options: ConvertOptions,
    succeeded: usize,
    failures: Vec<String>,
    replay_sha256: BTreeMap<String, String>,
    all: PairSummary,
    by_game_size: BTreeMap<String, PairSummary>,
    by_replay: BTreeMap<String, PairSummary>,
    by_context: BTreeMap<String, PairSummary>,
    worst_regret: Vec<Outlier>,
}

fn paths(root: &Path) -> Result<Vec<(String, PathBuf)>, Box<dyn Error>> {
    let mut result = Vec::new();
    for size in ["1v1", "2v2", "3v3"] {
        for entry in fs::read_dir(root.join(size))? {
            let path = entry?.path();
            if path.extension().is_some_and(|ext| ext == "replay") {
                result.push((size.to_owned(), path));
            }
        }
    }
    result.sort_by(|a, b| a.1.cmp(&b.1));
    Ok(result)
}

/// Bins are replay frame counts, not inferred simulator ticks.
fn frame_gap_bin(gap: usize) -> &'static str {
    match gap {
        1 => "1",
        2 => "2",
        3 => "3",
        _ => "4_plus",
    }
}

fn context(
    conversion: &ConversionOutput,
    residual: &PositionResidual,
    path: &Path,
) -> Option<Outlier> {
    let actor_id = residual.actor_id?;
    let sim_error = residual.simulated_angular_velocity_error_rad_per_sec?;
    let hold_error = residual.hold_angular_velocity_error_rad_per_sec?;
    let altitude_z = residual.altitude_z?;
    if !(50.0..=100.0).contains(&altitude_z) || !sim_error.is_finite() || !hold_error.is_finite() {
        return None;
    }
    let frame = conversion.observations.frames.get(residual.frame)?;
    let car = frame.cars.iter().find(|c| c.actor_id == actor_id)?;
    let previous_frame = residual.frame.checked_sub(1)?;
    let previous = conversion.observations.frames.get(previous_frame)?;
    let previous_car = previous
        .cars
        .iter()
        .find(|c| c.actor_id == actor_id && c.actor_created_frame == car.actor_created_frame)?;
    let previous_angular = previous_car.body.angular_velocity_replay_units.as_ref()?;
    let target_angular = car.body.angular_velocity_replay_units.as_ref()?;
    if target_angular.frame != residual.frame || previous_angular.frame >= residual.frame {
        return None;
    }
    let angular_packet_gap_frames = residual.frame - previous_angular.frame;
    let angular_packet_gap_seconds = frame.time
        - conversion
            .observations
            .frames
            .get(previous_angular.frame)?
            .time;
    let origin = previous_car.body.position.as_ref()?;
    let origin_z = origin.value[2];
    let origin_age_seconds = previous.time - conversion.observations.frames.get(origin.frame)?.time;
    let slot = car
        .player_key
        .as_ref()
        .and_then(|key| conversion.car_slots.iter().find(|s| &s.player_key == key))
        .map(|s| s.slot);
    let sim_frame = conversion.frames.get(previous_frame)?;
    let sim_car = slot.and_then(|slot| {
        sim_frame
            .state
            .cars
            .iter()
            .find(|(info, _)| info.idx == slot)
            .map(|(_, state)| state)
    });
    let mut labels = vec![
        format!(
            "packet_gap_frames:{}",
            frame_gap_bin(angular_packet_gap_frames)
        ),
        format!(
            "packet_gap_seconds:{}",
            if angular_packet_gap_seconds <= 0.05 {
                "le_0.05"
            } else if angular_packet_gap_seconds <= 0.1 {
                "0.05_to_0.1"
            } else {
                "gt_0.1"
            }
        ),
        format!(
            "origin:{}",
            if origin_z < 50.0 {
                "below_50"
            } else if origin_z > 100.0 {
                "above_100"
            } else {
                "50_to_100"
            }
        ),
        format!(
            "origin_freshness:{}",
            if origin_age_seconds <= 0.05 {
                "le_0.05"
            } else if origin_age_seconds <= 0.15 {
                "0.05_to_0.15"
            } else {
                "gt_0.15"
            }
        ),
        format!(
            "predicted_ground:{}",
            residual.is_on_ground.unwrap_or(false)
        ),
    ];
    if let Some(state) = sim_car {
        labels.push(format!("previous_sim_ground:{}", state.is_on_ground));
        labels.push(format!(
            "previous_sim_wheel_contact:{}",
            state.wheels_with_contact.iter().any(|v| v.is_some())
        ));
        labels.push(format!(
            "previous_sim_world_contact:{}",
            state.world_contact_normal.is_some()
        ));
        let ball_distance = state.phys.pos.distance(sim_frame.state.ball.phys.pos);
        labels.push(format!(
            "previous_sim_ball_distance:{}",
            if ball_distance < 350.0 {
                "lt_350"
            } else {
                "ge_350"
            }
        ));
        let near_car = sim_frame.state.cars.iter().any(|(info, other)| {
            Some(info.idx) != slot && state.phys.pos.distance(other.phys.pos) < 300.0
        });
        labels.push(format!("previous_sim_other_car_lt_300:{}", near_car));
        let near_pad = sim_frame
            .state
            .boost_pads
            .iter()
            .any(|(config, _)| state.phys.pos.distance(config.pos) < 250.0);
        labels.push(format!("previous_sim_pad_lt_250:{}", near_pad));
    } else {
        labels.push("previous_sim_car:missing".to_owned());
    }
    let mut recent_jump = false;
    let mut recent_replay_pad = false;
    let mut recent_sim_world = false;
    let mut recent_sim_ball = false;
    let mut recent_sim_car = false;
    let mut recent_sim_pad = false;
    for index in (0..=residual.frame).rev() {
        if index < car.actor_created_frame {
            break;
        }
        let observed = &conversion.observations.frames[index];
        if frame.time - observed.time > 0.15 {
            break;
        }
        if let Some(c) = observed
            .cars
            .iter()
            .find(|c| c.actor_id == actor_id && c.actor_created_frame == car.actor_created_frame)
        {
            let odd = |v: &Option<replay_to_rocketsim::observations::Value<u8>>| {
                v.as_ref()
                    .is_some_and(|v| v.frame == index && v.value % 2 == 1)
            };
            recent_jump |= odd(&c.inputs.jump_active_raw)
                || odd(&c.inputs.double_jump_active_raw)
                || odd(&c.inputs.dodge_active_raw);
            recent_replay_pad |= observed.pad_pickups.iter().any(|p| {
                p.instigator_car_id == Some(actor_id) && p.picked_up != 255 && p.picked_up % 2 == 1
            });
        }
        if let Some(slot) = slot {
            for event in &conversion.frames[index].simulated_events {
                match event.event {
                    ArenaEvent::CarHitWorld(v) if v.car_idx == slot => recent_sim_world = true,
                    ArenaEvent::CarHitBall(v) if v.car_idx == slot => recent_sim_ball = true,
                    ArenaEvent::CarHitCar(v)
                        if v.bumper_car_idx == slot || v.victim_car_idx == slot =>
                    {
                        recent_sim_car = true
                    }
                    ArenaEvent::CarPickupBoost(v) if v.car_idx == slot => recent_sim_pad = true,
                    _ => (),
                }
            }
        }
    }
    labels.extend([
        format!("recent_replay_jump_or_dodge_packet:{}", recent_jump),
        format!("recent_replay_pad_pickup:{}", recent_replay_pad),
        format!("recent_sim_world_hit:{}", recent_sim_world),
        format!("recent_sim_ball_hit:{}", recent_sim_ball),
        format!("recent_sim_car_hit:{}", recent_sim_car),
        format!("recent_sim_pad_pickup:{}", recent_sim_pad),
    ]);
    if (50.0..=100.0).contains(&origin_z) && residual.is_on_ground == Some(false) && !recent_jump {
        labels.push("persistent_low_air:true".to_owned());
    } else {
        labels.push("persistent_low_air:false".to_owned());
    }
    Some(Outlier {
        path: path.display().to_string(),
        frame: residual.frame,
        time: frame.time,
        actor_id,
        player_key: car.player_key.clone(),
        altitude_z,
        origin_z,
        origin_age_seconds,
        angular_packet_gap_frames,
        angular_packet_gap_seconds,
        labels,
        sim_error,
        hold_error,
        regret: sim_error - hold_error,
    })
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args_os().skip(1);
    let root = PathBuf::from(
        args.next()
            .ok_or("usage: diagnose_low_air <split_dir> <report.json> [collision_meshes]")?,
    );
    let output = PathBuf::from(
        args.next()
            .ok_or("usage: diagnose_low_air <split_dir> <report.json> [collision_meshes]")?,
    );
    let mut options = ConvertOptions::default();
    if let Some(meshes) = args.next() {
        options.collision_meshes = PathBuf::from(meshes);
    }
    if args.next().is_some() {
        return Err("too many arguments".into());
    }
    let replay_paths = paths(&root)?;
    let mut all = Paired::default();
    let mut by_game_size: BTreeMap<String, Paired> = BTreeMap::new();
    let mut by_replay: BTreeMap<String, Paired> = BTreeMap::new();
    let mut by_context: BTreeMap<String, Paired> = BTreeMap::new();
    let mut worst_regret: Vec<Outlier> = Vec::new();
    let mut failures = Vec::new();
    let mut replay_sha256 = BTreeMap::new();
    let mut succeeded = 0;
    for (n, (size, path)) in replay_paths.iter().enumerate() {
        match fs::read(path)
            .map_err(|e| e.to_string())
            .and_then(|b| convert_bytes(&b, &options).map_err(|e| e.to_string()))
        {
            Ok(conversion) => {
                succeeded += 1;
                replay_sha256.insert(
                    path.display().to_string(),
                    conversion.source_sha256.clone().unwrap_or_default(),
                );
                for residual in &conversion.position_residuals {
                    if let Some(sample) = context(&conversion, residual, path) {
                        all.add(sample.sim_error, sample.hold_error);
                        by_game_size
                            .entry(size.clone())
                            .or_default()
                            .add(sample.sim_error, sample.hold_error);
                        by_replay
                            .entry(path.display().to_string())
                            .or_default()
                            .add(sample.sim_error, sample.hold_error);
                        for label in &sample.labels {
                            by_context
                                .entry(label.clone())
                                .or_default()
                                .add(sample.sim_error, sample.hold_error);
                        }
                        worst_regret.push(sample);
                        worst_regret.sort_by(|a, b| b.regret.total_cmp(&a.regret));
                        worst_regret.truncate(100);
                    }
                }
            }
            Err(error) => failures.push(format!("{}: {error}", path.display())),
        }
        eprintln!("{}/{} {}", n + 1, replay_paths.len(), path.display());
    }
    let report = Report {
        schema_version: 1,
        split_directory: root.display().to_string(),
        metric: "fresh one-step pre-correction angular velocity error (rad/s) at target altitude 50–100 UU, paired with hold-last-fresh-angular baseline; contexts are marginal and may overlap; previous simulator geometry uses the last synchronized state, while recent simulator events include the target interval and are offline diagnostics",
        boxcars_version: "0.11.5",
        rocketsim_revision: replay_to_rocketsim::serialization::ROCKETSIM_REVISION,
        options,
        succeeded,
        failures,
        replay_sha256,
        all: all.summary(),
        by_game_size: by_game_size
            .into_iter()
            .map(|(k, v)| (k, v.summary()))
            .collect(),
        by_replay: by_replay
            .into_iter()
            .map(|(k, v)| (k, v.summary()))
            .collect(),
        by_context: by_context
            .into_iter()
            .map(|(k, v)| (k, v.summary()))
            .collect(),
        worst_regret,
    };
    fs::write(&output, serde_json::to_vec_pretty(&report)?)?;
    Ok(())
}
