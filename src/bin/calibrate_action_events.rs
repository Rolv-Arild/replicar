//! Relate replay component-counter transitions to nearby observed car motion.

use std::collections::HashMap;
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use replay_to_rocketsim::{observations, parse_replay};

#[derive(Clone, Copy)]
struct Motion {
    frame: usize,
    time: f32,
    z: f32,
    vz: f32,
}

#[derive(Clone, Copy)]
struct Activation {
    kind: usize,
    frame: usize,
    time: f32,
    from: u8,
    to: u8,
    torque_fresh: bool,
}

#[derive(Default)]
struct Trace {
    counters: [Option<u8>; 3],
    motions: Vec<Motion>,
    activations: Vec<Activation>,
}

#[derive(Default)]
struct Counts {
    transitions: usize,
    step_one: usize,
    torque_fresh: usize,
    paired_motion: usize,
    grounded_before: usize,
    upward_dvz_100: usize,
    upward_dvz_300: usize,
    rising_z_10: usize,
    dvz: Vec<f32>,
    fresh_at_event: usize,
    grounded_at_event: usize,
    upward_at_event: usize,
    pre_event_dvz: Vec<f32>,
    post_event_dvz: Vec<f32>,
}

fn quantile(values: &mut [f32], fraction: f64) -> Option<f32> {
    if values.is_empty() {
        return None;
    }
    values.sort_by(f32::total_cmp);
    Some(values[((values.len() - 1) as f64 * fraction).round() as usize])
}

fn paths(root: &Path) -> Result<Vec<PathBuf>, Box<dyn Error>> {
    let mut result = Vec::new();
    if root.is_file() {
        result.push(root.to_owned());
    } else {
        for size in ["1v1", "2v2", "3v3"] {
            for entry in fs::read_dir(root.join(size))? {
                let path = entry?.path();
                if path.extension().is_some_and(|ext| ext == "replay") {
                    result.push(path);
                }
            }
        }
    }
    result.sort();
    Ok(result)
}

fn record_motion(counts: &mut Counts, activation: Activation, motions: &[Motion]) {
    let before_index = motions.partition_point(|motion| motion.frame < activation.frame);
    let after_index = motions.partition_point(|motion| motion.frame <= activation.frame);
    if let Some(current) = motions
        .get(before_index)
        .filter(|motion| motion.frame == activation.frame)
    {
        counts.fresh_at_event += 1;
        counts.grounded_at_event += usize::from(current.z < 50.0);
        counts.upward_at_event += usize::from(current.vz > 200.0);
        if let Some(before) = before_index
            .checked_sub(1)
            .and_then(|index| motions.get(index))
        {
            if activation.time - before.time <= 0.15 {
                counts.pre_event_dvz.push(current.vz - before.vz);
            }
        }
        if let Some(after) = motions.get(after_index) {
            if after.time - activation.time <= 0.15 {
                counts.post_event_dvz.push(after.vz - current.vz);
            }
        }
    }
    let (Some(before), Some(after)) = (
        before_index
            .checked_sub(1)
            .and_then(|index| motions.get(index)),
        motions.get(after_index),
    ) else {
        return;
    };
    if before.frame >= activation.frame
        || after.frame <= activation.frame
        || activation.time - before.time > 0.15
        || after.time - activation.time > 0.15
    {
        return;
    }
    counts.paired_motion += 1;
    counts.grounded_before += usize::from(before.z < 50.0);
    let dvz = after.vz - before.vz;
    counts.upward_dvz_100 += usize::from(dvz > 100.0);
    counts.upward_dvz_300 += usize::from(dvz > 300.0);
    counts.rising_z_10 += usize::from(after.z - before.z > 10.0);
    counts.dvz.push(dvz);
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: calibrate_action_events <train replay or split directory>")?,
    );
    let replay_paths = paths(&path)?;
    // For each component, separate even-to-odd and odd-to-even transitions.
    let mut counts = std::array::from_fn::<_, 6, _>(|_| Counts::default());
    let mut fresh_updates = [0usize; 3];
    for path in &replay_paths {
        let observed = observations::extract(&parse_replay(&fs::read(path)?)?)
            .ok_or("network frames absent")?;
        let mut traces: HashMap<(i32, usize), Trace> = HashMap::new();
        for frame in &observed.frames {
            if !frame
                .game_state
                .as_ref()
                .is_some_and(|state| state.value == "Active")
            {
                continue;
            }
            for car in observations::primary_linked_cars(frame) {
                let trace = traces
                    .entry((car.actor_id, car.actor_created_frame))
                    .or_default();
                if let (Some(position), Some(velocity)) =
                    (&car.body.position, &car.body.linear_velocity)
                {
                    if position.frame == frame.index && velocity.frame == frame.index {
                        trace.motions.push(Motion {
                            frame: frame.index,
                            time: frame.time,
                            z: position.value[2],
                            vz: velocity.value[2],
                        });
                    }
                }
                for (kind, value) in [
                    &car.inputs.jump_active_raw,
                    &car.inputs.double_jump_active_raw,
                    &car.inputs.dodge_active_raw,
                ]
                .into_iter()
                .enumerate()
                {
                    let Some(value) = value.as_ref().filter(|value| value.frame == frame.index)
                    else {
                        continue;
                    };
                    fresh_updates[kind] += 1;
                    if let Some(from) = trace.counters[kind] {
                        if from != value.value {
                            trace.activations.push(Activation {
                                kind,
                                frame: frame.index,
                                time: frame.time,
                                from,
                                to: value.value,
                                torque_fresh: car
                                    .inputs
                                    .dodge_torque_replay_units
                                    .as_ref()
                                    .is_some_and(|torque| torque.frame == frame.index),
                            });
                        }
                    }
                    trace.counters[kind] = Some(value.value);
                }
            }
        }
        for trace in traces.values() {
            for activation in &trace.activations {
                let index = activation.kind * 2 + usize::from(activation.to % 2 == 0);
                let group = &mut counts[index];
                group.transitions += 1;
                group.step_one += usize::from(activation.to.wrapping_sub(activation.from) == 1);
                group.torque_fresh += usize::from(activation.torque_fresh);
                record_motion(group, *activation, &trace.motions);
            }
        }
    }
    println!(
        "{} replay(s); fresh updates jump/double-jump/dodge={fresh_updates:?}",
        replay_paths.len()
    );
    for (index, group) in counts.iter_mut().enumerate() {
        let kind = ["jump", "double-jump", "dodge"][index / 2];
        let parity = ["to odd", "to even"][index % 2];
        println!(
            "{kind} {parity}: transitions={} +1={} fresh_torque={} paired={} grounded_before={} dvz>100={} dvz>300={} dz>10={} dvz_p50={:?}",
            group.transitions,
            group.step_one,
            group.torque_fresh,
            group.paired_motion,
            group.grounded_before,
            group.upward_dvz_100,
            group.upward_dvz_300,
            group.rising_z_10,
            quantile(&mut group.dvz, 0.5),
        );
        println!(
            "  at_event={} z<50={} vz>200={} pre_dvz_p50={:?} post_dvz_p50={:?}",
            group.fresh_at_event,
            group.grounded_at_event,
            group.upward_at_event,
            quantile(&mut group.pre_event_dvz, 0.5),
            quantile(&mut group.post_event_dvz, 0.5),
        );
    }
    Ok(())
}
