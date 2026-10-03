//! Measure how well an inferred aerial control predicts the control fitted a short time later.
//!
//! For every pair of consecutive fresh car angular packets in the air, the constant RocketSim
//! control that explains the change is fitted (`solve_span_air_controls`). Fitted controls of the
//! same car actor lifetime are then paired at increasing lags. The tool uses only the packets and
//! the input process; no prediction error of any state field is involved. It is meant for train
//! replays and reports (a) the least-squares persistence coefficient by lag and (b) the mean later
//! control given the earlier control's magnitude.

use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::Mat3A;
use replay_to_rocketsim::conversion::{quaternion, solve_span_air_controls};
use replay_to_rocketsim::{observations, parse_replay};

const LAG_BIN: f32 = 1.0 / 30.0;
const LAG_BINS: usize = 9;
const MAGNITUDE_EDGES: [f32; 9] = [-1.0, -0.7, -0.5, -0.3, -0.1, 0.1, 0.3, 0.5, 0.7];

struct Packet {
    frame: usize,
    time: f32,
    rot: Mat3A,
    omega: [f32; 3],
    dodge_odd: bool,
    active: bool,
}

struct Span {
    mid: f32,
    controls: [f32; 3],
    /// Angular speed at the end packet (rad/s).
    end_speed: f32,
    /// Control of the immediately preceding adjacent span, when one exists.
    previous: Option<[f32; 3]>,
}

#[derive(Default, Clone, Copy)]
struct Accumulator {
    cross: f64,
    energy: f64,
    count: usize,
}

fn replay_paths(path: &Path) -> Result<Vec<PathBuf>, Box<dyn Error>> {
    if path.is_file() {
        return Ok(vec![path.to_owned()]);
    }
    let mut result = Vec::new();
    for size in ["1v1", "2v2", "3v3"] {
        for entry in fs::read_dir(path.join(size))? {
            let path = entry?.path();
            if path.extension().is_some_and(|ext| ext == "replay") {
                result.push(path);
            }
        }
    }
    result.sort();
    Ok(result)
}

fn magnitude_bin(value: f32) -> usize {
    MAGNITUDE_EDGES
        .iter()
        .rposition(|&edge| value >= edge)
        .map_or(0, |bin| bin)
}

fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: calibrate_air_control_persistence <train replay or directory>")?,
    );
    if replay_to_rocketsim::sealed_path_refused(&path, false) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    // Optional second argument: restrict fitted spans to at most this long (seconds). Short spans
    // average the control over less time, so they attenuate short inputs less.
    let max_span_seconds = env::args_os()
        .nth(2)
        .map_or(Ok(0.15f32), |value| value.to_string_lossy().parse())?;
    let min_z = 50.0f32;
    let axes = ["pitch", "yaw", "roll"];
    let mut by_lag = [[Accumulator::default(); LAG_BINS]; 3];
    let mut by_magnitude = [[(0.0f64, 0.0f64, 0usize); MAGNITUDE_EDGES.len()]; 3];
    let mut span_count = 0usize;
    // hold[axis][lag band][|u| bin] = (held count, total); held = later control keeps the sign
    // and at least half of the earlier magnitude.
    const HOLD_EDGES: [f32; 6] = [0.1, 0.3, 0.5, 0.7, 0.9, 1.01];
    let mut hold = [[[(0usize, 0usize); 6]; 3]; 3];
    // Same statistic, conditioned on the larger of |pitch| and |roll| of the earlier span, for
    // lag 0.033-0.133 s: [axis][joint |u| bin][own |u| below 0.1, or >= 0.1].
    let mut joint = [[(0usize, 0usize); 6]; 3];
    // Later control (sign-aligned with the earlier one) samples for medians:
    // [axis][lag band][|u| bin] -> (sum of earlier |u|, later aligned values).
    let mut medians: Vec<Vec<Vec<(f64, Vec<f32>)>>> = vec![vec![vec![(0.0, Vec::new()); 6]; 3]; 3];
    // (earlier control, previous control, cap flag, later control) for lags 0.033-0.133 s.
    let mut regression: [Vec<[f64; 4]>; 3] = Default::default();

    for replay_path in replay_paths(&path)? {
        let replay = parse_replay(&fs::read(&replay_path)?)?;
        let observed = observations::extract(&replay).ok_or("network frames absent")?;
        let mut lifetimes: std::collections::BTreeMap<(i32, usize), Vec<Packet>> =
            Default::default();
        for frame in &observed.frames {
            let active = frame
                .game_state
                .as_ref()
                .is_some_and(|state| state.value == "Active");
            for car in &frame.cars {
                let body = &car.body;
                let (Some(pos), Some(rot), Some(ang)) = (
                    &body.position,
                    &body.rotation_xyzw,
                    &body.angular_velocity_replay_units,
                ) else {
                    continue;
                };
                if pos.frame != frame.index
                    || rot.frame != frame.index
                    || ang.frame != frame.index
                    || pos.value[2] <= min_z
                {
                    continue;
                }
                let Some(q) = quaternion(rot.value) else {
                    continue;
                };
                lifetimes
                    .entry((car.actor_id, car.actor_created_frame))
                    .or_default()
                    .push(Packet {
                        frame: frame.index,
                        time: frame.time,
                        rot: Mat3A::from_quat(q),
                        omega: ang.value.map(|axis| axis * 0.01),
                        dodge_odd: car
                            .inputs
                            .dodge_active_raw
                            .as_ref()
                            .is_some_and(|d| d.frame == frame.index && d.value % 2 == 1),
                        active,
                    });
            }
        }
        for packets in lifetimes.values() {
            let mut spans = Vec::new();
            for pair in packets.windows(2) {
                let (a, b) = (&pair[0], &pair[1]);
                let dt = b.time - a.time;
                if !(a.active && b.active) || b.dodge_odd || !(0.0..=max_span_seconds).contains(&dt)
                {
                    continue;
                }
                if observed.frames[a.frame..=b.frame].iter().any(|frame| {
                    frame
                        .game_state
                        .as_ref()
                        .is_none_or(|state| state.value != "Active")
                }) {
                    continue;
                }
                let controls = solve_span_air_controls(
                    a.rot,
                    a.omega.into(),
                    b.omega.into(),
                    (dt * 120.0).round().max(1.0) as u32,
                    1,
                );
                let previous = spans.last().and_then(|last: &Span| {
                    ((a.time - last.mid).abs() < 0.5 * dt + 0.05).then_some(last.controls)
                });
                spans.push(Span {
                    mid: 0.5 * (a.time + b.time),
                    controls: [controls.pitch, controls.yaw, controls.roll],
                    end_speed: b.omega.iter().map(|w| w * w).sum::<f32>().sqrt(),
                    previous,
                });
            }
            span_count += spans.len();
            for (i, earlier) in spans.iter().enumerate() {
                for later in &spans[i + 1..] {
                    let lag = later.mid - earlier.mid;
                    if lag > LAG_BIN * LAG_BINS as f32 {
                        break;
                    }
                    let bin = ((lag / LAG_BIN) as usize).min(LAG_BINS - 1);
                    for axis in 0..3 {
                        let (u, v) = (earlier.controls[axis] as f64, later.controls[axis] as f64);
                        let acc = &mut by_lag[axis][bin];
                        acc.cross += u * v;
                        acc.energy += u * u;
                        acc.count += 1;
                        let band = if (LAG_BIN..LAG_BIN * 2.5).contains(&lag) {
                            Some(0)
                        } else if (LAG_BIN * 2.5..LAG_BIN * 4.0).contains(&lag) {
                            Some(1)
                        } else if (LAG_BIN * 4.0..LAG_BIN * 6.0).contains(&lag) {
                            Some(2)
                        } else {
                            None
                        };
                        if matches!(band, Some(0) | Some(1)) {
                            let m = earlier.controls[0].abs().max(earlier.controls[2].abs());
                            if let Some(bin) = HOLD_EDGES.iter().position(|&edge| m < edge) {
                                if earlier.controls[axis].abs() >= 0.1 {
                                    let cell = &mut joint[axis][bin];
                                    cell.1 += 1;
                                    if (v * u) as f32 >= 0.5 * earlier.controls[axis].powi(2) {
                                        cell.0 += 1;
                                    }
                                }
                            }
                        }
                        if let Some(band) = band {
                            let magnitude = earlier.controls[axis].abs();
                            if let Some(bin) = HOLD_EDGES.iter().position(|&edge| magnitude < edge)
                            {
                                if magnitude >= 0.1 {
                                    let cell = &mut medians[axis][band][bin];
                                    cell.0 += f64::from(magnitude);
                                    cell.1.push(
                                        later.controls[axis] * earlier.controls[axis].signum(),
                                    );
                                    let cell = &mut hold[axis][band][bin];
                                    cell.1 += 1;
                                    if (v * u) as f32 >= 0.5 * earlier.controls[axis].powi(2) {
                                        cell.0 += 1;
                                    }
                                }
                            }
                        }
                        if (LAG_BIN * 1.0..LAG_BIN * 4.0).contains(&lag) {
                            if let Some(previous) = earlier.previous {
                                regression[axis].push([
                                    u,
                                    previous[axis] as f64,
                                    f64::from(earlier.end_speed >= 5.48),
                                    v,
                                ]);
                            }
                            let cell =
                                &mut by_magnitude[axis][magnitude_bin(earlier.controls[axis])];
                            cell.0 += u;
                            cell.1 += v;
                            cell.2 += 1;
                        }
                    }
                }
            }
        }
    }

    println!("fitted spans (air, {min_z} UU+, <= {max_span_seconds}s): {span_count}");
    println!("\nleast-squares persistence coefficient rho(lag) = sum(u_i*u_j)/sum(u_i^2)");
    println!(
        "{:>14} {:>9} {:>9} {:>9} {:>9}",
        "lag (s)", "pairs", "pitch", "yaw", "roll"
    );
    for bin in 0..LAG_BINS {
        println!(
            "{:>6.3}-{:<7.3} {:>9} {:>9.3} {:>9.3} {:>9.3}",
            bin as f32 * LAG_BIN,
            (bin + 1) as f32 * LAG_BIN,
            by_lag[0][bin].count,
            by_lag[0][bin].cross / by_lag[0][bin].energy.max(1e-9),
            by_lag[1][bin].cross / by_lag[1][bin].energy.max(1e-9),
            by_lag[2][bin].cross / by_lag[2][bin].energy.max(1e-9),
        );
    }
    println!("\nmean later control given earlier control bin, lag 0.033-0.133 s");
    println!("(ratio = mean later / mean earlier; ratio near 1 means the control persists)");
    for (axis, name) in axes.iter().enumerate() {
        println!("{name}:");
        for (bin, cell) in by_magnitude[axis].iter().enumerate() {
            if cell.2 == 0 {
                continue;
            }
            let hi = MAGNITUDE_EDGES.get(bin + 1).copied().unwrap_or(1.0);
            let (earlier, later) = (cell.0 / cell.2 as f64, cell.1 / cell.2 as f64);
            println!(
                "  earlier in [{:>5.1},{:>4.1}) n={:>7} mean earlier {:>7.3} later {:>7.3} ratio {:>6.3}",
                MAGNITUDE_EDGES[bin],
                hi,
                cell.2,
                earlier,
                later,
                if earlier.abs() > 1e-6 {
                    later / earlier
                } else {
                    f64::NAN
                }
            );
        }
    }
    println!(
        "
P(held) = P(later control keeps sign and >= half the earlier magnitude)"
    );
    println!("lag bands: A 0.033-0.083 s, B 0.083-0.133 s, C 0.133-0.200 s; |u| bins from 0.1");
    for (axis, name) in axes.iter().enumerate() {
        for (band, label) in ["A", "B", "C"].iter().enumerate() {
            let cells: Vec<String> = hold[axis][band]
                .iter()
                .enumerate()
                .map(|(bin, (held, total))| {
                    format!(
                        "[{:.1},{:.1}) {:.3} ({})",
                        if bin == 0 { 0.1 } else { HOLD_EDGES[bin - 1] },
                        HOLD_EDGES[bin].min(1.0),
                        *held as f64 / (*total).max(1) as f64,
                        total
                    )
                })
                .collect();
            println!("{name} {label}: {}", cells.join(" | "));
        }
    }
    println!(
        "
median later control (aligned to the earlier sign) / mean earlier |u|, by lag band A/B/C"
    );
    for (axis, name) in axes.iter().enumerate() {
        for (band, label) in ["A", "B", "C"].iter().enumerate() {
            let cells: Vec<String> = medians[axis][band]
                .iter_mut()
                .enumerate()
                .filter(|(_, cell)| !cell.1.is_empty())
                .map(|(_, cell)| {
                    cell.1.sort_by(|a, b| a.total_cmp(b));
                    let n = cell.1.len();
                    let median = cell.1[n / 2];
                    let (q1, q3) = (cell.1[n / 4], cell.1[3 * n / 4]);
                    format!(
                        "|u|~{:.2}: med {:.3} ratio {:.3} [q1 {:.2} q3 {:.2}]",
                        cell.0 / n as f64,
                        median,
                        median as f64 / (cell.0 / n as f64),
                        q1,
                        q3
                    )
                })
                .collect();
            println!("{name} {label}: {}", cells.join(" | "));
        }
    }
    println!(
        "
P(held) by joint context m = max(|pitch|,|roll|) of the earlier span, lags 0.033-0.133 s,"
    );
    println!("only for axes whose own earlier |u| >= 0.1; bins of m as above");
    for (axis, name) in axes.iter().enumerate() {
        let cells: Vec<String> = joint[axis]
            .iter()
            .enumerate()
            .map(|(bin, (held, total))| {
                format!(
                    "m<{:.1}: {:.3} ({})",
                    HOLD_EDGES[bin].min(1.0),
                    *held as f64 / (*total).max(1) as f64,
                    total
                )
            })
            .collect();
        println!("{name}: {}", cells.join(" | "));
    }
    println!(
        "
regression of later control on earlier context, lags 0.033-0.133 s, spans with a predecessor"
    );
    println!(
        "models: M1 u_j=a*u_i ; M2 adds b*u_prev ; M3 adds c*cap*u_i (cap = end speed >= 5.48 rad/s)"
    );
    for (axis, name) in axes.iter().enumerate() {
        let rows = &regression[axis];
        let total: f64 = rows.iter().map(|r| r[3] * r[3]).sum();
        let fit = |columns: &[fn(&[f64; 4]) -> f64]| -> (Vec<f64>, f64) {
            let n = columns.len();
            let mut xtx = vec![vec![0.0f64; n]; n];
            let mut xty = vec![0.0f64; n];
            for row in rows {
                let x: Vec<f64> = columns.iter().map(|c| c(row)).collect();
                for i in 0..n {
                    xty[i] += x[i] * row[3];
                    for j in 0..n {
                        xtx[i][j] += x[i] * x[j];
                    }
                }
            }
            let beta = solve_linear(xtx, xty);
            let sse: f64 = rows
                .iter()
                .map(|row| {
                    let prediction: f64 = columns.iter().zip(&beta).map(|(c, b)| c(row) * b).sum();
                    (row[3] - prediction).powi(2)
                })
                .sum();
            (beta, 1.0 - sse / total)
        };
        let (m1, r1) = fit(&[|r| r[0]]);
        let (m2, r2) = fit(&[|r| r[0], |r| r[1]]);
        let (m3, r3) = fit(&[|r| r[0], |r| r[1], |r| r[2] * r[0]]);
        println!(
            "{name}: n={} | M1 a={:.3} R2={:.3} | M2 a={:.3} b={:.3} R2={:.3} | M3 a={:.3} b={:.3} c={:.3} R2={:.3}",
            rows.len(),
            m1[0],
            r1,
            m2[0],
            m2[1],
            r2,
            m3[0],
            m3[1],
            m3[2],
            r3
        );
    }
    Ok(())
}

/// Solves a small dense system by Gaussian elimination with partial pivoting.
fn solve_linear(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Vec<f64> {
    let n = b.len();
    for col in 0..n {
        let pivot = (col..n)
            .max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))
            .unwrap();
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..n {
            let factor = a[row][col] / a[col][col];
            for k in col..n {
                a[row][k] -= factor * a[col][k];
            }
            b[row] -= factor * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let tail: f64 = (row + 1..n).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - tail) / a[row][row];
    }
    x
}
