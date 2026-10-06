//! The ball's runs: free flight, hits bridged by the free-flight paths, and the placement of the ball runs
//! against the cars.

use std::collections::BTreeMap;

use glam::{Mat3A, Quat, Vec3A};
use rocketsim::CarBodyConfig;

use super::CarSample;
use super::chains::{ChainPacket, RawRun};
use crate::decode::CarLife;

const DT: f32 = 1.0 / 120.0;

/// One tick of the ball in free flight: gravity, then the exponential damping, and the position moves with
/// the new velocity (against server states: 0.008 UU/s and 0.005 UU per tick).
fn free_step(pos: [f32; 3], vel: [f32; 3]) -> ([f32; 3], [f32; 3]) {
    let keep = 0.97f32.powf(DT);
    let vel = [vel[0] * keep, vel[1] * keep, vel[2] * keep - 650.0 * DT];
    (
        [
            pos[0] + vel[0] * DT,
            pos[1] + vel[1] * DT,
            pos[2] + vel[2] * DT,
        ],
        vel,
    )
}

/// The inverse of `free_step`: the state one tick earlier.
fn free_step_back(pos: [f32; 3], vel: [f32; 3]) -> ([f32; 3], [f32; 3]) {
    let keep = 0.97f32.powf(DT);
    (
        [
            pos[0] - vel[0] * DT,
            pos[1] - vel[1] * DT,
            pos[2] - vel[2] * DT,
        ],
        [vel[0] / keep, vel[1] / keep, (vel[2] + 650.0 * DT) / keep],
    )
}

/// Away from the floor, ceiling, side walls and goals, where a bounce would change the velocity like a hit.
fn in_free_air(pos: [f32; 3]) -> bool {
    pos[2] > 125.0 && pos[2] < 1900.0 && pos[0].abs() < 3900.0 && pos[1].abs() < 4900.0
}

/// A hit between two ball updates that the chain bridged: the last free-flight state before it.
#[derive(Debug, Clone)]
pub(super) struct BallHit {
    pub(super) frame_a: usize,
    pub(super) frame_b: usize,
    pub(super) ticks_after_a: usize,
    pub(super) ball_pos: [f32; 3],
}

/// Elapsed ticks between two ball updates with a hit between them. The ball follows its exact free-flight
/// path up to the hit and, after it, the exact path leading to the second update; the two paths meet at the
/// hit up to the part of the hit tick spent moving with the new velocity, which lies along the velocity
/// change (measured between -0.4 and +0.4 of one tick of the change at the 10th to 90th percentile; the
/// window here is wider). The elapsed time `d` within the frame bounds `d_lo..=d_hi` that leaves the smallest
/// component across the velocity change wins, by more than 2 UU over every other candidate, or the pair is
/// not used. Returns `d`, the ticks from the first update to the hit, and the ball's position there.
pub(super) fn ball_hit_interval_ticks(
    a: &ChainPacket,
    b: &ChainPacket,
    d_lo: i64,
    d_hi: i64,
) -> Option<(i64, usize, [f32; 3])> {
    const MIN_VELOCITY_CHANGE: f32 = 400.0;
    const HIT_MARGIN: f32 = 2.0;
    const ALONG_MIN: f32 = -0.6;
    const ALONG_MAX: f32 = 1.2;
    let dv = [
        b.vel[0] - a.vel[0],
        b.vel[1] - a.vel[1],
        b.vel[2] - a.vel[2],
    ];
    let dv_norm = (dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]).sqrt();
    if dv_norm < MIN_VELOCITY_CHANGE || d_hi < d_lo.max(1) || d_hi > 64 {
        return None;
    }
    let dir = [dv[0] / dv_norm, dv[1] / dv_norm, dv[2] / dv_norm];
    let steps = d_hi as usize;
    // The pre-hit path forward from a and the post-hit path back from b, while in free flight.
    let mut pre = vec![a.pos];
    let (mut p, mut v) = (a.pos, a.vel);
    for _ in 0..steps {
        (p, v) = free_step(p, v);
        if !in_free_air(p) {
            break;
        }
        pre.push(p);
    }
    let mut post = vec![b.pos];
    let (mut p, mut v) = (b.pos, b.vel);
    for _ in 0..steps {
        (p, v) = free_step_back(p, v);
        if !in_free_air(p) {
            break;
        }
        post.push(p);
    }
    if !in_free_air(a.pos) || !in_free_air(b.pos) {
        return None;
    }
    let mut costs: Vec<(i64, usize, f32)> = Vec::new();
    for d in d_lo.max(1)..=d_hi {
        let mut best = f32::INFINITY;
        let mut best_t1 = 0;
        for t1 in 0..=(d as usize) {
            let t2 = d as usize - t1;
            let (Some(pa), Some(pb)) = (pre.get(t1), post.get(t2)) else {
                continue;
            };
            let r = [pa[0] - pb[0], pa[1] - pb[1], pa[2] - pb[2]];
            let along = r[0] * dir[0] + r[1] * dir[1] + r[2] * dir[2];
            if !(ALONG_MIN..=ALONG_MAX).contains(&(along / (dv_norm * DT))) {
                continue;
            }
            let across = [
                r[0] - along * dir[0],
                r[1] - along * dir[1],
                r[2] - along * dir[2],
            ];
            let across =
                (across[0] * across[0] + across[1] * across[1] + across[2] * across[2]).sqrt();
            if across < best {
                best = across;
                best_t1 = t1;
            }
        }
        costs.push((d, best_t1, best));
    }
    costs.sort_by(|x, y| x.2.total_cmp(&y.2));
    let (first, second) = (costs.first()?, costs.get(1));
    if !first.2.is_finite() {
        return None;
    }
    match second {
        Some(second) if second.2 - first.2 <= HIT_MARGIN => None,
        _ => Some((first.0, first.1, pre[first.1])),
    }
}

/// Places the ball runs relative to the car runs. In one frame the ball's server tick minus a car's is
/// `offset` on average (standard deviation 2.55 ticks per pair on the two remote-client games; the offset 3.1
/// there, stable across cars, minutes and games), so every ball run starts at the mean over its frames of
/// (the frame's car ticks + offset - its own K), rounded and clamped to its feasible range. Car runs do not
/// move: pulling them toward the ball's noisy levels made the cars worse.
pub(super) fn place_ball_runs(
    ball_runs: &mut [RawRun],
    car_runs: &[(CarLife, RawRun)],
    offset: f32,
) {
    let mut car_ticks: BTreeMap<usize, (f64, usize)> = BTreeMap::new();
    for (_, run) in car_runs {
        for &(frame, k) in &run.entries {
            let entry = car_ticks.entry(frame).or_default();
            entry.0 += (run.start + k) as f64;
            entry.1 += 1;
        }
    }
    for run in ball_runs.iter_mut() {
        let (mut sum, mut count) = (0.0f64, 0usize);
        for &(frame, k) in &run.entries {
            if let Some(&(total, n)) = car_ticks.get(&frame) {
                // The frame's mean car tick, once per car.
                sum += n as f64 * (total / n as f64 + f64::from(offset) - k as f64);
                count += n;
            }
        }
        if count > 0 {
            run.start = ((sum / count as f64).round() as i64).clamp(run.lo, run.hi);
        }
    }
}

/// The ball-minus-car offset at which the median hit is a touch. The gap between the hitting car and the
/// ball at the last state before a hit shrinks as the offset grows (the car is placed earlier relative to the
/// ball); it is evaluated on a grid of offsets, and the first step where the median gap reaches zero is
/// interpolated linearly. (On the two remote-client games, truth 3.1 ticks: 2.75 and 2.97.)
pub(super) fn estimate_ball_car_offset(
    hits: &[BallHit],
    ball_runs: &[RawRun],
    car_runs: &[(CarLife, RawRun)],
    samples: &BTreeMap<(CarLife, usize), CarSample>,
    hitboxes: &BTreeMap<CarLife, CarBodyConfig>,
) -> Option<f32> {
    const STEP: f32 = 0.5;
    let mut previous: Option<(f32, f32)> = None;
    for step in -8..=20 {
        let offset = step as f32 * STEP;
        let mut placed = ball_runs.to_vec();
        place_ball_runs(&mut placed, car_runs, offset);
        let gap = median_hit_gap(hits, &placed, car_runs, samples, hitboxes)?;
        if gap <= 0.0 {
            let (before, gap_before) = previous?;
            return Some(before + STEP * gap_before / (gap_before - gap));
        }
        previous = Some((offset, gap));
    }
    None
}

/// The median over the bridged hits of the gap between the closest car's hitbox and the ball at the last
/// state before the hit (negative: overlapping), with the runs as they are placed. Each car's update nearest
/// the hit is extrapolated to it with its velocity, gravity when off the ground, and its angular velocity
/// (a few UU of error over a few ticks). `None` with fewer than 20 hits.
fn median_hit_gap(
    hits: &[BallHit],
    ball_runs: &[RawRun],
    car_runs: &[(CarLife, RawRun)],
    samples: &BTreeMap<(CarLife, usize), CarSample>,
    hitboxes: &BTreeMap<CarLife, CarBodyConfig>,
) -> Option<f32> {
    const MAX_EXTRAPOLATION_TICKS: i64 = 12;
    let mut gaps: Vec<f32> = Vec::new();
    for hit in hits {
        let Some(run) = ball_runs.iter().find(|r| {
            r.entries.iter().any(|e| e.0 == hit.frame_a)
                && r.entries.iter().any(|e| e.0 == hit.frame_b)
        }) else {
            continue;
        };
        let Some(&(_, ka)) = run.entries.iter().find(|e| e.0 == hit.frame_a) else {
            continue;
        };
        let tick = run.start + ka + hit.ticks_after_a as i64;
        let ball = Vec3A::from(hit.ball_pos);
        let mut best: Option<f32> = None;
        for (life, car_run) in car_runs {
            let Some(&(frame, k)) = car_run
                .entries
                .iter()
                .min_by_key(|(_, k)| (car_run.start + k - tick).abs())
            else {
                continue;
            };
            let delta = tick - (car_run.start + k);
            if delta.abs() > MAX_EXTRAPOLATION_TICKS {
                continue;
            }
            let (Some(sample), Some(config)) = (samples.get(&(*life, frame)), hitboxes.get(life))
            else {
                continue;
            };
            let seconds = delta as f32 / 120.0;
            let gravity = if sample.pos.z > 30.0 { -650.0 } else { 0.0 };
            let pos = sample.pos
                + sample.vel * seconds
                + Vec3A::new(0.0, 0.0, 0.5 * gravity * seconds * seconds);
            let rot = Mat3A::from_quat(
                Quat::from_scaled_axis((sample.ang * seconds).into()) * sample.rot,
            );
            let local = rot.transpose() * (ball - pos) - config.hitbox_pos_offset;
            let q = local.abs() - config.hitbox_size * 0.5;
            let gap = q.max(Vec3A::ZERO).length() + q.max_element().min(0.0) - 91.25;
            best = Some(best.map_or(gap, |b: f32| b.min(gap)));
        }
        if let Some(gap) = best {
            gaps.push(gap);
        }
    }
    if gaps.len() < 20 {
        return None;
    }
    gaps.sort_by(|a, b| a.total_cmp(b));
    Some(gaps[gaps.len() / 2])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_free_flight_step_back_undoes_a_step() {
        let (pos, vel) = ([100.0, -200.0, 800.0], [1200.0, 300.0, -500.0]);
        let (p1, v1) = free_step(pos, vel);
        let (p0, v0) = free_step_back(p1, v1);
        for i in 0..3 {
            assert!((p0[i] - pos[i]).abs() < 1e-3 && (v0[i] - vel[i]).abs() < 1e-3);
        }
    }

    /// Two free-flight paths that meet: a ball flying for 7 ticks, hit to a new velocity, flying 5 more.
    #[test]
    fn a_hit_between_two_updates_recovers_the_elapsed_ticks() {
        let (mut p, mut v) = ([0.0, 0.0, 600.0], [800.0, 0.0, 0.0]);
        let a = ChainPacket {
            frame: 0,
            pos: p,
            vel: v,
        };
        for _ in 0..7 {
            (p, v) = free_step(p, v);
        }
        let hit_pos = p;
        v = [v[0] - 600.0, v[1] + 900.0, v[2] + 300.0];
        for _ in 0..5 {
            (p, v) = free_step(p, v);
        }
        let b = ChainPacket {
            frame: 3,
            pos: p,
            vel: v,
        };
        let (d, t1, at) = ball_hit_interval_ticks(&a, &b, 8, 16).unwrap();
        assert_eq!(d, 12);
        assert_eq!(t1, 7);
        assert!((0..3).all(|i| (at[i] - hit_pos[i]).abs() < 1e-3));
    }
}
