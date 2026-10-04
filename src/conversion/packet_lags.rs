//! Packet timing: each replay packet's server tick from chains of whole-tick intervals, ball runs
//! across hits placed against the cars, and lag-free replays.

use super::*;

/// Lags of zero for every fresh packet (`zero_packet_lag`).
pub fn zero_packet_lags(observations: &ObservedReplay) -> PacketLags {
    let frames = &observations.frames;
    let mut lags = PacketLags {
        ball: vec![None; frames.len()],
        cars: vec![None; frames.len()],
        car_actor: HashMap::new(),
        ..PacketLags::default()
    };
    for (f, frame) in frames.iter().enumerate() {
        if frame
            .ball
            .as_ref()
            .and_then(|b| b.position.as_ref())
            .is_some_and(|p| p.frame == f)
        {
            lags.ball[f] = Some(0.0);
        }
        for car in &frame.cars {
            if car.body.position.as_ref().is_some_and(|p| p.frame == f) {
                lags.cars[f] = Some(0.0);
                lags.car_actor
                    .insert((car.actor_id, car.actor_created_frame, f), 0.0);
            }
        }
    }
    lags
}

#[derive(Debug, Clone, Default)]
pub struct PacketLags {
    /// Ball packet lag in ticks behind each frame time, when inferred.
    pub ball: Vec<Option<f32>>,
    /// Median lag of the cars with a fresh packet in each frame, when inferred.
    pub cars: Vec<Option<f32>>,
    /// Lag of one car's own packet, keyed by (actor id, creation frame, frame).
    pub car_actor: HashMap<(i32, usize, usize), f32>,
    /// Diagnostics of the ball-car placement: the offset used (ticks, ball minus car) and the
    /// number of bridged ball hits found.
    pub ball_car_offset: Option<f32>,
    pub bridged_hits: usize,
    /// The replay was detected as lag-free (`detect_lag_free_replays`: a server-saved replay whose packets
    /// all sit at their frame's own tick) and every fresh packet got a lag of zero.
    pub lag_free: bool,
    /// The exact-chain runs of the cars: every packet of one run shares one level (its start), which
    /// may move by whole ticks inside `[lo, hi]` (`contact_alignment` moves whole runs).
    pub car_runs: Vec<CarRun>,
    /// The run (index in `car_runs`) of each car packet that belongs to one, by (actor, lifetime, frame).
    pub car_run_of: HashMap<(i32, usize, usize), usize>,
}

/// One exact-chain run of a car's packets on the integer timeline.
#[derive(Debug, Clone)]
pub struct CarRun {
    pub actor: i32,
    pub created: usize,
    /// (frame, K): the packet's physical tick is `start + K`.
    pub entries: Vec<(usize, i64)>,
    /// Feasible integer starts and the chosen one.
    pub lo: i64,
    pub hi: i64,
    pub start: i64,
}

/// One fresh packet of a chained object: frame index, position, and velocity.
pub(super) struct ChainPacket {
    pub(super) frame: usize,
    pub(super) pos: [f32; 3],
    pub(super) vel: [f32; 3],
}

/// Ticks of physical time between two packets, from the displacement along their mean velocity.
pub(super) fn implied_interval_ticks(a: &ChainPacket, b: &ChainPacket) -> Option<f32> {
    let mean = [
        0.5 * (a.vel[0] + b.vel[0]),
        0.5 * (a.vel[1] + b.vel[1]),
        0.5 * (a.vel[2] + b.vel[2]),
    ];
    let speed_sq = mean[0] * mean[0] + mean[1] * mean[1] + mean[2] * mean[2];
    if speed_sq < 1.0 {
        return None;
    }
    let dot = (0..3).map(|i| (b.pos[i] - a.pos[i]) * mean[i]).sum::<f32>();
    let ticks = dot / speed_sq * 120.0;
    ticks.is_finite().then_some(ticks)
}

pub(super) fn packet_pos(car: &observations::Car, frame: usize) -> [f32; 3] {
    car.body
        .position
        .as_ref()
        .filter(|v| v.frame == frame)
        .map_or([0.0; 3], |v| v.value)
}

pub(super) fn packet_vel(car: &observations::Car, frame: usize) -> [f32; 3] {
    car.body
        .linear_velocity
        .as_ref()
        .filter(|v| v.frame == frame)
        .map_or([0.0; 3], |v| v.value)
}

/// The ball-minus-car offset at which the median hit is a touch. The gap between the hitting car
/// and the ball at the last state before a hit shrinks as the offset grows (the car is placed
/// earlier relative to the ball). It is evaluated on a grid of offsets with the runs placed by
/// `place_ball_runs`; the first grid step where the median gap drops to zero or below is
/// interpolated linearly. (On the two remote-client games, truth 3.1 ticks: 2.75 and 2.97.)
pub(super) fn estimate_ball_car_offset(
    hits: &[BallHit],
    ball_runs: &[RawRun],
    car_runs: &[((i32, usize), RawRun)],
    samples: &HashMap<(i32, usize, usize), CarSample>,
    hitboxes: &HashMap<(i32, usize), CarBodyConfig>,
) -> Option<f32> {
    const STEP: f32 = 0.5;
    let mut previous: Option<(f32, f32)> = None;
    for step in -8..=20 {
        let offset = step as f32 * STEP;
        let mut b = ball_runs.to_vec();
        place_ball_runs(&mut b, car_runs, offset);
        let result = median_hit_gap(hits, &b, car_runs, samples, hitboxes);
        if std::env::var_os("OFFSET_PROFILE").is_some() {
            eprintln!("offset {offset}: {result:?} of {} hits", hits.len());
        }
        let (gap, _) = result?;
        if gap <= 0.0 {
            let (before, gap_before) = previous?;
            return Some(before + STEP * gap_before / (gap_before - gap));
        }
        previous = Some((offset, gap));
    }
    None
}

/// One run of chained packets on the integer tick timeline: `(frame, K)` entries (physical tick of
/// each packet relative to the run's first), the feasible integer starts `lo..=hi` of the run (the
/// physical tick of the entry with `K = 0`), and the start chosen so far.
#[derive(Debug, Clone)]
pub(super) struct RawRun {
    pub(super) entries: Vec<(usize, i64)>,
    pub(super) lo: i64,
    pub(super) hi: i64,
    pub(super) start: i64,
}

/// Per run and entry: whether the run owns that packet's lag. Two consecutive runs of one actor can
/// share a packet (the last of the first and the first of the second); the earlier run owns it
/// (RESULTS.md, 'Packet shared by two lag runs').
pub(super) fn owned_entries(runs: &[RawRun]) -> Vec<Vec<bool>> {
    let mut owned: Vec<Vec<bool>> = runs
        .iter()
        .map(|run| vec![true; run.entries.len()])
        .collect();
    for i in 1..runs.len() {
        let (previous, next) = (&runs[i - 1], &runs[i]);
        let (Some(last), Some(first)) = (previous.entries.last(), next.entries.first()) else {
            continue;
        };
        if last.0 != first.0 {
            continue;
        }
        owned[i][0] = false;
    }
    owned
}

/// A copy of `run` with only the entries it owns (`owned_entries`).
pub(super) fn keep_owned(run: &RawRun, owned: &[bool]) -> RawRun {
    let mut run = run.clone();
    let mut flags = owned.iter();
    run.entries
        .retain(|_| flags.next().copied().unwrap_or(true));
    run
}

/// Places the ball runs relative to the car runs. A car's physical tick in a frame is
/// `tick - lag` with a lag spread uniformly over the frame window, so a car run is already
/// centred by its own window bounds (`chain_packet_lags_exact`). In one frame the ball's physical
/// tick minus a car's is `offset` on average (standard deviation 2.55 ticks per pair, measured on
/// the two remote-client games; the offset 3.1 there, stable across cars, minutes and games), so
/// every ball run starts at the mean over its frames of (the frame's car ticks + offset - its own
/// K), rounded and clamped to its feasible range. Car runs do not move: pulling them toward
/// the ball's noisy levels made the cars worse.
pub(super) fn place_ball_runs(
    ball_runs: &mut [RawRun],
    car_runs: &[((i32, usize), RawRun)],
    offset: f32,
) {
    let mut car_ticks: HashMap<usize, (f64, usize)> = HashMap::new();
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
                // The mean car tick of the frame, one term per car.
                sum += n as f64 * (total / n as f64 + f64::from(offset) - k as f64);
                count += n;
            }
        }
        if count > 0 {
            run.start = ((sum / count as f64).round() as i64).clamp(run.lo, run.hi);
        }
    }
}

/// One tick of the ball in free flight (gravity, then the exponential damping; the position moves
/// with the new velocity; measured against server states: 0.008 UU/s and 0.005 UU per tick).
pub(super) fn ball_free_step(pos: [f32; 3], vel: [f32; 3]) -> ([f32; 3], [f32; 3]) {
    const DT: f32 = 1.0 / 120.0;
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

/// The inverse of `ball_free_step`: the state one tick earlier.
pub(super) fn ball_free_step_back(pos: [f32; 3], vel: [f32; 3]) -> ([f32; 3], [f32; 3]) {
    const DT: f32 = 1.0 / 120.0;
    let keep = 0.97f32.powf(DT);
    let before = [
        pos[0] - vel[0] * DT,
        pos[1] - vel[1] * DT,
        pos[2] - vel[2] * DT,
    ];
    (
        before,
        [vel[0] / keep, vel[1] / keep, (vel[2] + 650.0 * DT) / keep],
    )
}

/// Whether a ball position is in free flight: away from the floor, ceiling, side walls and goals
/// (a bounce or a wall hit changes the velocity just like a hit does).
pub(super) fn ball_in_free_air(pos: [f32; 3]) -> bool {
    pos[2] > 125.0 && pos[2] < 1900.0 && pos[0].abs() < 3900.0 && pos[1].abs() < 4900.0
}

/// Elapsed ticks between two ball packets with a hit between them. The ball follows its exact
/// free-flight path up to the hit, and after it the exact path leading to the second packet. The
/// two paths meet at the hit up to the part of the hit tick spent moving with the new velocity:
/// the displacement of that tick lies along the velocity change, by a fraction (measured between
/// -0.4 and +0.4 of one tick of the velocity change at the 10th to 90th percentile; the window
/// here is wider). The candidate elapsed time `d` (ticks between the packets, within the real-time
/// bounds `d_lo..=d_hi` of the frame windows) that leaves the smallest component across the
/// velocity change wins; it must beat every other candidate by more than `HIT_MARGIN` UU, or the
/// pair is not used.
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
    const DT: f32 = 1.0 / 120.0;
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
    // Positions along the pre-hit path (forward from a) and the post-hit path (back from b), as
    // long as the ball stays in free flight.
    let mut pre = vec![a.pos];
    let (mut p, mut v) = (a.pos, a.vel);
    for _ in 0..steps {
        (p, v) = ball_free_step(p, v);
        if !ball_in_free_air(p) {
            break;
        }
        pre.push(p);
    }
    let mut post = vec![b.pos];
    let (mut p, mut v) = (b.pos, b.vel);
    for _ in 0..steps {
        (p, v) = ball_free_step_back(p, v);
        if !ball_in_free_air(p) {
            break;
        }
        post.push(p);
    }
    if !ball_in_free_air(a.pos) || !ball_in_free_air(b.pos) {
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
            let fraction = along / (dv_norm * DT);
            if !(ALONG_MIN..=ALONG_MAX).contains(&fraction) {
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
        // The last state before the hit: `t1` ticks after the first packet.
        _ => Some((first.0, first.1, pre[first.1])),
    }
}

/// A hit between two ball packets that the chain bridged: the last free-flight state before it.
#[derive(Debug, Clone)]
pub(super) struct BallHit {
    pub(super) frame_a: usize,
    pub(super) frame_b: usize,
    pub(super) ticks_after_a: usize,
    pub(super) ball_pos: [f32; 3],
}

/// One fresh car sample for the contact check: replay position, velocity, rotation and angular
/// velocity at a frame.
#[derive(Debug, Clone, Copy)]
pub(super) struct CarSample {
    pub(super) pos: glam::Vec3A,
    pub(super) vel: glam::Vec3A,
    pub(super) rot: Quat,
    pub(super) ang: glam::Vec3A,
}

/// Median over the bridged hits of the gap between the hitting car's hitbox and the ball at the
/// last state before the hit (the closest car, its fresh samples extrapolated to that tick;
/// negative: overlapping), for run starts as they are now. The extrapolation uses the car's
/// velocity, gravity when it is off the ground, and its angular velocity: over a few ticks the
/// error is a few UU. Returns the median and the number of hits used.
pub(super) fn median_hit_gap(
    hits: &[BallHit],
    ball_runs: &[RawRun],
    car_runs: &[((i32, usize), RawRun)],
    samples: &HashMap<(i32, usize, usize), CarSample>,
    hitboxes: &HashMap<(i32, usize), CarBodyConfig>,
) -> Option<(f32, usize)> {
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
        let ball = glam::Vec3A::from(hit.ball_pos);
        let mut best: Option<f32> = None;
        for (key, car_run) in car_runs {
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
            let (Some(sample), Some(config)) =
                (samples.get(&(key.0, key.1, frame)), hitboxes.get(key))
            else {
                continue;
            };
            let seconds = delta as f32 / 120.0;
            let airborne = sample.pos.z > 30.0;
            let gravity = if airborne { -650.0 } else { 0.0 };
            let pos = sample.pos
                + sample.vel * seconds
                + glam::Vec3A::new(0.0, 0.0, 0.5 * gravity * seconds * seconds);
            let rot = glam::Mat3A::from_quat(
                Quat::from_scaled_axis((sample.ang * seconds).into()) * sample.rot,
            );
            let local = rot.transpose() * (ball - pos) - config.hitbox_pos_offset;
            let q = local.abs() - config.hitbox_size * 0.5;
            let gap = q.max(glam::Vec3A::ZERO).length() + q.max_element().min(0.0) - 91.25;
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
    Some((gaps[gaps.len() / 2], gaps.len()))
}

/// Exact whole-tick chains. Elapsed ticks between chained packets are snapped to integers (pairs
/// more than 0.25 tick from one are rejected), so each packet's physical tick is `S = S0 + K` with
/// integer `K`. A packet was generated no later than its frame time and no earlier than the
/// previous frame's time, so on the integer timeline `tl(previous) <= S <= tl(frame)`; a run ends at
/// an unreliable pair or when no integer `S0` satisfies these bounds for every packet (a mistaken
/// interval shows up this way). Within the feasible starts, `S0` is the one that violates the
/// real-time window `0 <= T - S <= window` least (ties go to the middle). The assigned lag is
/// `tl(frame) - S`, an integer.
pub(super) fn chain_packet_lags_exact(
    observations: &ObservedReplay,
    packets: &[ChainPacket],
    valid: impl Fn(&ChainPacket, &ChainPacket) -> bool,
    fallback: impl Fn(&ChainPacket, &ChainPacket, i64, i64) -> Option<i64>,
    runs: &mut Vec<RawRun>,
) {
    let frames = &observations.frames;
    let first_time = f64::from(frames.first().map_or(0.0, |frame| frame.time));
    let real_tick = |frame: usize| (f64::from(frames[frame].time) - first_time) * 120.0;
    // The first frame has no previous frame, so its window is its own nominal period.
    let window = |frame: usize| {
        if frame == 0 {
            f64::from(frames[0].delta * 120.0).max(1.0)
        } else {
            real_tick(frame) - real_tick(frame - 1)
        }
    };
    let timeline = |frame: usize| real_tick(frame).round() as i64;
    // Integer bounds on S0 from one packet at cumulative interval K.
    let bounds = |frame: usize, k: i64| {
        let earliest = if frame == 0 {
            timeline(0) - window(0).round() as i64
        } else {
            timeline(frame - 1)
        };
        (earliest - k, timeline(frame) - k)
    };
    // (frame, K): physical tick relative to the first packet of the run.
    let mut run: Vec<(usize, i64)> = Vec::new();
    let (mut lo, mut hi) = (i64::MIN, i64::MAX);
    let finish = |run: &[(usize, i64)], lo: i64, hi: i64, runs: &mut Vec<RawRun>| {
        if run.len() < 2 || lo > hi {
            return;
        }
        let violation = |start: i64| -> f64 {
            run.iter()
                .map(|&(f, k)| {
                    let lag = real_tick(f) - (start + k) as f64;
                    (-lag).max(0.0) + (lag - window(f)).max(0.0)
                })
                .sum()
        };
        let scores: Vec<(i64, f64)> = (lo..=hi).map(|s| (s, violation(s))).collect();
        let best = scores.iter().map(|&(_, v)| v).fold(f64::INFINITY, f64::min);
        let tied: Vec<i64> = scores
            .iter()
            .filter(|&&(_, v)| v <= best + 1e-9)
            .map(|&(s, _)| s)
            .collect();
        let start = tied[tied.len() / 2];
        runs.push(RawRun {
            entries: run.to_vec(),
            lo,
            hi,
            start,
        });
    };
    for pair in packets.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        let interval = if valid(a, b) {
            implied_interval_ticks(a, b).and_then(|interval| {
                let snapped = interval.round();
                ((interval - snapped).abs() <= 0.25).then_some(snapped as i64)
            })
        } else {
            None
        };
        // Physical ticks between two packets lie between the frame times that bracket them.
        let interval = interval.or_else(|| {
            let d_lo = timeline(b.frame.saturating_sub(1)) - timeline(a.frame);
            let d_hi = timeline(b.frame) - timeline(a.frame.saturating_sub(1));
            fallback(a, b, d_lo, d_hi)
        });
        let Some(interval) = interval else {
            finish(&run, lo, hi, runs);
            run.clear();
            (lo, hi) = (i64::MIN, i64::MAX);
            continue;
        };
        if run.is_empty() {
            let (l, h) = bounds(a.frame, 0);
            run.push((a.frame, 0));
            (lo, hi) = (l, h);
        }
        let k_next = run.last().map_or(0, |entry| entry.1) + interval;
        let (l, h) = bounds(b.frame, k_next);
        let (new_lo, new_hi) = (lo.max(l), hi.min(h));
        if new_lo <= new_hi {
            run.push((b.frame, k_next));
            (lo, hi) = (new_lo, new_hi);
        } else {
            finish(&run, lo, hi, runs);
            let (la, ha) = bounds(a.frame, 0);
            let (lb, hb) = bounds(b.frame, interval);
            let (start_lo, start_hi) = (la.max(lb), ha.min(hb));
            if start_lo <= start_hi {
                run = vec![(a.frame, 0), (b.frame, interval)];
                (lo, hi) = (start_lo, start_hi);
            } else {
                run.clear();
                (lo, hi) = (i64::MIN, i64::MAX);
            }
        }
    }
    finish(&run, lo, hi, runs);
}

/// Offline inference of packet lags from chained ball and car motion. Uses packets after a frame,
/// so it is reconstruction, not prediction. Chains never bridge a withheld frame.
pub fn infer_packet_lags(observations: &ObservedReplay, options: &ConvertOptions) -> PacketLags {
    let frames = &observations.frames;
    let mut lags = PacketLags {
        ball: vec![None; frames.len()],
        cars: vec![None; frames.len()],
        car_actor: HashMap::new(),
        ..PacketLags::default()
    };
    let active = |frame: usize| {
        frames[frame]
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active")
    };
    let withheld = |frame: usize| {
        options
            .withheld_frames
            .as_ref()
            .is_some_and(|w| w.get(frame).copied().unwrap_or(false))
    };
    let bridge_ok = |a: usize, b: usize| {
        b > a
            && (a..=b).all(&active)
            && !withheld(a)
            && (a + 1..b).all(|f| !withheld(f))
            && !withheld(b)
    };
    let fresh = |body: &Body, frame: usize| -> Option<ChainPacket> {
        let position = body.position.as_ref().filter(|v| v.frame == frame)?;
        let velocity = body.linear_velocity.as_ref().filter(|v| v.frame == frame)?;
        Some(ChainPacket {
            frame,
            pos: position.value,
            vel: velocity.value,
        })
    };
    let norm = |v: [f32; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();

    // Ball: smooth motion between consecutive frames.
    let ball_packets: Vec<ChainPacket> = frames
        .iter()
        .filter_map(|frame| {
            frame
                .ball
                .as_ref()
                .and_then(|body| fresh(body, frame.index))
        })
        .collect();
    let mut ball_runs: Vec<RawRun> = Vec::new();
    let ball_hits: std::cell::RefCell<Vec<BallHit>> = std::cell::RefCell::new(Vec::new());
    chain_packet_lags_exact(
        observations,
        &ball_packets,
        |a, b| {
            // The implied interval is a displacement along the mean velocity: exact for constant
            // acceleration and biased only at second order in the turn angle, so smooth motion
            // (including rolling and low bounces) is usable; sharp direction or speed changes
            // (hits, bounces) are not.
            let (na, nb) = (norm(a.vel), norm(b.vel));
            let cosine = if na > 1.0 && nb > 1.0 {
                (0..3).map(|i| a.vel[i] * b.vel[i]).sum::<f32>() / (na * nb)
            } else {
                0.0
            };
            bridge_ok(a.frame, b.frame)
                && b.frame - a.frame <= 2
                && na.min(nb) > 300.0
                && cosine >= 0.97
                && (na - nb).abs() <= 0.25 * na.max(nb)
        },
        |a, b, d_lo, d_hi| {
            let (d, ticks_after_a, ball_pos) = (bridge_ok(a.frame, b.frame)
                && b.frame - a.frame <= 3)
                .then(|| ball_hit_interval_ticks(a, b, d_lo, d_hi))
                .flatten()?;
            ball_hits.borrow_mut().push(BallHit {
                frame_a: a.frame,
                frame_b: b.frame,
                ticks_after_a,
                ball_pos,
            });
            Some(d)
        },
        &mut ball_runs,
    );

    // Cars: fast, smooth motion between packets of one actor lifetime (dodges excluded).
    let mut chains: HashMap<(i32, usize), Vec<ChainPacket>> = HashMap::new();
    let mut dodge_frames: HashSet<(i32, usize, usize)> = HashSet::new();
    let mut samples: HashMap<(i32, usize, usize), CarSample> = HashMap::new();
    let mut hitboxes: HashMap<(i32, usize), CarBodyConfig> = HashMap::new();
    for frame in frames {
        for car in &frame.cars {
            if car
                .inputs
                .dodge_active_raw
                .as_ref()
                .is_some_and(|d| d.frame == frame.index && d.value % 2 == 1)
            {
                dodge_frames.insert((car.actor_id, car.actor_created_frame, frame.index));
            }
            if let Some(packet) = fresh(&car.body, frame.index) {
                chains
                    .entry((car.actor_id, car.actor_created_frame))
                    .or_default()
                    .push(packet);
                let key = (car.actor_id, car.actor_created_frame);
                // The hitbox the conversion uses (`use_loadout_hitboxes`), for the ball-car offset.
                hitboxes.entry(key).or_insert_with(|| {
                    car.body_product_id
                        .as_ref()
                        .filter(|_| options.use_loadout_hitboxes)
                        .and_then(|v| hitbox_for_body_product(v.value))
                        .map_or(CarBodyConfig::OCTANE, |(_, config)| config)
                });
                if let Some(rot) = car
                    .body
                    .rotation_xyzw
                    .as_ref()
                    .and_then(|r| quaternion(r.value))
                {
                    let ang = car
                        .body
                        .angular_velocity_replay_units
                        .as_ref()
                        .map_or([0.0; 3], |v| v.value);
                    samples.insert(
                        (car.actor_id, car.actor_created_frame, frame.index),
                        CarSample {
                            pos: glam::Vec3A::from(packet_pos(car, frame.index)),
                            vel: glam::Vec3A::from(packet_vel(car, frame.index)),
                            rot,
                            ang: glam::Vec3A::from(ang) * 0.01,
                        },
                    );
                }
            }
        }
    }
    let mut per_frame: Vec<Vec<f32>> = vec![Vec::new(); frames.len()];
    let mut car_runs: Vec<((i32, usize), RawRun)> = Vec::new();
    // Parallel to `car_runs`: which entries each run owns (`owned_entries`).
    let mut car_owned: Vec<Vec<bool>> = Vec::new();
    for ((actor, created), packets) in &chains {
        let mut runs: Vec<RawRun> = Vec::new();
        chain_packet_lags_exact(
            observations,
            packets,
            |a, b| {
                let (na, nb) = (norm(a.vel), norm(b.vel));
                let cosine = if na > 1.0 && nb > 1.0 {
                    (0..3).map(|i| a.vel[i] * b.vel[i]).sum::<f32>() / (na * nb)
                } else {
                    0.0
                };
                bridge_ok(a.frame, b.frame)
                    && b.frame - a.frame <= 8
                    && na.min(nb) > 350.0
                    && cosine >= 0.95
                    && (na - nb).abs() <= 0.4 * na.max(nb)
                    && (a.frame..=b.frame).all(|f| !dodge_frames.contains(&(*actor, *created, f)))
            },
            |_, _, _, _| None,
            &mut runs,
        );
        car_owned.extend(owned_entries(&runs));
        car_runs.extend(runs.into_iter().map(|run| ((*actor, *created), run)));
    }
    // A replay saved by the server (host) has every packet fresh at its frame's own tick: its chain
    // links equal the gaps of the frame timeline (99.5-99.7% on two host replays of the remote-client
    // games, 13-47% on 36 client replays of the corpus and 24-25% on the two remote-client replays),
    // while a client's packets have lags that jitter inside the window. Such a replay has no lag.
    let first_time = f64::from(frames.first().map_or(0.0, |frame| frame.time));
    let tl = |frame: usize| ((f64::from(frames[frame].time) - first_time) * 120.0).round() as i64;
    let (mut equal, mut total) = (0usize, 0usize);
    for run in ball_runs.iter().chain(car_runs.iter().map(|(_, r)| r)) {
        for pair in run.entries.windows(2) {
            total += 1;
            equal += usize::from(pair[1].1 - pair[0].1 == tl(pair[1].0) - tl(pair[0].0));
        }
    }
    if total >= 200 && equal as f64 >= 0.9 * total as f64 {
        let mut zero = zero_packet_lags(observations);
        zero.lag_free = true;
        return zero;
    }

    // The offset is estimated on the packets it is applied to: a packet shared by two runs counts once
    let ball_owned = owned_entries(&ball_runs);
    let owned_car_runs: Vec<((i32, usize), RawRun)> = car_runs
        .iter()
        .zip(&car_owned)
        .map(|((key, run), owned)| (*key, keep_owned(run, owned)))
        .collect();
    // Hits are looked up in the ball runs as they are (a hit pair whose first packet is the shared one of
    // two runs stays usable); ownership counts car packets only.
    let offset = estimate_ball_car_offset(
        &ball_hits.borrow(),
        &ball_runs,
        &owned_car_runs,
        &samples,
        &hitboxes,
    );
    lags.ball_car_offset = offset;
    lags.bridged_hits = ball_hits.borrow().len();
    if let Some(offset) = offset {
        place_ball_runs(&mut ball_runs, &owned_car_runs, offset);
    }
    // Exact runs: the lag of an entry is the frame's tick minus its physical tick.
    let first_time = f64::from(frames.first().map_or(0.0, |frame| frame.time));
    let timeline =
        |frame: usize| ((f64::from(frames[frame].time) - first_time) * 120.0).round() as i64;
    for (run, owned) in ball_runs.iter().zip(&ball_owned) {
        for (&(frame, k), &own) in run.entries.iter().zip(owned) {
            if own {
                lags.ball[frame] = Some((timeline(frame) - (run.start + k)).max(0) as f32);
            }
        }
    }
    for (((actor, created), run), owned) in car_runs.iter().zip(&car_owned) {
        let index = lags.car_runs.len();
        for (&(frame, k), &own) in run.entries.iter().zip(owned) {
            if !own {
                continue;
            }
            let lag = (timeline(frame) - (run.start + k)).max(0) as f32;
            per_frame[frame].push(lag);
            lags.car_actor.insert((*actor, *created, frame), lag);
            lags.car_run_of.insert((*actor, *created, frame), index);
        }
        lags.car_runs.push(CarRun {
            actor: *actor,
            created: *created,
            entries: run.entries.clone(),
            lo: run.lo,
            hi: run.hi,
            start: run.start,
        });
    }
    for (frame, values) in per_frame.iter_mut().enumerate() {
        if !values.is_empty() {
            values.sort_by(|a, b| a.total_cmp(b));
            lags.cars[frame] = Some(values[values.len() / 2]);
        }
    }
    lags
}
