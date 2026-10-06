//! Exact whole-tick chains of a body's updates.

use crate::decode::NetworkFrame;

/// One update of a chained body: its frame, position and velocity.
pub(super) struct ChainPacket {
    pub(super) frame: usize,
    pub(super) pos: [f32; 3],
    pub(super) vel: [f32; 3],
}

/// One run of chained updates on the integer tick timeline: `(frame, K)` entries (each update's server tick
/// relative to the run's first), the feasible integer starts `lo..=hi` (the server tick of the entry with
/// `K = 0`), and the start chosen.
#[derive(Debug, Clone)]
pub(super) struct RawRun {
    pub(super) entries: Vec<(usize, i64)>,
    pub(super) lo: i64,
    pub(super) hi: i64,
    pub(super) start: i64,
}

/// The frames' times on the 120 Hz scale.
pub(super) struct Timeline<'a> {
    frames: &'a [NetworkFrame],
    first_time: f64,
}

impl<'a> Timeline<'a> {
    pub(super) fn new(frames: &'a [NetworkFrame]) -> Self {
        Self {
            frames,
            first_time: f64::from(frames.first().map_or(0.0, |frame| frame.time)),
        }
    }

    /// A frame's time in ticks since the first frame, unrounded.
    fn real_tick(&self, frame: usize) -> f64 {
        (f64::from(self.frames[frame].time) - self.first_time) * 120.0
    }

    /// A frame's replay tick: its time in ticks since the first frame, rounded.
    pub(super) fn tick(&self, frame: usize) -> i64 {
        self.real_tick(frame).round() as i64
    }

    /// The frame's window in ticks: the time since the previous frame (the first frame's own delta).
    fn window(&self, frame: usize) -> f64 {
        if frame == 0 {
            f64::from(self.frames[0].delta * 120.0).max(1.0)
        } else {
            self.real_tick(frame) - self.real_tick(frame - 1)
        }
    }

    /// The integer bounds on a run's start from one update at cumulative interval `k`: the update is no
    /// earlier than the previous frame and no later than its own.
    fn bounds(&self, frame: usize, k: i64) -> (i64, i64) {
        let earliest = if frame == 0 {
            self.tick(0) - self.window(0).round() as i64
        } else {
            self.tick(frame - 1)
        };
        (earliest - k, self.tick(frame) - k)
    }
}

/// Ticks of server time between two updates, from the displacement along their mean velocity.
fn implied_interval_ticks(a: &ChainPacket, b: &ChainPacket) -> Option<f32> {
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

/// The runs of a body's updates. Elapsed ticks between valid pairs are snapped to integers (a pair more than
/// 0.25 tick from one is rejected), so each update's server tick is `S0 + K` with integer `K`; `fallback`
/// may supply the interval of a pair that is not smooth (`d_lo..=d_hi` bound it by the frame times). A run
/// ends at a pair without an interval, or when no integer `S0` fits every update's bounds (a mistaken
/// interval shows this way). Of the feasible starts, the one that leaves each update least outside its
/// frame's real-time window wins; ties go to the middle.
pub(super) fn chain_runs(
    timeline: &Timeline,
    packets: &[ChainPacket],
    valid: impl Fn(&ChainPacket, &ChainPacket) -> bool,
    mut fallback: impl FnMut(&ChainPacket, &ChainPacket, i64, i64) -> Option<i64>,
) -> Vec<RawRun> {
    let mut runs = Vec::new();
    let finish = |run: &[(usize, i64)], lo: i64, hi: i64, runs: &mut Vec<RawRun>| {
        if run.len() < 2 || lo > hi {
            return;
        }
        let violation = |start: i64| -> f64 {
            run.iter()
                .map(|&(f, k)| {
                    let lag = timeline.real_tick(f) - (start + k) as f64;
                    (-lag).max(0.0) + (lag - timeline.window(f)).max(0.0)
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
        runs.push(RawRun {
            entries: run.to_vec(),
            lo,
            hi,
            start: tied[tied.len() / 2],
        });
    };
    let mut run: Vec<(usize, i64)> = Vec::new();
    let (mut lo, mut hi) = (i64::MIN, i64::MAX);
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
        // The server ticks between two updates lie between the frame times that bracket them.
        let interval = interval.or_else(|| {
            let d_lo = timeline.tick(b.frame.saturating_sub(1)) - timeline.tick(a.frame);
            let d_hi = timeline.tick(b.frame) - timeline.tick(a.frame.saturating_sub(1));
            fallback(a, b, d_lo, d_hi)
        });
        let Some(interval) = interval else {
            finish(&run, lo, hi, &mut runs);
            run.clear();
            (lo, hi) = (i64::MIN, i64::MAX);
            continue;
        };
        if run.is_empty() {
            (lo, hi) = timeline.bounds(a.frame, 0);
            run.push((a.frame, 0));
        }
        let k_next = run.last().map_or(0, |entry| entry.1) + interval;
        let (l, h) = timeline.bounds(b.frame, k_next);
        let (new_lo, new_hi) = (lo.max(l), hi.min(h));
        if new_lo <= new_hi {
            run.push((b.frame, k_next));
            (lo, hi) = (new_lo, new_hi);
        } else {
            finish(&run, lo, hi, &mut runs);
            let (la, ha) = timeline.bounds(a.frame, 0);
            let (lb, hb) = timeline.bounds(b.frame, interval);
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
    finish(&run, lo, hi, &mut runs);
    runs
}

/// Per run and entry: whether the run owns that update's tick. Two consecutive runs of one body can share an
/// update (the last of the first and the first of the second); the earlier run owns it (RESULTS.md, "Packet
/// shared by two lag runs").
pub(super) fn owned_entries(runs: &[RawRun]) -> Vec<Vec<bool>> {
    let mut owned: Vec<Vec<bool>> = runs
        .iter()
        .map(|run| vec![true; run.entries.len()])
        .collect();
    for i in 1..runs.len() {
        if let (Some(last), Some(first)) = (runs[i - 1].entries.last(), runs[i].entries.first())
            && last.0 == first.0
        {
            owned[i][0] = false;
        }
    }
    owned
}

/// A copy of `run` with only the entries it owns.
pub(super) fn keep_owned(run: &RawRun, owned: &[bool]) -> RawRun {
    let mut run = run.clone();
    let mut flags = owned.iter();
    run.entries
        .retain(|_| flags.next().copied().unwrap_or(true));
    run
}
