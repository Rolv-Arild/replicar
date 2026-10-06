//! Update-tick inference: the tick whose server state each body update shows (docs/glossary.md,
//! "Update tick"). Offline: it uses later updates.
//!
//! An update is an exact server state from 0-4 ticks before the frame that carries it. Between two updates
//! of a body in smooth motion, the displacement along the mean velocity gives the elapsed ticks, which are
//! whole: chained, they fix every update of a run up to one shared start, and each frame bounds that start
//! (an update is no later than its frame and no earlier than the previous frame). Ball runs also continue
//! across hits, and are placed against the cars with a ball-minus-car offset estimated from the hits. A
//! replay saved by the server has every update at its frame's own tick and is detected as lag-free.

mod ball;
mod chains;

use std::collections::BTreeMap;

use glam::{Quat, Vec3A};
use replicar_format::FrameIndex;
use rocketsim::CarBodyConfig;

use crate::decode::{CarLife, GameState, NetworkBody, NetworkReplay};
use crate::hitbox::Hitbox;
use ball::{BallHit, ball_hit_interval_ticks};
use chains::{ChainPacket, RawRun, chain_runs, keep_owned, owned_entries};

/// How far before its frame's own tick each update's server state is, in ticks.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct UpdateTicks {
    /// The ball's update in each frame, when its tick was inferred.
    pub ball: Vec<Option<u32>>,
    /// Each car's update, by car life and frame, when its tick was inferred.
    pub cars: BTreeMap<(CarLife, FrameIndex), u32>,
    /// The median over the frame's inferred car updates.
    pub car_median: Vec<Option<u32>>,
    /// The ball-minus-car offset in ticks that placed the ball runs, when it could be estimated, and the
    /// number of ball hits the chains bridged.
    pub ball_car_offset: Option<f32>,
    pub bridged_hits: usize,
    /// The replay was saved by the server: every update is at its frame's own tick.
    pub lag_free: bool,
    /// The cars' runs of chained updates, ordered by car life and then by frame. Contact alignment moves
    /// whole runs.
    pub car_runs: Vec<CarRun>,
    /// The run (index in `car_runs`) of each car update that belongs to one.
    pub car_run_of: BTreeMap<(CarLife, FrameIndex), usize>,
}

/// One run of a car's chained updates on the integer tick timeline.
#[derive(Debug, Clone, PartialEq)]
pub struct CarRun {
    pub life: CarLife,
    /// (frame, K): the update's server tick is `start + K` on the replay tick scale.
    pub entries: Vec<(FrameIndex, i64)>,
    /// The feasible starts, and the one chosen.
    pub lo: i64,
    pub hi: i64,
    pub start: i64,
}

/// Which frames' updates a masked evaluation hides; every inference must refuse to look at them.
#[derive(Debug, Clone, Copy, Default)]
pub struct Withheld<'a>(pub Option<&'a [bool]>);

impl Withheld<'_> {
    fn contains(self, frame: usize) -> bool {
        self.0
            .is_some_and(|w| w.get(frame).copied().unwrap_or(false))
    }
}

/// A unit quaternion from replay x, y, z, w, or `None` when it is not finite or degenerate.
#[must_use]
pub fn quaternion(xyzw: [f32; 4]) -> Option<Quat> {
    let q = Quat::from_xyzw(xyzw[0], xyzw[1], xyzw[2], xyzw[3]);
    (q.is_finite() && q.length_squared() > 1e-8).then(|| q.normalize())
}

/// Every update at its frame's own tick: for a replay recorded without network delay.
#[must_use]
pub fn lag_free(network: &NetworkReplay) -> UpdateTicks {
    let frames = &network.frames;
    let mut ticks = UpdateTicks {
        ball: vec![None; frames.len()],
        car_median: vec![None; frames.len()],
        ..UpdateTicks::default()
    };
    for frame in frames {
        let f = frame.index;
        if updated_at(frame.ball.as_ref(), f) {
            ticks.ball[f.get()] = Some(0);
        }
        for car in &frame.cars {
            if updated_at(Some(&car.body), f) {
                ticks.car_median[f.get()] = Some(0);
                ticks.cars.insert((car.life, f), 0);
            }
        }
    }
    ticks
}

fn updated_at(body: Option<&NetworkBody>, frame: FrameIndex) -> bool {
    body.and_then(|b| b.position.as_ref())
        .is_some_and(|p| p.frame == frame)
}

/// The update of a body at a frame with both position and velocity, as a chain link.
fn chain_packet(body: &NetworkBody, frame: FrameIndex) -> Option<ChainPacket> {
    let position = body.position.as_ref().filter(|v| v.frame == frame)?;
    let velocity = body.linear_velocity.as_ref().filter(|v| v.frame == frame)?;
    Some(ChainPacket {
        frame: frame.get(),
        pos: position.value,
        vel: velocity.value,
    })
}

fn norm(v: [f32; 3]) -> f32 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn cosine(a: [f32; 3], b: [f32; 3]) -> f32 {
    let (na, nb) = (norm(a), norm(b));
    if na > 1.0 && nb > 1.0 {
        (0..3).map(|i| a[i] * b[i]).sum::<f32>() / (na * nb)
    } else {
        0.0
    }
}

/// One car update for the hit gap: position, velocity, rotation and angular velocity (rad/s).
#[derive(Debug, Clone, Copy)]
struct CarSample {
    pos: Vec3A,
    vel: Vec3A,
    rot: Quat,
    ang: Vec3A,
}

/// Infer every update's tick from the chained motion of the ball and the cars. `loadout_hitboxes` selects
/// each car's hitbox from its body (else Octane), for the ball-car offset. Chains never bridge a withheld
/// frame.
#[must_use]
pub fn infer(network: &NetworkReplay, loadout_hitboxes: bool, withheld: Withheld) -> UpdateTicks {
    let frames = &network.frames;
    let active = |f: usize| {
        frames[f]
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == GameState::Active)
    };
    let bridge_ok = |a: usize, b: usize| {
        b > a
            && (a..=b).all(active)
            && !withheld.contains(a)
            && (a + 1..b).all(|f| !withheld.contains(f))
            && !withheld.contains(b)
    };
    let timeline = chains::Timeline::new(frames);

    // The ball: smooth motion between consecutive frames, and hits bridged by the free-flight paths.
    let ball_packets: Vec<ChainPacket> = frames
        .iter()
        .filter_map(|frame| {
            frame
                .ball
                .as_ref()
                .and_then(|body| chain_packet(body, frame.index))
        })
        .collect();
    let mut ball_hits: Vec<BallHit> = Vec::new();
    let mut ball_runs = chain_runs(
        &timeline,
        &ball_packets,
        |a, b| {
            // The implied interval is a displacement along the mean velocity: exact for constant
            // acceleration and biased only at second order in the turn angle, so smooth motion (rolling and
            // low bounces included) is usable, sharp changes (hits, bounces) are not.
            let (na, nb) = (norm(a.vel), norm(b.vel));
            bridge_ok(a.frame, b.frame)
                && b.frame - a.frame <= 2
                && na.min(nb) > 300.0
                && cosine(a.vel, b.vel) >= 0.97
                && (na - nb).abs() <= 0.25 * na.max(nb)
        },
        |a, b, d_lo, d_hi| {
            if !(bridge_ok(a.frame, b.frame) && b.frame - a.frame <= 3) {
                return None;
            }
            let (d, ticks_after_a, ball_pos) = ball_hit_interval_ticks(a, b, d_lo, d_hi)?;
            ball_hits.push(BallHit {
                frame_a: a.frame,
                frame_b: b.frame,
                ticks_after_a,
                ball_pos,
            });
            Some(d)
        },
    );

    // The cars: fast, smooth motion between updates of one car life, dodges excluded.
    let mut chains: BTreeMap<CarLife, Vec<ChainPacket>> = BTreeMap::new();
    let mut dodging: BTreeMap<CarLife, Vec<usize>> = BTreeMap::new();
    let mut samples: BTreeMap<(CarLife, usize), CarSample> = BTreeMap::new();
    let mut hitboxes: BTreeMap<CarLife, CarBodyConfig> = BTreeMap::new();
    for frame in frames {
        let f = frame.index;
        for car in &frame.cars {
            if car
                .inputs
                .dodge_active_raw
                .as_ref()
                .is_some_and(|d| d.frame == f && d.value % 2 == 1)
            {
                dodging.entry(car.life).or_default().push(f.get());
            }
            let Some(packet) = chain_packet(&car.body, f) else {
                continue;
            };
            hitboxes.entry(car.life).or_insert_with(|| {
                car.body_product_id
                    .as_ref()
                    .filter(|_| loadout_hitboxes)
                    .and_then(|body| Hitbox::of_body(body.value))
                    .unwrap_or_default()
                    .config()
            });
            if let Some(rot) = car.body.rotation.as_ref().and_then(|r| quaternion(r.value)) {
                let ang = car
                    .body
                    .angular_velocity_raw
                    .as_ref()
                    .map_or([0.0; 3], |v| v.value);
                samples.insert(
                    (car.life, f.get()),
                    CarSample {
                        pos: Vec3A::from(packet.pos),
                        vel: Vec3A::from(packet.vel),
                        rot,
                        ang: Vec3A::from(ang) * 0.01,
                    },
                );
            }
            chains.entry(car.life).or_default().push(packet);
        }
    }
    let mut car_runs: Vec<(CarLife, RawRun)> = Vec::new();
    let mut car_owned: Vec<Vec<bool>> = Vec::new();
    for (life, packets) in &chains {
        let dodges = dodging.get(life).map_or(&[][..], Vec::as_slice);
        let runs = chain_runs(
            &timeline,
            packets,
            |a, b| {
                let (na, nb) = (norm(a.vel), norm(b.vel));
                bridge_ok(a.frame, b.frame)
                    && b.frame - a.frame <= 8
                    && na.min(nb) > 350.0
                    && cosine(a.vel, b.vel) >= 0.95
                    && (na - nb).abs() <= 0.4 * na.max(nb)
                    && !dodges.iter().any(|f| (a.frame..=b.frame).contains(f))
            },
            |_, _, _, _| None,
        );
        car_owned.extend(owned_entries(&runs));
        car_runs.extend(runs.into_iter().map(|run| (*life, run)));
    }

    // A replay saved by the server has every update at its frame's own tick, so its chain links equal the
    // gaps of the frame timeline (99.5-99.7% on two host replays; 13-47% on 36 client replays of the corpus).
    let (mut equal, mut total) = (0usize, 0usize);
    for run in ball_runs.iter().chain(car_runs.iter().map(|(_, run)| run)) {
        for pair in run.entries.windows(2) {
            total += 1;
            equal += usize::from(
                pair[1].1 - pair[0].1 == timeline.tick(pair[1].0) - timeline.tick(pair[0].0),
            );
        }
    }
    if total >= 200 && equal as f64 >= 0.9 * total as f64 {
        let mut ticks = lag_free(network);
        ticks.lag_free = true;
        return ticks;
    }

    // The offset is estimated on the updates it is applied to: an update shared by two runs counts once.
    let ball_owned = owned_entries(&ball_runs);
    let owned_car_runs: Vec<(CarLife, RawRun)> = car_runs
        .iter()
        .zip(&car_owned)
        .map(|((life, run), owned)| (*life, keep_owned(run, owned)))
        .collect();
    let offset = ball::estimate_ball_car_offset(
        &ball_hits,
        &ball_runs,
        &owned_car_runs,
        &samples,
        &hitboxes,
    );
    if let Some(offset) = offset {
        ball::place_ball_runs(&mut ball_runs, &owned_car_runs, offset);
    }

    let mut ticks = UpdateTicks {
        ball: vec![None; frames.len()],
        car_median: vec![None; frames.len()],
        ball_car_offset: offset,
        bridged_hits: ball_hits.len(),
        ..UpdateTicks::default()
    };
    let before_frame =
        |frame: usize, server_tick: i64| (timeline.tick(frame) - server_tick).max(0) as u32;
    for (run, owned) in ball_runs.iter().zip(&ball_owned) {
        for (&(frame, k), &own) in run.entries.iter().zip(owned) {
            if own {
                ticks.ball[frame] = Some(before_frame(frame, run.start + k));
            }
        }
    }
    let mut per_frame: Vec<Vec<u32>> = vec![Vec::new(); frames.len()];
    for ((life, run), owned) in car_runs.iter().zip(&car_owned) {
        let index = ticks.car_runs.len();
        for (&(frame, k), &own) in run.entries.iter().zip(owned) {
            if own {
                let lag = before_frame(frame, run.start + k);
                per_frame[frame].push(lag);
                let key = (*life, FrameIndex(frame as u32));
                ticks.cars.insert(key, lag);
                ticks.car_run_of.insert(key, index);
            }
        }
        ticks.car_runs.push(CarRun {
            life: *life,
            entries: run
                .entries
                .iter()
                .map(|&(f, k)| (FrameIndex(f as u32), k))
                .collect(),
            lo: run.lo,
            hi: run.hi,
            start: run.start,
        });
    }
    for (frame, mut values) in per_frame.into_iter().enumerate() {
        if !values.is_empty() {
            values.sort_unstable();
            ticks.car_median[frame] = Some(values[values.len() / 2]);
        }
    }
    ticks
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testkit::{frame, moving_body, replay};

    /// Updates are exact whole-tick server states from up to a frame before their frame; the frame times
    /// jitter a little, which pins the absolute tick.
    const TRUE_TICKS_BEFORE: [i64; 16] = [2, 0, 3, 1, 3, 0, 2, 1, 3, 0, 1, 2, 3, 0, 2, 1];
    const VELOCITY: [f32; 3] = [1200.0, 300.0, 0.0];

    fn frame_tick(index: usize) -> f32 {
        4.0 * index as f32 + 0.3 * (index % 3) as f32
    }

    fn ball_replay() -> NetworkReplay {
        replay(
            TRUE_TICKS_BEFORE
                .iter()
                .enumerate()
                .map(|(index, before)| {
                    let server_tick = 4.0 * index as f32 - *before as f32;
                    let position = [
                        VELOCITY[0] * server_tick / 120.0,
                        VELOCITY[1] * server_tick / 120.0,
                        400.0,
                    ];
                    let mut f = frame(index as u32, frame_tick(index) / 120.0, 4.0 / 120.0);
                    f.ball = Some(moving_body(index as u32, position, VELOCITY));
                    f
                })
                .collect(),
        )
    }

    #[test]
    fn whole_tick_updates_get_their_exact_tick() {
        let ticks = infer(&ball_replay(), true, Withheld::default());
        for (index, truth) in TRUE_TICKS_BEFORE.iter().enumerate() {
            let expected = frame_tick(index).round() as i64 - (4 * index as i64 - truth);
            assert_eq!(
                ticks.ball[index].map(i64::from),
                Some(expected),
                "frame {index}"
            );
        }
    }

    /// Half-tick offsets are not whole-tick physics: the exact chains reject them.
    #[test]
    fn fractional_ticks_are_rejected() {
        let mut network = ball_replay();
        for (index, f) in network.frames.iter_mut().enumerate() {
            let position = f.ball.as_mut().unwrap().position.as_mut().unwrap();
            position.value[0] += 0.5 * (index % 2) as f32 * VELOCITY[0] / 120.0;
        }
        let ticks = infer(&network, true, Withheld::default());
        assert!(ticks.ball.iter().all(Option::is_none));
    }

    /// A withheld frame breaks the chain: no pair bridges it.
    #[test]
    fn no_chain_bridges_a_withheld_frame() {
        let mut network = ball_replay();
        network.frames[5].ball = network.frames[4].ball.clone();
        let withheld: Vec<bool> = (0..TRUE_TICKS_BEFORE.len()).map(|i| i == 5).collect();
        let ticks = infer(&network, true, Withheld(Some(&withheld)));
        assert!(ticks.ball[5].is_none());
        assert!(ticks.ball[4].is_some() && ticks.ball[6].is_some());
    }
}
