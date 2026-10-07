//! The masked evaluation's schedule and masking (v1: `evaluate_corpus.rs`, `MaskSchedule` and
//! `masked_observations`): in every block of 100 frames, four consecutive frames have their body updates,
//! boost and pad records withheld (replaced by the previous frame's), and the conversion is told which frames
//! those are, so that no inference reads them.

use replicar::decode::NetworkReplay;

/// Which frames are withheld, and at which horizon (1-4 frames into the window).
#[derive(Debug, Clone, Copy)]
pub struct MaskSchedule {
    /// `None`: every block's window starts at its frame 1. Else a window start drawn from the seed, the
    /// replay's hash and the block.
    pub seed: Option<u64>,
    /// The first 16 hex digits of the replay's SHA-256, as a number.
    pub replay_hash: u64,
}

impl MaskSchedule {
    /// The horizon of frame `index` (1-4), or `None` when it is not withheld.
    #[must_use]
    pub fn horizon(self, index: usize) -> Option<usize> {
        let offset = index % 100;
        let start = if let Some(seed) = self.seed {
            let mut value = seed ^ self.replay_hash ^ (index / 100) as u64;
            value = value.wrapping_add(0x9e37_79b9_7f4a_7c15);
            value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
            value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
            value ^= value >> 31;
            1 + (value % 95) as usize
        } else {
            1
        };
        (start..start + 4)
            .contains(&offset)
            .then(|| offset - start + 1)
    }

    /// Whether each of `frames` frames is withheld.
    #[must_use]
    pub fn withheld(self, frames: usize) -> Vec<bool> {
        (0..frames).map(|i| self.horizon(i).is_some()).collect()
    }
}

/// The replay with the withheld frames' ball and car bodies, boost and pad records replaced by the previous
/// frame's (the same car life's).
#[must_use]
pub fn mask(network: &NetworkReplay, schedule: MaskSchedule) -> NetworkReplay {
    let mut masked = network.clone();
    for index in 1..masked.frames.len() {
        if schedule.horizon(index).is_none() {
            continue;
        }
        let previous = masked.frames[index - 1].clone();
        let frame = &mut masked.frames[index];
        frame.pad_records.clear();
        if let Some(ball) = previous.ball {
            frame.ball = Some(ball);
        }
        for car in &mut frame.cars {
            if let Some(prior) = previous.cars.iter().find(|prior| prior.life == car.life) {
                car.body = prior.body.clone();
                car.boost = prior.boost.clone();
                car.boost_raw = prior.boost_raw.clone();
            }
        }
    }
    masked
}

/// v1's masking, copied from `evaluate_corpus.rs` (a binary, so not importable): the oracle for `mask`.
#[must_use]
pub fn mask_v1(
    original: &replicar_v1::observations::ObservedReplay,
    schedule: MaskSchedule,
) -> replicar_v1::observations::ObservedReplay {
    let mut masked = original.clone();
    for index in 1..masked.frames.len() {
        if schedule.horizon(index).is_none() {
            continue;
        }
        let previous = masked.frames[index - 1].clone();
        let frame = &mut masked.frames[index];
        frame.pad_pickups.clear();
        if let Some(previous_ball) = previous.ball {
            frame.ball = Some(previous_ball);
        }
        for car in &mut frame.cars {
            if let Some(prior) = previous.cars.iter().find(|prior| {
                prior.actor_id == car.actor_id
                    && prior.actor_created_frame == car.actor_created_frame
            }) {
                car.body = prior.body.clone();
                car.boost = prior.boost.clone();
                car.boost_raw = prior.boost_raw.clone();
            }
        }
    }
    masked
}
