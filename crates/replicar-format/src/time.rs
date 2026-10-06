//! Frames and ticks (docs/glossary.md, "Time").
//!
//! Three scales that must not be mixed, each its own type: a replay frame (`FrameIndex`), the replay's
//! 120 Hz timeline (`ReplayTick`) and RocketSim's own tick count (`SimTick`), which advances only while the
//! match is simulated and so falls behind the replay tick at every pause.

use std::fmt;

/// Ticks per second, RocketSim's physics rate and the server's.
pub const TICKS_PER_SECOND: u32 = 120;

/// A replay frame: its index in the replay, counted from 0 (`frame`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct FrameIndex(pub u32);

/// A point on the replay's 120 Hz timeline: replay time since the first frame, in ticks, rounded
/// (`replay_tick`). Pauses and goal replays included.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct ReplayTick(pub u32);

/// RocketSim's arena tick count (`sim_tick`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct SimTick(pub u64);

impl FrameIndex {
    /// The index as a `usize`, for indexing per-frame vectors.
    #[must_use]
    pub fn get(self) -> usize {
        self.0 as usize
    }
}

impl ReplayTick {
    /// The tick of a moment `seconds` after the first frame, rounded to the nearest tick (half away from
    /// zero); a negative or non-finite elapsed time gives `None`.
    #[must_use]
    pub fn from_elapsed_seconds(seconds: f64) -> Option<Self> {
        if !seconds.is_finite() || seconds < 0.0 {
            return None;
        }
        let ticks = (seconds * f64::from(TICKS_PER_SECOND)).round();
        (ticks <= f64::from(u32::MAX)).then_some(Self(ticks as u32))
    }

    /// Ticks from `earlier` to `self`, or 0 when `earlier` is not earlier.
    #[must_use]
    pub fn ticks_since(self, earlier: Self) -> u32 {
        self.0.saturating_sub(earlier.0)
    }
}

impl fmt::Display for FrameIndex {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "frame {}", self.0)
    }
}

impl fmt::Display for ReplayTick {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "replay tick {}", self.0)
    }
}

impl fmt::Display for SimTick {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "sim tick {}", self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn elapsed_seconds_round_to_the_nearest_tick() {
        assert_eq!(ReplayTick::from_elapsed_seconds(0.0), Some(ReplayTick(0)));
        assert_eq!(ReplayTick::from_elapsed_seconds(1.0), Some(ReplayTick(120)));
        assert_eq!(
            ReplayTick::from_elapsed_seconds(1.0 / 60.0),
            Some(ReplayTick(2))
        );
        // Half a tick rounds away from zero, as `f64::round` does.
        assert_eq!(
            ReplayTick::from_elapsed_seconds(0.5 / 120.0),
            Some(ReplayTick(1))
        );
        assert_eq!(ReplayTick::from_elapsed_seconds(-0.1), None);
        assert_eq!(ReplayTick::from_elapsed_seconds(f64::NAN), None);
    }

    #[test]
    fn ticks_since_never_goes_negative() {
        assert_eq!(ReplayTick(10).ticks_since(ReplayTick(4)), 6);
        assert_eq!(ReplayTick(4).ticks_since(ReplayTick(10)), 0);
    }
}
