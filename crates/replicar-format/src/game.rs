//! The game's named states as the file stores them (docs/glossary.md, "Clock phase", "Period", "Car status",
//! "Future group"). Each is a dictionary-encoded string column; `name` is the string.

/// `regulation` or `overtime`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Period {
    Regulation,
    Overtime,
}

/// Where the match clock is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ClockPhase {
    Pregame,
    Countdown,
    /// Cars released, the clock held until the first touch.
    Kickoff,
    Running,
    /// Regulation at 0, waiting for the ball to touch the ground.
    Expired,
    /// The ball touched the ground after expiry: the game ends or goes to overtime.
    Decided,
    /// Goal celebration and goal replay.
    GoalPause,
    Other,
}

/// How a play segment ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SegmentEnd {
    BlueGoal,
    OrangeGoal,
    /// Regulation ran out: the segment's last frame has the clock expired or decided.
    TimeExpired,
    /// The replay stops during play, so the end is unknown.
    ReplayEnded,
    /// Play stopped some other way.
    Other,
}

/// A match statistic the game counts per player (docs/glossary.md, "Stat event"). The ones from `EpicSave` on
/// are counted only by the builds since September 2026.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum StatKind {
    Goal,
    Assist,
    Save,
    Shot,
    Demolition,
    EpicSave,
    Clear,
    Center,
    AerialHit,
    FirstTouch,
    CrossbarHit,
    BicycleHit,
    JuggleHit,
    FlipReset,
    Demolished,
}

/// What set a car's pitch, yaw and roll in a frame (docs/glossary.md, "Controls source"). The replay does not carry
/// them: every value but `Unset` is inferred.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AirControlSource {
    /// Not inferred: 0. The car was on the ground at its last update (it may have left it since).
    Unset,
    /// In the air without a fit: the steer as yaw (as roll with the handbrake), pitch 0.
    Steer,
    /// The air controls solved from the car's last two updates, decayed with the time since.
    Persisted,
    /// The air controls solved for the interval to the car's next update (future-derived).
    Lookahead,
    /// A per-tick air schedule solved to the car's next update (future-derived).
    Schedule,
    /// A jump or dodge press from this frame's action counters, with its direction.
    Press,
    /// A fitted dodge press: the jump release before it, the press with its direction, then the flip cancel.
    Dodge,
    /// A flipping car's fitted flip cancel as the pitch; yaw and roll as `Steer` or `Persisted`/`Lookahead`.
    FlipCancel,
}

/// What set a car's throttle, steer, handbrake and boost in a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GroundControlSource {
    /// The replay's values, from the tick replicar infers they took effect.
    Network,
    /// A fitted ground schedule (control timing or a jump) to the car's next update (future-derived).
    Schedule,
    /// A fitted dodge press: the controls of the update it was planned at.
    Dodge,
}

/// A player's car in a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CarStatus {
    /// The player has no car in this frame.
    Absent,
    Active,
    /// The car has had no update yet: shown at the announced spawn and kept out of collisions.
    Spawning,
    Demolished,
}

macro_rules! names {
    ($type:ty { $($variant:ident => $name:literal),* $(,)? }) => {
        impl $type {
            /// Every value, in the order of the file's dictionary.
            pub const ALL: &'static [Self] = &[$(Self::$variant),*];

            /// The value as the file stores it.
            #[must_use]
            pub fn name(self) -> &'static str {
                match self {
                    $(Self::$variant => $name),*
                }
            }

            /// The value of a stored name.
            #[must_use]
            pub fn from_name(name: &str) -> Option<Self> {
                match name {
                    $($name => Some(Self::$variant),)*
                    _ => None,
                }
            }
        }
    };
}

names!(Period { Regulation => "regulation", Overtime => "overtime" });
names!(ClockPhase {
    Pregame => "pregame",
    Countdown => "countdown",
    Kickoff => "kickoff",
    Running => "running",
    Expired => "expired",
    Decided => "decided",
    GoalPause => "goal_pause",
    Other => "other",
});
names!(SegmentEnd {
    BlueGoal => "blue_goal",
    OrangeGoal => "orange_goal",
    TimeExpired => "time_expired",
    ReplayEnded => "replay_ended",
    Other => "other",
});
names!(StatKind {
    Goal => "goal",
    Assist => "assist",
    Save => "save",
    Shot => "shot",
    Demolition => "demolition",
    EpicSave => "epic_save",
    Clear => "clear",
    Center => "center",
    AerialHit => "aerial_hit",
    FirstTouch => "first_touch",
    CrossbarHit => "crossbar_hit",
    BicycleHit => "bicycle_hit",
    JuggleHit => "juggle_hit",
    FlipReset => "flip_reset",
    Demolished => "demolished",
});
names!(AirControlSource {
    Unset => "none",
    Steer => "steer",
    Persisted => "persisted",
    Lookahead => "lookahead",
    Schedule => "schedule",
    Press => "press",
    Dodge => "dodge",
    FlipCancel => "flip_cancel",
});
names!(GroundControlSource {
    Network => "network",
    Schedule => "schedule",
    Dodge => "dodge",
});
names!(CarStatus {
    Absent => "absent",
    Active => "active",
    Spawning => "spawning",
    Demolished => "demolished",
});

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_name_reads_back() {
        for &phase in ClockPhase::ALL {
            assert_eq!(ClockPhase::from_name(phase.name()), Some(phase));
        }
        for &end in SegmentEnd::ALL {
            assert_eq!(SegmentEnd::from_name(end.name()), Some(end));
        }
        for &status in CarStatus::ALL {
            assert_eq!(CarStatus::from_name(status.name()), Some(status));
        }
        for &source in AirControlSource::ALL {
            assert_eq!(AirControlSource::from_name(source.name()), Some(source));
        }
        for &source in GroundControlSource::ALL {
            assert_eq!(GroundControlSource::from_name(source.name()), Some(source));
        }
        assert_eq!(Period::from_name("overtime"), Some(Period::Overtime));
        assert_eq!(Period::from_name("halftime"), None);
    }
}
