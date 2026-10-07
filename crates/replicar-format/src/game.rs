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
        assert_eq!(Period::from_name("overtime"), Some(Period::Overtime));
        assert_eq!(Period::from_name("halftime"), None);
    }
}
