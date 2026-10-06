//! Players and teams (docs/glossary.md, "Bodies and identity").

use std::fmt;

/// A player's position in every per-player column (`player`): fixed for the whole replay, so
/// `car_position[frame, player]` is that player's car whichever car it currently drives.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct PlayerIndex(pub u8);

impl PlayerIndex {
    /// The index as a `usize`, for indexing per-player vectors.
    #[must_use]
    pub fn get(self) -> usize {
        usize::from(self.0)
    }
}

impl fmt::Display for PlayerIndex {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "player {}", self.0)
    }
}

/// Team 0 is blue, team 1 orange.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Team {
    Blue,
    Orange,
}

impl Team {
    /// The team of a replay team number: 0 blue, 1 orange, anything else `None`.
    #[must_use]
    pub fn from_number(number: u8) -> Option<Self> {
        match number {
            0 => Some(Self::Blue),
            1 => Some(Self::Orange),
            _ => None,
        }
    }

    /// 0 for blue, 1 for orange.
    #[must_use]
    pub fn number(self) -> u8 {
        match self {
            Self::Blue => 0,
            Self::Orange => 1,
        }
    }

    /// The other team.
    #[must_use]
    pub fn opponent(self) -> Self {
        match self {
            Self::Blue => Self::Orange,
            Self::Orange => Self::Blue,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn team_numbers_round_trip() {
        for team in [Team::Blue, Team::Orange] {
            assert_eq!(Team::from_number(team.number()), Some(team));
            assert_eq!(team.opponent().opponent(), team);
        }
        assert_eq!(Team::from_number(2), None);
    }
}
