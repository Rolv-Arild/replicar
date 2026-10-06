//! Hitboxes: RocketSim's car body configurations, and which one a car body uses (docs/glossary.md,
//! "Hitbox", "Car body").

use std::sync::OnceLock;

use rocketsim::CarBodyConfig;

/// One of RocketSim's car body configurations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Hitbox {
    #[default]
    Octane,
    Dominus,
    Plank,
    Breakout,
    Hybrid,
    Merc,
    Psyclops,
}

impl Hitbox {
    /// The hitbox of its roster name (`octane`, `dominus`, ...).
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        Some(match name {
            "octane" => Self::Octane,
            "dominus" => Self::Dominus,
            "plank" => Self::Plank,
            "breakout" => Self::Breakout,
            "hybrid" => Self::Hybrid,
            "merc" => Self::Merc,
            "psyclops" => Self::Psyclops,
            _ => return None,
        })
    }

    /// The roster name.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Octane => "octane",
            Self::Dominus => "dominus",
            Self::Plank => "plank",
            Self::Breakout => "breakout",
            Self::Hybrid => "hybrid",
            Self::Merc => "merc",
            Self::Psyclops => "psyclops",
        }
    }

    /// RocketSim's configuration.
    #[must_use]
    pub fn config(self) -> CarBodyConfig {
        match self {
            Self::Octane => CarBodyConfig::OCTANE,
            Self::Dominus => CarBodyConfig::DOMINUS,
            Self::Plank => CarBodyConfig::PLANK,
            Self::Breakout => CarBodyConfig::BREAKOUT,
            Self::Hybrid => CarBodyConfig::HYBRID,
            Self::Merc => CarBodyConfig::MERC,
            Self::Psyclops => CarBodyConfig::PSYCLOPS,
        }
    }

    /// The hitbox of a car body product id (from the replay's loadout), when the body is known. The table
    /// (`data/body_hitboxes.tsv`) is generated from an item catalog, the official hitbox roster and reviewed
    /// name aliases (data/README.md at the repository root); a body it marks `unmapped` has none.
    #[must_use]
    pub fn of_body(product_id: u32) -> Option<Self> {
        static TABLE: OnceLock<Vec<(u32, Hitbox)>> = OnceLock::new();
        let table = TABLE.get_or_init(|| parse_table(include_str!("../data/body_hitboxes.tsv")));
        let index = table
            .binary_search_by_key(&product_id, |&(id, _)| id)
            .ok()?;
        Some(table[index].1)
    }
}

/// The rows of the body table with a hitbox, sorted by product id. A malformed row is skipped; the test
/// below checks that the embedded table has none.
fn parse_table(text: &str) -> Vec<(u32, Hitbox)> {
    let mut rows: Vec<(u32, Hitbox)> = text
        .lines()
        .skip(1)
        .filter_map(|line| {
            let mut columns = line.split('\t');
            let id = columns.next()?.parse().ok()?;
            let _name = columns.next()?;
            Some((id, Hitbox::from_name(columns.next()?)?))
        })
        .collect();
    rows.sort_by_key(|&(id, _)| id);
    rows
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_row_of_the_embedded_table_is_read() {
        let text = include_str!("../data/body_hitboxes.tsv");
        let mapped = text
            .lines()
            .skip(1)
            .filter(|l| !l.ends_with("\tunmapped") && !l.contains("\tunmapped\t"))
            .count();
        let rows = parse_table(text);
        assert_eq!(rows.len(), mapped, "a row with a hitbox failed to parse");
        assert!(
            rows.windows(2).all(|pair| pair[0].0 < pair[1].0),
            "product ids repeat"
        );
    }

    #[test]
    fn known_bodies_select_their_hitbox() {
        assert_eq!(Hitbox::of_body(23), Some(Hitbox::Octane));
        assert_eq!(Hitbox::of_body(22), Some(Hitbox::Breakout));
        assert_eq!(Hitbox::of_body(24), Some(Hitbox::Plank));
        assert_eq!(Hitbox::of_body(u32::MAX), None);
        for hitbox in [
            Hitbox::Octane,
            Hitbox::Dominus,
            Hitbox::Plank,
            Hitbox::Breakout,
            Hitbox::Hybrid,
            Hitbox::Merc,
            Hitbox::Psyclops,
        ] {
            assert_eq!(Hitbox::from_name(hitbox.name()), Some(hitbox));
        }
    }
}
