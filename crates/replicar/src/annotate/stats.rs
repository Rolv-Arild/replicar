//! Stat events (docs/glossary.md, "Stat event"): a player's match counter (goals, saves, shots, clears, ...)
//! going up, in the frame the replay updates it. Observed: the game's own judgement, seen with the replay's
//! delay. The replay re-sends every counter at its current value now and then (at kickoffs), so only a value
//! above the last one seen is an event; a counter's first value counts from 0 when it is 1, and is a baseline
//! (a replay that starts mid-match) when it is larger.
//!
//! Goals are attributed from the counters too: the scorer is the scoring team's player whose goal counter goes
//! up nearest the goal report (within half a second), the assister the one whose assist counter does
//! (within half a second before to two seconds after: it can arrive during the goal pause).

use std::collections::HashMap;

use replicar_format::{StatKind, Team};

use crate::decode::{NetworkEvent, NetworkFrame, NetworkValue, PlayerKey, PlayerStats};

/// Frames around a goal report within which its scorer's goal counter goes up (30 frames per second).
const SCORER_WINDOW: i64 = 15;
/// Frames after a goal report within which its assister's counter goes up (it is often a few frames late).
const ASSISTER_WINDOW: (i64, i64) = (-15, 60);

/// One counter going up.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StatEvent {
    pub kind: StatKind,
    pub player: PlayerKey,
    pub team: Option<Team>,
    pub total: i32,
}

pub(crate) fn counters(stats: &PlayerStats) -> [(StatKind, Option<&NetworkValue<i32>>); 15] {
    [
        (StatKind::Goal, stats.goals.as_ref()),
        (StatKind::Assist, stats.assists.as_ref()),
        (StatKind::Save, stats.saves.as_ref()),
        (StatKind::Shot, stats.shots.as_ref()),
        (StatKind::Demolition, stats.demolitions.as_ref()),
        (StatKind::EpicSave, stats.epic_saves.as_ref()),
        (StatKind::Clear, stats.clears.as_ref()),
        (StatKind::Center, stats.centers.as_ref()),
        (StatKind::AerialHit, stats.aerial_hits.as_ref()),
        (StatKind::FirstTouch, stats.first_touches.as_ref()),
        (StatKind::CrossbarHit, stats.crossbar_hits.as_ref()),
        (StatKind::BicycleHit, stats.bicycle_hits.as_ref()),
        (StatKind::JuggleHit, stats.juggle_hits.as_ref()),
        (StatKind::FlipReset, stats.flip_resets.as_ref()),
        (StatKind::Demolished, stats.times_demolished.as_ref()),
    ]
}

/// Every frame's stat events, in frame order.
#[must_use]
pub fn stat_events(frames: &[NetworkFrame]) -> Vec<Vec<StatEvent>> {
    let mut last: HashMap<(PlayerKey, StatKind), i32> = HashMap::new();
    frames
        .iter()
        .map(|frame| {
            let mut events = Vec::new();
            for player in &frame.players {
                for (kind, value) in counters(&player.stats) {
                    let Some(value) = value.filter(|v| v.frame == frame.index) else {
                        continue;
                    };
                    let previous = last.insert((player.key.clone(), kind), value.value);
                    let up = match previous {
                        Some(previous) => value.value > previous,
                        None => value.value == 1,
                    };
                    if up {
                        events.push(StatEvent {
                            kind,
                            player: player.key.clone(),
                            team: player.team,
                            total: value.value,
                        });
                    }
                }
            }
            events
        })
        .collect()
}

/// The scorer and assister of every goal report, by frame and by the team scored on: the players of the
/// scoring team whose goal and assist counters went up nearest the report.
#[must_use]
pub fn goal_attribution(
    frames: &[NetworkFrame],
    events: &[Vec<StatEvent>],
) -> HashMap<(usize, Team), (Option<PlayerKey>, Option<PlayerKey>)> {
    let mut out = HashMap::new();
    for (f, frame) in frames.iter().enumerate() {
        for event in &frame.events {
            let NetworkEvent::GoalScoredOn { team } = event else {
                continue;
            };
            let scoring = team.opponent();
            let nearest = |kind: StatKind, (lo, hi): (i64, i64), not: Option<&PlayerKey>| {
                (lo..=hi)
                    .filter_map(|d| {
                        let g = usize::try_from(f as i64 + d).ok()?;
                        let found = events.get(g)?.iter().find(|e| {
                            e.kind == kind && e.team == Some(scoring) && Some(&e.player) != not
                        })?;
                        Some((d.abs(), found.player.clone()))
                    })
                    .min_by_key(|(distance, _)| *distance)
                    .map(|(_, player)| player)
            };
            let scorer = nearest(StatKind::Goal, (-SCORER_WINDOW, SCORER_WINDOW), None);
            let assister = nearest(StatKind::Assist, ASSISTER_WINDOW, scorer.as_ref());
            out.insert((f, *team), (scorer, assister));
        }
    }
    out
}
