//! Decode: the replay's network feed as typed values per frame, each with the frame of its last change
//! (docs/glossary.md, "Network value"). The first stage of the pipeline; nothing here is inferred except
//! the spawn pose and a start-of-match score of 0, both marked as such.

mod attributes;
mod network;
mod tracker;

use boxcars::{HeaderProp, Replay};
use replicar_format::FrameIndex;

pub use network::*;
pub use tracker::DEMOLITION_REPEAT_WINDOW;

use crate::Error;

/// Decode a parsed replay's network frames.
pub fn decode(replay: &Replay) -> Result<NetworkReplay, Error> {
    let frames = &replay
        .network_frames
        .as_ref()
        .ok_or(Error::NoNetworkFrames)?
        .frames;
    if u32::try_from(frames.len()).is_err() {
        return Err(Error::TooManyFrames(frames.len()));
    }
    let name_of = |object: boxcars::ObjectId| {
        usize::try_from(object.0)
            .ok()
            .and_then(|index| replay.objects.get(index))
            .map(String::as_str)
    };
    let mut tracker = tracker::Tracker::default();
    let mut decoded = Vec::with_capacity(frames.len());
    let mut previous_time: Option<f32> = None;
    for (index, frame) in frames.iter().enumerate() {
        let index = FrameIndex(index as u32);
        if previous_time.is_some_and(|previous| frame.time < previous) {
            tracker.diagnostics.non_monotonic_frame_times += 1;
        }
        previous_time = Some(frame.time);
        for actor in &frame.deleted_actors {
            tracker.delete(ActorId(actor.0));
        }
        for actor in &frame.new_actors {
            if let Some(class) = name_of(actor.object_id) {
                tracker.announce(
                    ActorId(actor.actor_id.0),
                    class,
                    index,
                    &actor.initial_trajectory,
                );
            }
        }
        // Links first, so that every value below finds its car, component and player.
        for update in &frame.updated_actors {
            if let Some(property) = name_of(update.object_id) {
                tracker.link(
                    ActorId(update.actor_id.0),
                    property,
                    &update.attribute,
                    index,
                );
            }
        }
        let mut reports = tracker::FrameReports::default();
        for update in &frame.updated_actors {
            if let Some(property) = name_of(update.object_id) {
                tracker.observe(
                    ActorId(update.actor_id.0),
                    property,
                    &update.attribute,
                    &replay.names,
                    index,
                    &mut reports,
                );
            }
        }
        decoded.push(tracker.snapshot(index, frame.time, frame.delta, reports));
    }
    let mut diagnostics = tracker.diagnostics;
    diagnostics.map_name =
        replay
            .properties
            .iter()
            .find_map(|(name, prop)| match (name.as_str(), prop) {
                (attributes::HEADER_MAP_NAME, HeaderProp::Name(map) | HeaderProp::Str(map)) => {
                    Some(map.clone())
                }
                _ => None,
            });
    diagnostics.nonstandard_notes =
        tracker::nonstandard_notes(diagnostics.map_name.as_deref(), &diagnostics.game_settings);
    Ok(NetworkReplay {
        header: ReplayHeader {
            game_type: replay.game_type.clone(),
            levels: replay.levels.clone(),
            final_scores: [
                header_int(replay, attributes::HEADER_BLUE_SCORE),
                header_int(replay, attributes::HEADER_ORANGE_SCORE),
            ],
        },
        frames: decoded,
        diagnostics,
    })
}

fn header_int(replay: &Replay, key: &str) -> Option<i32> {
    replay
        .properties
        .iter()
        .find_map(|(name, prop)| match prop {
            HeaderProp::Int(value) if name == key => Some(*value),
            _ => None,
        })
}
