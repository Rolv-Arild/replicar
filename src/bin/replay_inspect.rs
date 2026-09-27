//! Print selected replay network attributes for schema investigation.

use std::env;
use std::error::Error;
use std::fs;

use replay_to_rocketsim::parse_replay;

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let path = args
        .next()
        .ok_or("usage: replay_inspect <file> <name-substring> [limit]")?;
    let needle = args
        .next()
        .ok_or("usage: replay_inspect <file> <name-substring> [limit]")?;
    let exact = needle.strip_prefix('=');
    let limit: usize = args.next().unwrap_or_else(|| "20".into()).parse()?;
    let replay = parse_replay(&fs::read(path)?)?;
    let frames = replay
        .network_frames
        .as_ref()
        .ok_or("network frames absent")?;
    let mut emitted = 0;
    for (frame_idx, frame) in frames.frames.iter().enumerate() {
        for actor in &frame.new_actors {
            let name = usize::try_from(actor.object_id.0)
                .ok()
                .and_then(|id| replay.objects.get(id));
            if name.is_some_and(|name| exact.map_or_else(|| name.contains(&needle), |x| name == x))
            {
                println!(
                    "frame {frame_idx} time {:.3} NEW {:?} {name:?}",
                    frame.time, actor.actor_id
                );
                emitted += 1;
            }
            if emitted >= limit {
                return Ok(());
            }
        }
        for update in &frame.updated_actors {
            let name = usize::try_from(update.object_id.0)
                .ok()
                .and_then(|id| replay.objects.get(id));
            if name.is_some_and(|name| exact.map_or_else(|| name.contains(&needle), |x| name == x))
            {
                println!(
                    "frame {frame_idx} time {:.3} actor {:?} {name:?}: {:?}",
                    frame.time, update.actor_id, update.attribute
                );
                emitted += 1;
            }
            if emitted >= limit {
                return Ok(());
            }
        }
    }
    Ok(())
}
