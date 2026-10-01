//! Lists the object names containing "Demolish" in a replay and how often each is updated.
//! usage: list_demolish_attributes <replay>
use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;

fn main() -> Result<(), Box<dyn Error>> {
    let path = env::args().nth(1).ok_or("usage: list_demolish_attributes <replay>")?;
    if path.contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let bytes = fs::read(path)?;
    let replay = boxcars::ParserBuilder::new(&bytes).must_parse_network_data().parse()?;
    let mut counts: BTreeMap<String, usize> = BTreeMap::new();
    let mut example: BTreeMap<String, String> = BTreeMap::new();
    for frame in &replay.network_frames.as_ref().ok_or("no network frames")?.frames {
        for u in &frame.updated_actors {
            let name = replay.objects.get(u.object_id.0 as usize).cloned().unwrap_or_default();
            if name.contains("Demolish") || name.contains("Demo") {
                *counts.entry(name.clone()).or_default() += 1;
                example.entry(name).or_insert_with(|| format!("{:?}", u.attribute));
            }
        }
    }
    for (k, v) in counts {
        println!("{k}: {v}  e.g. {}", example[&k]);
    }
    Ok(())
}
