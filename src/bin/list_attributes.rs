//! Lists the replay object names that contain any of the given patterns (case-insensitive), whether the
//! name exists in the object table and how often an attribute of that name is updated, with an example.
//! Also prints the header's build and date. Generalises `list_demolish_attributes`.
//! With `--trace` it also prints every matching update as `frame time actor_id name attribute`.
//! usage: list_attributes <replay> <pattern>[,<pattern>...] [--trace]
use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::fs;

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let path = args.next().ok_or("usage: list_attributes <replay> <pattern>[,<pattern>...]")?;
    let patterns: Vec<String> = args
        .next()
        .ok_or("usage: list_attributes <replay> <pattern>[,<pattern>...]")?
        .split(',')
        .map(|p| p.to_lowercase())
        .collect();
    if path.contains("test") {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    let trace = args.next().is_some_and(|a| a == "--trace");
    let bytes = fs::read(path)?;
    let replay = boxcars::ParserBuilder::new(&bytes).must_parse_network_data().parse()?;
    for (key, value) in &replay.properties {
        if matches!(key.as_str(), "BuildVersion" | "Date" | "GameVersion" | "MatchType" | "TeamSize") {
            println!("header {key}: {value:?}");
        }
    }
    let matches = |name: &str| {
        let lower = name.to_lowercase();
        patterns.iter().any(|p| lower.contains(p))
    };
    let in_table: Vec<&String> = replay.objects.iter().filter(|n| matches(n)).collect();
    println!("object table entries matching: {}", in_table.len());
    let mut counts: BTreeMap<String, usize> = BTreeMap::new();
    let mut example: BTreeMap<String, String> = BTreeMap::new();
    for (index, frame) in replay.network_frames.as_ref().ok_or("no network frames")?.frames.iter().enumerate() {
        for u in &frame.updated_actors {
            let name = replay.objects.get(u.object_id.0 as usize).cloned().unwrap_or_default();
            if matches(&name) {
                if trace {
                    println!("{index} {:.3} {} {name} {:?}", frame.time, u.actor_id.0, u.attribute);
                }
                *counts.entry(name.clone()).or_default() += 1;
                example.entry(name).or_insert_with(|| format!("{:?}", u.attribute));
            }
        }
    }
    for name in in_table {
        let updated = counts.get(name.as_str()).copied().unwrap_or(0);
        println!("{name}: updated {updated} times{}", example.get(name.as_str()).map_or(String::new(), |e| format!("  e.g. {e}")));
    }
    Ok(())
}
