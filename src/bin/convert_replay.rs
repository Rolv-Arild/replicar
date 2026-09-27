use std::env;
use std::error::Error;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::PathBuf;

use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes};
use replay_to_rocketsim::serialization::write_jsonl;

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args_os().skip(1);
    let input = PathBuf::from(
        args.next()
            .ok_or("usage: convert_replay <input.replay> <output.jsonl> [collision_meshes]")?,
    );
    let output_path = PathBuf::from(
        args.next()
            .ok_or("usage: convert_replay <input.replay> <output.jsonl> [collision_meshes]")?,
    );
    let mut options = ConvertOptions::default();
    let mut mesh_path = None;
    for arg in args {
        if arg == "--no-inferred-boost" {
            options.infer_boost_from_active = false;
        } else if arg == "--inferred-jump" {
            options.infer_jump_from_active = true;
        } else if arg == "--octane-hitbox" {
            options.use_loadout_hitboxes = false;
        } else if mesh_path.is_none() {
            mesh_path = Some(PathBuf::from(arg));
        } else {
            return Err("usage: convert_replay <input.replay> <output.jsonl> [collision_meshes] [--no-inferred-boost] [--inferred-jump] [--octane-hitbox]".into());
        }
    }
    if let Some(path) = mesh_path {
        options.collision_meshes = path;
    }
    let conversion = convert_bytes(&fs::read(&input)?, &options)?;
    let file = File::create(&output_path)?;
    let mut writer = BufWriter::new(file);
    write_jsonl(&conversion, &mut writer)?;
    writer.flush()?;
    println!(
        "{} frames -> {}",
        conversion.frames.len(),
        output_path.display()
    );
    Ok(())
}
