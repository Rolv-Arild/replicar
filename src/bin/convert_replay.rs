use std::env;
use std::error::Error;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::PathBuf;

use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes};
use replay_to_rocketsim::parquet_export::write_parquet_with_tables;
use replay_to_rocketsim::parquet_tables::table_path;
use replay_to_rocketsim::serialization::write_jsonl;

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args_os().skip(1);
    let input = PathBuf::from(args.next().ok_or(
        "usage: convert_replay <input.replay> <output.jsonl|output.parquet> [collision_meshes]",
    )?);
    let output_path = PathBuf::from(args.next().ok_or(
        "usage: convert_replay <input.replay> <output.jsonl|output.parquet> [collision_meshes]",
    )?);
    let mut options = ConvertOptions::default();
    let mut mesh_path = None;
    let mut event_tables = true;
    while let Some(arg) = args.next() {
        if arg == "--no-inferred-boost" {
            options.infer_boost_from_active = false;
        } else if arg == "--inferred-jump" {
            options.infer_jump_from_active = true;
            options.gate_jump_on_observed_impulse = false;
        } else if arg == "--gated-jump" {
            options.infer_jump_from_active = true;
            options.gate_jump_on_observed_impulse = true;
        } else if arg == "--no-inferred-jump" {
            options.infer_jump_from_active = false;
            options.gate_jump_on_observed_impulse = false;
        } else if arg == "--no-align-contacts" {
            options.align_contacts = false;
        } else if arg == "--lag-boundary" {
            let name = args.next().ok_or("--lag-boundary requires a name")?;
            options.lag_boundary =
                replay_to_rocketsim::conversion::LagBoundary::from_name(&name.to_string_lossy())
                    .ok_or("--lag-boundary: later, earlier or longer")?;
        } else if arg == "--no-event-tables" {
            event_tables = false;
        } else if arg == "--octane-hitbox" {
            options.use_loadout_hitboxes = false;
        } else if arg == "--gated-low-air-angular" {
            options.hold_low_air_angular = true;
            options.gate_low_air_angular_by_speed = true;
        } else if mesh_path.is_none() {
            mesh_path = Some(PathBuf::from(arg));
        } else {
            return Err("usage: convert_replay <input.replay> <output.jsonl|output.parquet> [collision_meshes] [--no-inferred-boost] [--no-inferred-jump] [--inferred-jump] [--gated-jump] [--octane-hitbox] [--gated-low-air-angular] [--lag-boundary later|earlier|longer] [--no-event-tables]".into());
        }
    }
    if let Some(path) = mesh_path {
        options.collision_meshes = path;
    }
    let bytes = fs::read(&input)?;
    let count = if output_path
        .extension()
        .is_some_and(|extension| extension.eq_ignore_ascii_case("parquet"))
    {
        let summary = write_parquet_with_tables(
            &bytes,
            &options,
            File::create(&output_path)?,
            event_tables.then_some(output_path.as_path()),
        )?;
        for (table, rows) in &summary.table_rows {
            println!(
                "{rows} rows -> {}",
                table_path(&output_path, table).display()
            );
        }
        summary.frames
    } else {
        let conversion = convert_bytes(&bytes, &options)?;
        let file = File::create(&output_path)?;
        let mut writer = BufWriter::new(file);
        write_jsonl(&conversion, &mut writer)?;
        writer.flush()?;
        conversion.frames.len()
    };
    println!("{count} frames -> {}", output_path.display());
    Ok(())
}
