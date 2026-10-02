use std::env;
use std::error::Error;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::PathBuf;

use replay_to_rocketsim::conversion::{ConvertOptions, convert_bytes};
use replay_to_rocketsim::parquet_export::write_parquet_with_tables;
use replay_to_rocketsim::parquet_tables::{TABLE_NAMES, table_path};
use replay_to_rocketsim::serialization::write_jsonl;

/// `<dir>/.<stem>.partial-<pid>.<ext>` beside the output: the name an export is written under until it is
/// complete (its record tables get `.<stem>.partial-<pid>.<table>.parquet` by `table_path`).
fn temp_path(output: &std::path::Path) -> PathBuf {
    let stem = output.file_stem().map_or_else(|| "export".into(), |s| s.to_string_lossy().into_owned());
    let extension = output.extension().map_or_else(String::new, |e| format!(".{}", e.to_string_lossy()));
    output.with_file_name(format!(".{stem}.partial-{}{extension}", std::process::id()))
}

/// Remove a half-written export: the temporary main file and its record tables.
fn discard(temp: &std::path::Path) {
    let _ = fs::remove_file(temp);
    for table in TABLE_NAMES {
        let _ = fs::remove_file(table_path(temp, table));
    }
}

fn main() {
    if let Err(error) = run() {
        // Display, not Debug: a plain message (`ConvertError` has one) instead of the error's structure.
        eprintln!("error: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
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
                    .ok_or("--lag-boundary: later or earlier")?;
        } else if arg == "--no-event-tables" {
            event_tables = false;
        } else if arg == "--octane-hitbox" {
            options.use_loadout_hitboxes = false;
        } else if arg == "--gated-low-air-angular" {
            options.hold_low_air_angular = true;
            options.gate_low_air_angular_by_speed = true;
        } else if arg.to_string_lossy().starts_with('-') {
            return Err(format!("unknown option {}", arg.to_string_lossy()).into());
        } else if mesh_path.is_none() {
            mesh_path = Some(PathBuf::from(arg));
        } else {
            return Err("usage: convert_replay <input.replay> <output.jsonl|output.parquet> [collision_meshes] [--no-inferred-boost] [--no-inferred-jump] [--inferred-jump] [--gated-jump] [--octane-hitbox] [--gated-low-air-angular] [--lag-boundary later|earlier] [--no-event-tables]".into());
        }
    }
    if let Some(path) = mesh_path {
        options.collision_meshes = path;
    }
    let extension = output_path
        .extension()
        .map(|extension| extension.to_string_lossy().to_ascii_lowercase());
    let parquet = match extension.as_deref() {
        Some("parquet") => true,
        Some("jsonl") => false,
        _ => {
            return Err(format!(
                "unsupported output extension for {}: use .jsonl (JSON Lines) or .parquet (gzip output such as .jsonl.gz is not written: compress the .jsonl file afterwards)",
                output_path.display()
            )
            .into());
        }
    };
    let bytes = fs::read(&input)?;
    // Everything is written under temporary names beside the output and published only when the whole export
    // succeeded: a failed run leaves an existing output (and its tables) as it was.
    let temp = temp_path(&output_path);
    let written = (|| -> Result<(usize, Vec<(&'static str, usize)>), Box<dyn Error>> {
        if parquet {
            let summary = write_parquet_with_tables(
                &bytes,
                &options,
                File::create(&temp)?,
                event_tables.then_some(temp.as_path()),
            )?;
            Ok((summary.frames, summary.table_rows))
        } else {
            let conversion = convert_bytes(&bytes, &options)?;
            let mut writer = BufWriter::new(File::create(&temp)?);
            write_jsonl(&conversion, &mut writer)?;
            writer.flush()?;
            Ok((conversion.frames.len(), Vec::new()))
        }
    })();
    let (count, table_rows) = match written {
        Ok(done) => done,
        Err(error) => {
            discard(&temp);
            return Err(error);
        }
    };
    // Publish: the tables first, the main file last, then the stale tables a run without tables leaves behind
    // (they would be read next to the new main file).
    let published = (|| -> std::io::Result<()> {
        for (table, _) in &table_rows {
            fs::rename(table_path(&temp, table), table_path(&output_path, table))?;
        }
        fs::rename(&temp, &output_path)?;
        if parquet && !event_tables {
            for table in TABLE_NAMES {
                match fs::remove_file(table_path(&output_path, table)) {
                    Ok(()) => println!("removed stale {}", table_path(&output_path, table).display()),
                    Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
                    Err(error) => return Err(error),
                }
            }
        }
        Ok(())
    })();
    if let Err(error) = published {
        discard(&temp);
        return Err(error.into());
    }
    for (table, rows) in &table_rows {
        println!("{rows} rows -> {}", table_path(&output_path, table).display());
    }
    println!("{count} frames -> {}", output_path.display());
    Ok(())
}
