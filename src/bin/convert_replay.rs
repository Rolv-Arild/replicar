use std::env;
use std::error::Error;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::PathBuf;

use replicar_v1::conversion::{ConvertOptions, convert_bytes};
use replicar_v1::parquet_export::write_parquet_with_tables;
use replicar_v1::parquet_tables::{TABLE_NAMES, table_path};
use replicar_v1::serialization::write_jsonl;

/// `<dir>/.<stem>.partial-<pid>.<ext>` beside the output: the name an export is written under until it is
/// complete (its record tables get `.<stem>.partial-<pid>.<table>.parquet` by `table_path`).
fn temp_path(output: &std::path::Path) -> PathBuf {
    let stem = output
        .file_stem()
        .map_or_else(|| "export".into(), |s| s.to_string_lossy().into_owned());
    let extension = output
        .extension()
        .map_or_else(String::new, |e| format!(".{}", e.to_string_lossy()));
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
    // The test split is sealed until the frozen assessment (TEST_PROTOCOL.md); only that run passes the flag.
    let mut final_assessment = false;
    for arg in args {
        if arg == "--final-assessment" {
            final_assessment = true;
        } else if arg == "--no-event-tables" {
            event_tables = false;
        } else if arg == "--octane-hitbox" {
            options.use_loadout_hitboxes = false;
        } else if arg.to_string_lossy().starts_with('-') {
            return Err(format!("unknown option {}", arg.to_string_lossy()).into());
        } else if mesh_path.is_none() {
            mesh_path = Some(PathBuf::from(arg));
        } else {
            return Err("usage: convert_replay <input.replay> <output.jsonl|output.parquet> [collision_meshes] [--octane-hitbox] [--no-event-tables] [--final-assessment]".into());
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
    let bytes = replicar_v1::read_replay_file(&input, final_assessment)?;
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
    // Publish. Existing outputs (the main file and the tables this run writes) are first moved to `.bak` names;
    // then the tables and the main file are renamed into place. If anything fails, what this run published is
    // deleted again and the backups are restored, so an old export stays complete (old main with its old
    // tables); on success the backups are deleted. A failed backup removal is a warning.
    let mut targets: Vec<PathBuf> = table_rows
        .iter()
        .map(|(table, _)| table_path(&output_path, table))
        .collect();
    targets.push(output_path.clone());
    let backup_of = |target: &std::path::Path| -> PathBuf {
        let name = target
            .file_name()
            .map_or_else(String::new, |n| n.to_string_lossy().into_owned());
        target.with_file_name(format!(".{name}.bak-{}", std::process::id()))
    };
    let mut backups: Vec<(PathBuf, PathBuf)> = Vec::new();
    let mut published: Vec<PathBuf> = Vec::new();
    let publish = (|| -> std::io::Result<()> {
        for target in &targets {
            if target.exists() {
                let backup = backup_of(target);
                fs::rename(target, &backup)?;
                backups.push((target.clone(), backup));
            }
        }
        for (table, _) in &table_rows {
            let target = table_path(&output_path, table);
            fs::rename(table_path(&temp, table), &target)?;
            published.push(target);
        }
        fs::rename(&temp, &output_path)?;
        published.push(output_path.clone());
        Ok(())
    })();
    if let Err(error) = publish {
        for target in &published {
            if let Err(remove_error) = fs::remove_file(target) {
                eprintln!(
                    "warning: could not remove {} while rolling back: {remove_error}",
                    target.display()
                );
            }
        }
        for (target, backup) in &backups {
            if let Err(restore_error) = fs::rename(backup, target) {
                eprintln!(
                    "warning: could not restore {} from {}: {restore_error}",
                    target.display(),
                    backup.display()
                );
            }
        }
        discard(&temp);
        return Err(error.into());
    }
    for (_, backup) in &backups {
        if let Err(error) = fs::remove_file(backup) {
            eprintln!(
                "warning: could not remove the backup {}: {error}",
                backup.display()
            );
        }
    }
    // The stale tables a run without tables leaves behind would be read next to the new main file. The export
    // is complete by now: failing to remove one is a warning.
    if parquet && !event_tables {
        for table in TABLE_NAMES {
            let stale = table_path(&output_path, table);
            match fs::remove_file(&stale) {
                Ok(()) => println!("removed stale {}", stale.display()),
                Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
                Err(error) => eprintln!(
                    "warning: could not remove the stale table {}: {error} (it belongs to an earlier export; delete it)",
                    stale.display()
                ),
            }
        }
    }
    for (table, rows) in &table_rows {
        println!(
            "{rows} rows -> {}",
            table_path(&output_path, table).display()
        );
    }
    println!("{count} frames -> {}", output_path.display());
    Ok(())
}
