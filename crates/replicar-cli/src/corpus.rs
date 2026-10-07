//! A folder of replays: one file per replay, mirroring the input's subfolders, converted `jobs` at a time, and
//! `index.parquet` with one row per replay (docs/v2-plan.md, section 3.4). A replay that fails becomes a row
//! with its error, not an aborted batch.

use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

use replicar_format::{Columns, WriteOptions};
use sha2::{Digest, Sha256};

/// One row of the index.
#[derive(Debug, Default, Clone)]
struct IndexRow {
    replay: String,
    file: String,
    sha256: Option<String>,
    error: Option<String>,
    rows: Option<u32>,
    duration_seconds: Option<f32>,
    map: Option<String>,
    players: Option<u8>,
    blue_players: Option<u8>,
    orange_players: Option<u8>,
    blue_score: Option<i32>,
    orange_score: Option<i32>,
    segments: Option<u32>,
}

fn replays(dir: &Path, out: &mut Vec<PathBuf>) -> std::io::Result<()> {
    for entry in std::fs::read_dir(dir)? {
        let path = entry?.path();
        if path.is_dir() {
            replays(&path, out)?;
        } else if path.extension().is_some_and(|e| e == "replay") {
            out.push(path);
        }
    }
    Ok(())
}

/// The index row of a written file, from its header.
fn row_of_file(replay: &str, file: &Path, relative: &str) -> IndexRow {
    let mut row = IndexRow {
        replay: replay.to_owned(),
        file: relative.to_owned(),
        ..IndexRow::default()
    };
    match replicar_format::read(file, Some(&["replay_time"])) {
        Ok((header, batch)) => {
            let times = batch
                .column(0)
                .as_any()
                .downcast_ref::<replicar_format::arrow::Float32Array>()
                .map(|t| t.values().to_vec())
                .unwrap_or_default();
            row.sha256 = Some(header.replay_sha256.clone());
            row.rows = u32::try_from(batch.num_rows()).ok();
            row.duration_seconds = times.first().zip(times.last()).map(|(a, b)| b - a);
            row.map = header.diagnostics["decode"]["map_name"]
                .as_str()
                .map(str::to_owned);
            row.players = u8::try_from(header.players.len()).ok();
            let team =
                |t: u8| u8::try_from(header.players.iter().filter(|p| p.team == t).count()).ok();
            row.blue_players = team(0);
            row.orange_players = team(1);
            row.blue_score = header.final_scores[0];
            row.orange_score = header.final_scores[1];
            row.segments = u32::try_from(header.segments.len()).ok();
        }
        Err(error) => row.error = Some(format!("reading the written file: {error}")),
    }
    row
}

pub(crate) fn convert_folder(
    meshes: &replicar::Meshes,
    input: &Path,
    output: &Path,
    options: &WriteOptions,
    jobs: usize,
    skip_existing: bool,
) -> Result<(), String> {
    let mut paths = Vec::new();
    replays(input, &mut paths).map_err(|e| format!("{}: {e}", input.display()))?;
    paths.sort();
    std::fs::create_dir_all(output).map_err(|e| e.to_string())?;
    let next = AtomicUsize::new(0);
    let rows = Mutex::new(Vec::with_capacity(paths.len()));
    let failures = AtomicUsize::new(0);
    std::thread::scope(|scope| {
        for _ in 0..jobs.max(1) {
            scope.spawn(|| {
                let converter = replicar::Converter::new(meshes, replicar::Config::default());
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    let Some(path) = paths.get(i) else {
                        break;
                    };
                    let relative = path.strip_prefix(input).unwrap_or(path);
                    let target = output.join(relative).with_extension("parquet");
                    let relative_file = target
                        .strip_prefix(output)
                        .unwrap_or(&target)
                        .to_string_lossy()
                        .replace('\\', "/");
                    let replay = relative.to_string_lossy().replace('\\', "/");
                    let row = if skip_existing && target.exists() {
                        row_of_file(&replay, &target, &relative_file)
                    } else {
                        match crate::convert_one(&converter, path, &target, options) {
                            Ok(_) => row_of_file(&replay, &target, &relative_file),
                            Err(error) => {
                                failures.fetch_add(1, Ordering::Relaxed);
                                eprintln!("failed: {replay}: {error}");
                                IndexRow {
                                    replay: replay.clone(),
                                    sha256: std::fs::read(path)
                                        .ok()
                                        .map(|b| format!("{:x}", Sha256::digest(b))),
                                    error: Some(error),
                                    ..IndexRow::default()
                                }
                            }
                        }
                    };
                    eprintln!(
                        "{}/{}  {replay}",
                        rows.lock().map_or(0, |r| r.len()) + 1,
                        paths.len()
                    );
                    if let Ok(mut rows) = rows.lock() {
                        rows.push(row);
                    }
                }
            });
        }
    });
    let mut rows = rows.into_inner().map_err(|e| e.to_string())?;
    rows.sort_by(|a, b| a.replay.cmp(&b.replay));
    write_index(&output.join("index.parquet"), &rows, options)?;
    let failed = failures.load(Ordering::Relaxed);
    eprintln!(
        "{} replays, {failed} failed; index: {}",
        rows.len(),
        output.join("index.parquet").display()
    );
    if failed == 0 {
        Ok(())
    } else {
        Err(format!("{failed} replays failed (see the index)"))
    }
}

fn write_index(path: &Path, rows: &[IndexRow], options: &WriteOptions) -> Result<(), String> {
    const GROUP: &str = "index";
    let mut columns = Columns::default();
    let strings = |f: fn(&IndexRow) -> Option<String>| rows.iter().map(f).collect::<Vec<_>>();
    columns.strings("replay", GROUP, strings(|r| Some(r.replay.clone())));
    columns.strings(
        "file",
        GROUP,
        strings(|r| (!r.file.is_empty()).then(|| r.file.clone())),
    );
    columns.strings("sha256", GROUP, strings(|r| r.sha256.clone()));
    columns.strings("error", GROUP, strings(|r| r.error.clone()));
    columns.u32("rows", GROUP, None, rows.iter().map(|r| r.rows));
    columns.f32(
        "duration_seconds",
        GROUP,
        Some("s"),
        rows.iter().map(|r| r.duration_seconds),
    );
    columns.strings("map", GROUP, strings(|r| r.map.clone()));
    columns.u8("players", GROUP, None, rows.iter().map(|r| r.players));
    columns.u8(
        "blue_players",
        GROUP,
        None,
        rows.iter().map(|r| r.blue_players),
    );
    columns.u8(
        "orange_players",
        GROUP,
        None,
        rows.iter().map(|r| r.orange_players),
    );
    columns.i32("blue_score", GROUP, rows.iter().map(|r| r.blue_score));
    columns.i32("orange_score", GROUP, rows.iter().map(|r| r.orange_score));
    columns.u32("segments", GROUP, None, rows.iter().map(|r| r.segments));
    let batch = columns.finish().map_err(|e| e.to_string())?;
    let groups: Vec<&str> = options.groups.iter().map(|g| g.name()).collect();
    replicar_format::write_table(
        path,
        &batch,
        &[(
            "replicar_index",
            serde_json::json!({
                "groups": groups,
                "precision": options.precision.name(),
                "all_frames": options.all_frames,
            })
            .to_string(),
        )],
    )
    .map_err(|e| e.to_string())
}
