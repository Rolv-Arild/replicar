//! `replicar`: converts Rocket League replays to replicar files (docs/v2-plan.md, section 4.5).
//!
//! ```text
//! replicar convert match.replay -o match.parquet [--rows ticks|frames] [--tick-step N] [--precision float32|quantized]
//!     [--with GROUPS | --groups GROUPS] [--all-frames]
//! replicar convert replays/ -o out/ [--jobs N] [--skip-existing] ...   # one file per replay + out/index.parquet
//! replicar resimulate match.parquet --replay match.replay -o full.parquet
//! replicar inspect match.replay | match.parquet
//! replicar verify match.parquet [--replay match.replay]
//!   --meshes DIR, else $REPLICAR_MESHES, else ./collision_meshes
//! ```

mod corpus;

#[global_allocator]
static ALLOCATOR: mimalloc::MiMalloc = mimalloc::MiMalloc;

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{Args, Parser, Subcommand, ValueEnum};
use replicar_format::{Group, Precision, RowRate, WriteOptions};

#[derive(Parser)]
#[command(
    name = "replicar",
    version,
    about = "Rocket League replays reconstructed as RocketSim states, one Parquet file per replay"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Convert a replay to a file, or every replay under a folder to a folder of files and an index.
    Convert {
        /// A .replay file or a folder of them (searched recursively).
        input: PathBuf,
        /// The output file, or folder for a folder input (subfolders are mirrored).
        #[arg(short, long)]
        output: PathBuf,
        #[command(flatten)]
        file: FileArgs,
        /// Replays converted at once (folder input).
        #[arg(long, default_value_t = default_jobs())]
        jobs: usize,
        /// Leave replays whose output file exists (folder input; resumes an interrupted run).
        #[arg(long)]
        skip_existing: bool,
        #[command(flatten)]
        meshes: MeshArgs,
    },
    /// Rebuild a file from its replay and its `resimulation` group, without fitting: the same states.
    Resimulate {
        /// A replicar file with the `resimulation` group.
        file: PathBuf,
        /// The replay the file was converted from.
        #[arg(long)]
        replay: PathBuf,
        #[arg(short, long)]
        output: PathBuf,
        #[command(flatten)]
        options: FileArgs,
        #[command(flatten)]
        meshes: MeshArgs,
    },
    /// Summarize a replay or a replicar file.
    Inspect {
        path: PathBuf,
        /// Print the file's players as CSV (index, name, team, hitbox, body, key and final statistics).
        #[arg(long)]
        players: bool,
    },
    /// Check a replicar file: its header and every column; with the replay, also that resimulating it
    /// reproduces its states.
    Verify {
        file: PathBuf,
        #[arg(long)]
        replay: Option<PathBuf>,
        #[command(flatten)]
        meshes: MeshArgs,
    },
}

#[derive(Clone, Copy, ValueEnum)]
enum RowsArg {
    /// A row per simulated tick in play (every `--tick-step`-th).
    Ticks,
    /// A row per replay frame.
    Frames,
}

#[derive(Clone, Copy, ValueEnum)]
enum PrecisionArg {
    Float32,
    Quantized,
}

/// What a written file holds.
#[derive(Args, Clone)]
struct FileArgs {
    /// How the state's bodies are stored.
    #[arg(long, value_enum, default_value = "float32")]
    precision: PrecisionArg,
    /// Groups added to the default ones (state, game, updates, future): resimulation, network, diagnostics.
    #[arg(long, value_delimiter = ',')]
    with: Vec<String>,
    /// The whole set of groups, instead of the default ones.
    #[arg(long, value_delimiter = ',', conflicts_with = "with")]
    groups: Vec<String>,
    /// A row per simulated 120 Hz tick in play, or per replay frame.
    #[arg(long, value_enum, default_value = "ticks")]
    rows: RowsArg,
    /// With `--rows ticks`: only the ticks whose sim tick is a multiple of N.
    #[arg(long, default_value_t = 1, value_parser = clap::value_parser!(u32).range(1..))]
    tick_step: u32,
    /// Also write the frames outside play segments (countdowns, goal pauses and replays).
    #[arg(long)]
    all_frames: bool,
}

#[derive(Args, Clone)]
struct MeshArgs {
    /// RocketSim's collision meshes (a folder with `soccar/*.cmf`); else $REPLICAR_MESHES, else
    /// ./collision_meshes.
    #[arg(long)]
    meshes: Option<PathBuf>,
}

fn default_jobs() -> usize {
    std::thread::available_parallelism().map_or(1, usize::from)
}

impl FileArgs {
    fn options(&self) -> Result<WriteOptions, String> {
        let parse = |names: &[String]| -> Result<Vec<Group>, String> {
            names
                .iter()
                .map(|n| Group::from_name(n).ok_or_else(|| format!("unknown group {n}")))
                .collect()
        };
        let mut groups: BTreeSet<Group> = if self.groups.is_empty() {
            Group::DEFAULT.into_iter().collect()
        } else {
            parse(&self.groups)?.into_iter().collect()
        };
        groups.extend(parse(&self.with)?);
        Ok(WriteOptions {
            groups,
            precision: match self.precision {
                PrecisionArg::Float32 => Precision::Float32,
                PrecisionArg::Quantized => Precision::Quantized,
            },
            rows: match self.rows {
                RowsArg::Ticks => RowRate::Ticks(self.tick_step),
                RowsArg::Frames => RowRate::Frames,
            },
            all_frames: self.all_frames,
            ..WriteOptions::default()
        })
    }
}

impl MeshArgs {
    fn load(&self) -> Result<replicar::Meshes, String> {
        let directory = self
            .meshes
            .clone()
            .or_else(|| std::env::var_os("REPLICAR_MESHES").map(PathBuf::from))
            .unwrap_or_else(|| PathBuf::from("collision_meshes"));
        replicar::Meshes::load(&directory).map_err(|e| e.to_string())
    }
}

fn run(cli: Cli) -> Result<(), String> {
    match cli.command {
        Command::Convert {
            input,
            output,
            file,
            jobs,
            skip_existing,
            meshes,
        } => {
            let options = file.options()?;
            let meshes = meshes.load()?;
            if input.is_dir() {
                corpus::convert_folder(&meshes, &input, &output, &options, jobs, skip_existing)
            } else {
                let converter = replicar::Converter::new(&meshes, replicar::Config::default());
                replicar::corpus::convert_file(&converter, &input, &output, &options)
                    .map(|_| ())
                    .map_err(|e| e.to_string())
            }
        }
        Command::Resimulate {
            file,
            replay,
            output,
            options,
            meshes,
        } => {
            let options = options.options()?;
            let meshes = meshes.load()?;
            let converter = replicar::Converter::new(&meshes, replicar::Config::default());
            let bytes = std::fs::read(&replay).map_err(|e| format!("{}: {e}", replay.display()))?;
            let conversion = converter
                .resimulate(&bytes, &file)
                .map_err(|e| e.to_string())?;
            conversion
                .write(&output, &options)
                .map_err(|e| e.to_string())
        }
        Command::Inspect { path, players } if players => players_csv(&path),
        Command::Inspect { path, .. } => inspect(&path),
        Command::Verify {
            file,
            replay,
            meshes,
        } => verify(&file, replay.as_deref(), &meshes),
    }
}

fn inspect(path: &Path) -> Result<(), String> {
    if path.extension().is_some_and(|e| e == "replay") {
        let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
        let network =
            replicar::decode::decode(&replicar::parse(&bytes).map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?;
        let duration = network.frames.last().map_or(0.0, |f| f.time)
            - network.frames.first().map_or(0.0, |f| f.time);
        let mut players: Vec<String> = network
            .frames
            .iter()
            .flat_map(|f| &f.players)
            .map(|p| {
                format!(
                    "{} ({})",
                    p.name.clone().unwrap_or_else(|| p.key.0.clone()),
                    p.team.map_or("no team", |t| if t.number() == 0 {
                        "blue"
                    } else {
                        "orange"
                    })
                )
            })
            .collect();
        players.sort();
        players.dedup();
        println!("replay       {}", path.display());
        println!("game type    {}", network.header.game_type);
        println!("frames       {} ({duration:.1} s)", network.frames.len());
        println!("final score  {:?}", network.header.final_scores);
        println!("players      {}", players.join(", "));
        return Ok(());
    }
    let (header, batch) =
        replicar_format::read(path, Some(&["frame"])).map_err(|e| e.to_string())?;
    println!("file          {}", path.display());
    println!("format        {}", header.format_version);
    println!("replay        {}", header.replay_sha256);
    println!(
        "versions      replicar {}, RocketSim {}",
        header.replicar_version, header.rocketsim_version
    );
    println!(
        "groups        {} ({}{})",
        header.groups.join(", "),
        header.precision,
        if header.all_frames {
            ", all frames"
        } else {
            ""
        }
    );
    println!("rows          {}", batch.num_rows());
    println!("final score   {:?}", header.final_scores);
    for player in &header.players {
        println!(
            "player {}      {} (team {}, {})",
            player.index,
            player.name.as_deref().unwrap_or(&player.key),
            player.team,
            player.hitbox
        );
    }
    let ends: Vec<&str> = header.segments.iter().map(|s| s.end.as_str()).collect();
    println!("segments      {} ({})", ends.len(), ends.join(", "));
    Ok(())
}

/// A CSV field, quoted when it needs to be.
fn csv_field(value: &str) -> String {
    if value.contains([',', '"', '\n']) {
        format!("\"{}\"", value.replace('"', "\"\""))
    } else {
        value.to_owned()
    }
}

fn players_csv(path: &Path) -> Result<(), String> {
    let (header, _) = replicar_format::read(path, Some(&["frame"])).map_err(|e| e.to_string())?;
    let stats: std::collections::BTreeSet<&String> = header
        .players
        .iter()
        .flat_map(|p| p.final_stats.keys())
        .collect();
    let mut line = vec!["player", "name", "team", "hitbox", "body_product_id", "key"]
        .into_iter()
        .map(str::to_owned)
        .collect::<Vec<_>>();
    line.extend(stats.iter().map(|s| format!("final_{s}")));
    println!("{}", line.join(","));
    for p in &header.players {
        let mut line = vec![
            p.index.to_string(),
            csv_field(p.name.as_deref().unwrap_or("")),
            p.team.to_string(),
            p.hitbox.clone(),
            p.body_product_id.map_or(String::new(), |b| b.to_string()),
            csv_field(&p.key),
        ];
        line.extend(stats.iter().map(|s| {
            p.final_stats
                .get(*s)
                .map_or(String::new(), ToString::to_string)
        }));
        println!("{}", line.join(","));
    }
    Ok(())
}

fn verify(file: &Path, replay: Option<&Path>, meshes: &MeshArgs) -> Result<(), String> {
    let (header, batch) = replicar_format::read(file, None).map_err(|e| e.to_string())?;
    println!(
        "{}: format {}, {} rows, {} columns, groups {}",
        file.display(),
        header.format_version,
        batch.num_rows(),
        batch.num_columns(),
        header.groups.join(", ")
    );
    if let Some(replay) = replay {
        let meshes = meshes.load()?;
        let converter = replicar::Converter::new(&meshes, replicar::Config::default());
        let bytes = std::fs::read(replay).map_err(|e| e.to_string())?;
        converter
            .resimulate(&bytes, file)
            .map_err(|e| e.to_string())?;
        println!("resimulated: the states match the file's checksum");
    }
    Ok(())
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("error: {error}");
            ExitCode::FAILURE
        }
    }
}
