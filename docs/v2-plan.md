# replicar v2: design and migration plan

Status: proposal, fifth draft, 2026-10-06, branch `v2-plan`. Nothing here is implemented.

Earlier drafts:
- `ac4b8c4`: a refactor of v1 in place;
- `95b0fd6`: the first ground-up draft;
- `1c33303`: the third draft (column groups, `exact`/`compact`), before the names were settled;
- `a67e23d`: the fourth draft with the glossary, before the frames were limited to play segments.

Sources and conventions:
- Facts about v1 cite files at `master` = `5a7da58`.
- Facts about VirxEC's converter cite `external/replay-to-rocketsim` (`bc61b66`).
- Facts about RocketSim's rlpr format cite `rocketsim_test/src/rlpr/` at our pin `9910c58`.
- The measurements are in RESULTS.md, "Output size and conversion cost (2026-10-06)" and "v2 encodings:
  rotation and lossy state (2026-10-06)", with the scripts that reproduce them.
- Names follow `docs/glossary.md`. **[decide]** marks a choice for the user, and **[verify]** an assumption a story must check before relying on it.

## 1. What the user asked for (2026-10-06)

- **Outputs that are simple and small.** JSONL takes too much space, and the Parquet export is eight files per
  replay. The target corpus is about **140,000 replays**, so file size is a first-order constraint. The
  conversion is also too slow to embed in a streaming pipeline.
- **Code that is not opaque**, redesigned from the ground up.
- **The same accuracy.** Speed must be no worse (mean +3%, no replay +10%), and we take obvious gains along the
  way, but speed is not the focus yet.
- **Decisions taken:**
  - The sealed-split guard moves into the evaluation tools.
  - `thiserror`, `clap`, `pyo3`/`maturin` and `criterion` may be added.
  - Aim for crates.io and PyPI releases.
  - Update RocketSim to the latest compatible revision before the work, and run every comparison on that one
    revision.
  - Leave the collision meshes to the user.
  - Python 3.12 or newer.
- **Decisions from the later rounds:**
  - **States by default; replay + inferred inputs as an option**, and both without making the code a mess. Not
    every user wants the cost, or the dependency, of running the replay parser and the simulator.
  - **Quaternions** for rotations; their precision is plenty.
  - **A lossy option** if it saves a substantial amount of space. The reconstruction error is far larger than any
    reasonable quantization.
  - **No stopgap.** Nothing will be run until v2 is done.
  - **The network values are opt-in** (the `network` group): they are essentially the replay's network feed.
  - **Names must be clear**, with a terminology reference (`docs/glossary.md`, section 8).
  - **Only play is core data.** Rows run from the kickoff (countdown at 0, cars free) to the goal; countdowns,
    goal celebrations and goal replays are left out unless asked for (`--all-frames`).
  - **The future-derived columns are in the default groups**, under a better name than "labels" (`future`).
  - **Plain `.parquet`.** `inferred` was ambiguous (now `resimulation`).

## 2. What the measurements say

Three train replays were measured, one per game size (0.83, 0.75 and 1.61 MB), converted with the v1 release
build.

**Where the bytes go in v1.**
- A Parquet export is **15–17.5× the replay**, and JSONL about 150×.
- **72% of it is `frame_json`**, a complete JSON copy of every frame. Of what only that column holds:
  - the car-state fields still needed for restoration (timers, flags, previous controls) cost 0.2–0.6 MB as
    typed columns;
  - the replay's own field values with their update frames cost 0.9–2.8 MB;
  - the evaluator's position residuals cost 1.2–2.7 MB;
  - the rest repeats typed columns or constants (pad positions, Dropshot and Heatseeker fields on every frame).

**What a v2 state file costs.** Typed columns with BYTE_STREAM_SPLIT float encoding at zstd 9, including the car
internals:

| State file | 1v1 | 2v2 | 3v3 | × replay |
| --- | ---: | ---: | ---: | ---: |
| v1 Parquet (for comparison) | 12.45 MB | 12.09 MB | 28.14 MB | 15–17.5 |
| lossless, rotation as a 3×3 matrix | 2.34 | 2.38 | 5.49 | 2.8–3.4 |
| **lossless, rotation as a quaternion** | **1.85** | **1.90** | **4.34** | **2.2–2.7** |
| **quantized: 0.01 UU, 0.01 UU/s, 1e-4 rad/s, int16 quaternion** | **1.31** | **1.33** | **2.88** | **1.6–1.8** |
| quantized coarse: 0.1 UU, 0.1 UU/s, 1e-3 rad/s | 1.21 | 1.23 | 2.64 | 1.5–1.6 |

- **Rotations.** They are about a third of a matrix-based file, because a matrix is 9 floats per body with
  incompressible low bits. A quaternion cuts the whole file by about 21%. A matrix rebuilt from it differs from
  RocketSim's by at most 9e-7 per element.
- **The quantized level saves another 30–34%.** Its worst errors are 0.005 UU, 0.005 UU/s and 5e-5 rad/s. The coarse
  level adds only about 8% more, so one quantized level is enough.

**For 140,000 replays** (1.23 MB per replay on average, about 172 GB of replays):

| Output | Total |
| --- | ---: |
| v1 Parquet | about 2.8 TB |
| lossless v2 | about 380–460 GB |
| quantized v2 | about 280–310 GB |

**Time.**

| Stage | Time per replay |
| --- | --- |
| Parse, decode, update ticks | under 0.1 s |
| Simulation with given lags, no fits | 0.15–0.32 s |
| Full default conversion | 2.3–5.5 s |

The fits take 90–95% of a conversion. This is why resimulation from the `resimulation` group (section 3.3) is cheap:
35,000–70,000 frames per second.

## 3. Outputs

### 3.1 One format, column groups

**There is one file format.** A replicar file is a single ordinary Parquet file with one row per replay frame
**in a play segment**: from the frame where the kickoff countdown has ended and the cars can move to the frame
that reports the goal (or the last frame in play, when regulation ends without one). This is v1's episode rule
(`labels.rs:10-23`), so the definition and its tests exist already. `--all-frames` adds the frames outside play
(countdowns, goal celebrations and replays) with a null `segment`; the simulation does not change, because v1
already simulates only in play (`conversion/mod.rs:1281`: a frame is simulated only when it and the previous one are `Active`), and those rows were 16–23% of frames. Its header
(players, teams, versions, configuration, diagnostics, pad layout) is JSON in the file's key-value metadata. What a
file contains is a choice of **column groups**, and the header lists the groups it has. Every name in this section
is defined in `docs/glossary.md`.

| Group | Default | Contents |
| --- | --- | --- |
| (always) | yes | `frame`, `segment`, `replay_time`, `replay_tick`, `sim_tick` |
| `state` | yes | ball and car physics, car internals (what restoration needs), controls, boost, pads, car status |
| `game` | yes | scoreboard, events, ball contacts and boost pickups (as `list<struct>` columns) |
| `updates` | yes | whether each body was updated, its update tick, ticks and seconds since its last update, ping |
| `future` | yes | the future-derived `future_` columns: next goal's team, seconds until it, seconds until the segment ends |
| `network` | no | the replay's network feed as replicar decodes it: each value with the frame of its last change |
| `resimulation` | no | what the fitted inference chose; with the replay, enough to resimulate `state` exactly (3.3) |
| `diagnostics` | no | prediction errors before each correction, simulated touches, RocketSim's own events |

`--precision float32|quantized` sets how `state` is stored:
- `float32`: as RocketSim has the values, with rotations as unit quaternions.
- `quantized`: the integers of section 2.

The scale of each quantized column is in its field metadata, and every reader decodes it to float32
transparently. So a user never sees the difference except in size and the last decimals.

This keeps the code simple. There is one schema module, one writer that skips the groups it was not asked for,
and one reader. The schema is self-describing (Arrow types, field metadata with units and scales, a format
version), so a plain `pyarrow.parquet.read_table` is enough to use a file. The `replicar` reader only adds
convenience (NumPy arrays, decoding of quantized columns, the header).

### 3.2 The usual files

| Use | Flags | Contains | Reading needs |
| --- | --- | --- | --- |
| **default** | none | the default groups | pyarrow (or any Parquet reader) |
| small | `--precision quantized` | the same, quantized state | pyarrow |
| resimulable | `--with resimulation` | the default groups + `resimulation` | pyarrow; replicar + the replay only to resimulate |
| without states | `--groups game,updates,future,resimulation` | no `state` | replicar + the replay |
| research | `--with network,diagnostics --all-frames` | more | pyarrow |

`--with` adds groups to the default set; `--groups` names the whole set.

The **default needs neither the replay parser nor the simulator to read**. A file without `state` is filled in
from the replay: `replicar.read("x.parquet", replay="x.replay")` resimulates the missing group and returns the
same object a full file gives. Python users therefore use one function whichever file they have.

### 3.3 The resimulation group

The `resimulation` group holds what the fitted inference chose, per frame, so that the replay can be resimulated
without fitting:
- the update ticks;
- the times observed control changes took effect;
- the presses (jump, dodge with direction and flip cancel);
- the air-control segments;
- the other state edits (dodge starts, contact-aligned ticks).

These are `resim_*` columns, using `list<struct>` where a frame has several values. The header records the
replicar and RocketSim versions, the configuration, and a checksum of a few resimulated states. A build that would
diverge then refuses the file instead of silently producing other states.

Its size is unknown until written. v1's `fitted_inputs` and `packet_lags` tables hold part of it and are
0.13–0.26 MB per replay **[verify]** (story 5.5). Its limits: resimulation needs the same replicar and RocketSim
version, the replay, and 0.15–0.3 s of CPU per replay.

**What rlpr shows about this idea.** RocketSim's own `.rlpr` recordings (`rocketsim_test/src/rlpr/mod.rs`)
store full 120 Hz per-tick car and ball records: 604 bytes per car in v9, including wheel suspension and impulse
records. They are zstd-compressed, about 30 MB for 300 s of 3v3. Their replay mode
(`examples/rlpr_replay/mod.rs`, a struct named `ReplayPlan`) does what resimulation does: it feeds each tick's
recorded controls and restores state only at discontinuities (kickoffs, demolition respawns). So "inputs plus
resets" is RocketSim's own way to reproduce a run. Three lessons, and one thing not to copy:
- **Versioned fields with explicit "unknown".** rlpr gates each later field by format version and gives older
  files a sentinel (`TOUCH_FRAME_UNKNOWN`), never a fake default. Our equivalent is Parquet nulls, a format
  version in the header, and new columns only appended.
- **Validation before use.** rlpr checks magic, endianness, version, sizes, bool bytes and a bounded zstd output
  before trusting a file. Our reader checks the format version, the group list and the replay hash (for
  resimulation), and refuses rather than guesses.
- **Restoration is not continuation.** rlpr restores wheel and contact internals that `ArenaState` does not
  expose. That is why v1 says a restored live arena is not an exact continuation (RESULTS.md, "Soccar state
  restoration"). Resimulation starts from the beginning of the replay, so it never needs those internals.
- **Not copied: the raw C struct dump.** It is fast to read from C++ or Rust, but layout-dependent and hard to
  read from anything else. Parquet with Arrow types is about as compact (rlpr compresses its whole file with
  zstd too) and readable everywhere.

### 3.4 Where files go, and who can read them

**One replay, one file.** `replicar convert match.replay -o match.parquet` writes exactly that file, and nothing
beside it: v1's seven record tables (`match.touches.parquet`, ...) are columns now. The file is self-contained
(its header has the players, versions, configuration and pad layout), so a folder per conversion would add a
level without adding anything. The file is written under a temporary name in the same directory and renamed
when complete, so a failed run leaves no partial file.

**Plain `.parquet`.** A replicar file is an ordinary Parquet file, and the extension should say so: every tool
recognizes it, and the header says it is a replicar file (format version, groups). A double extension would
only help someone who cannot open the file. To keep "any Parquet reader" true, the schema uses only types that
mainstream readers handle: numbers, booleans, strings, lists and structs. It uses no nullable fixed-size lists,
because Parquet drops the child values of a null list and PyArrow then reads them wrongly (RESULTS.md, "Parquet
columns and record tables"); a missing player's values are NaN or -1 inside a present list, plus `car_status`
`absent`. **[verify]** with pyarrow, polars and DuckDB in story 7.1.

**A corpus**

`replicar convert replays/ -o out/ --jobs N` writes one file per replay plus `out/index.parquet`, mirroring
the input's subfolders (`replays/2v2/x.replay` becomes `out/2v2/x.parquet`). The index has
one row per replay: file name, SHA-256, status or error, frames, duration, map, game size, players and teams,
final score, and the groups and precision written.
- A failed replay becomes a row with its error, not an aborted batch.
- `--skip-existing` resumes an interrupted run.

## 4. The code, from the ground up

### 4.1 Pipeline

```
bytes --parse----> boxcars::Replay
      --decode---> NetworkFrames     the network values per frame with their last-change frames; car lives; events
      --time-----> UpdateTicks       each update's tick (update chains, ball runs, contact alignment)
      --run------> for each frame:   the Simulator applies updates at their ticks and steps RocketSim;
                                     where v1 fits, it asks the Inference (fitted, or recorded)
      --annotate-> ball contacts, pickups, updates, scoreboard, future, play segments
      --write----> one file, the column groups asked for
```

Each arrow is a module with a function or a small struct, and each stage's output is a plain data type that can
be tested and printed. Compare v1:
- `run` and most of `annotate` are one function of about 1,830 lines (`conversion/mod.rs:1065-2897`), with about
  40 mutable locals declared before its loop (`1101-1257`).
- Contact alignment is a recursive call of that function (`1075-1085`).

### 4.2 Inference and simulator: why both outputs stay simple

The **simulator** owns the arena and the player table. It does what RocketSim needs and nothing more:
- applies each update at its update tick;
- holds or releases cars (spawn poses, dead shells, observed demolitions);
- reconciles pads, steps ticks, and applies the controls it is given.

It has no fitting and no lookahead.

The **inference** answers the simulator's questions at the moments v1's fits run:
- when a ground control change takes effect;
- when a jump or dodge was pressed, with what direction and cancel;
- which air controls to fly between two updates;
- which tick the first update after a dodge belongs to.

There is one trait with two implementations:
- **`FittedInference`**: v1's fits (`fits.rs`, `air.rs`, the jump/dodge logic of the loop), each in its own module.
  It reads the future only through a `Lookahead` view that knows which frames a masked evaluation withholds.
- **`RecordedInference`**: answers from a file's `resimulation` group.

Every answer is a value of one enum, `Choice`. Writing the `resimulation` group means writing the answers down as
they are given; resimulating means handing them back. So there is no second simulation path, no special mode in
the simulator, and nothing for the two to disagree about. A run without fits is the simulator with an inference
that never changes anything. Masked evaluation is an inference configuration (`Holdout`).

The v1 evidence that the split is clean: the main arena changes only through explicit calls (`set_car_state`,
`set_car_controls`, `set_ball_state`, `set_boost_pad_state`, `add_car`: 21 call sites in `conversion/mod.rs`
before line 2898, plus `step_ticks` and `limit_reported_velocities`, which use the same calls). The fits run in
scratch arenas and return values.

**[verify]** that RocketSim is deterministic for identical inputs across processes (story 4.4).

### 4.3 Types

- **Newtypes for identity and time:** `PlayerIndex`, `ActorId`, `CarLife { actor, created_frame }`, `PlayerKey`,
  `ReplayTick`, `SimTick`, `FrameIndex`. In v1 they are all `usize` or `i32`, and a tick-rebase bug lived in
  that gap (RESULTS.md, "Audit fix batch 5").
- **Per-car state in one struct:** `CarTrack` per car life and `PlayerState` per player. They replace v1's maps keyed
  by `(actor, created_frame)` tuples (`conversion/mod.rs:1105-1257`): `spawn_started`, `spawn_held`,
  `spawn_demolished`, `gated_jump_active`, `last_dodge_raw`, `last_double_raw`, `last_counters`,
  `ground_counters`, `car_shifts`, `handled_dodges`, `flip_cache`, `flip_last`, `lag_overrides` (a `RefCell`
  today), `dead_shells`, `demo_hold_until`, `last_contact_tick` and `slot_bodies`.
- **Enums for v1's strings:** `LagSource`, `FittedInput`, `HoldSource`, `GameState`. The game state is compared
  as `== "Active"` today (`ball_evidence.rs:81`).
- **Network property names as constants** (44 inline literals in `observations.rs`), as VirxEC does.
- **One `thiserror` error type.** No `Box<dyn Error>` or string errors in the library.

### 4.4 Crates and packages: the dependency split

```
crates/replicar-format   the schema (column groups, encodings, precision), the writer and the reader;
                         depends on arrow/parquet only; no boxcars, no RocketSim
crates/replicar          decode, time, simulate, infer, annotate; depends on replicar-format, boxcars, rocketsim
crates/replicar-cli      the `replicar` binary: convert (file or folder), resimulate, inspect, verify
crates/replicar-python   pyo3 module (maturin) exposing convert / resimulate
crates/replicar-eval     publish = false: evaluate_corpus, error_budget, rlbot_*, dump_reconstruction, the
                         consistency checks, the v1 parity checks; the sealed test-split guard
python/replicar          pure-Python reader (pyarrow + numpy), installed alone
```

- **Python.** `pip install replicar` installs the pure-Python reader: any platform, no Rust, no simulator.
  `pip install replicar[convert]` adds the native wheel that converts and resimulates. `replicar.read` imports
  the native module only when a file needs resimulating, and otherwise never touches it.
- **Rust.** A user who only reads depends on `replicar-format`.
- **One schema.** The Python reader follows the schema the files describe themselves (types, field metadata,
  format version) instead of a second hand-written list of columns. A test checks it against `replicar-format`
  on generated files.

Other conventions:
- No function over about 150 lines.
- Tests sit next to their code; 41 of v1's 91 tests are in `conversion/mod.rs`, testing code in other files.
- From VirxEC: a `Converter` builder, the `rocketsim` re-export instead of a separate `glam` pin, crate docs that
  start with the timing model and the event streams, `examples/`, a criterion benchmark, a public-API test.

### 4.5 Using it

```rust
let meshes = replicar::Meshes::load("collision_meshes")?;      // checks the soccar meshes; rocketsim::init once
let converter = replicar::Converter::new(&meshes, Config::default());
let output = Output::default().with(Group::Resimulation).precision(Precision::Quantized);   // groups and precision
converter.convert_to_file(&bytes, "match.parquet", &output)?;  // one file, written atomically
let replay = converter.convert(&bytes)?;                       // or frames in memory
let frames = replicar_format::read("match.parquet")?;          // reading needs no simulator
```

```
replicar convert match.replay -o match.parquet [--precision float32|quantized] [--with GROUPS | --groups GROUPS] [--all-frames]
replicar convert replays/ -o out/ --jobs 16 [--skip-existing] ...   # folder: one file per replay + index.parquet
replicar resimulate match.parquet --replay match.replay -o full.parquet
replicar inspect match.replay | match.parquet
replicar verify match.parquet
  --meshes DIR, else $REPLICAR_MESHES, else ./collision_meshes
```

```python
import replicar                                    # pure Python: reading
f = replicar.read("match.parquet")                 # .header, .arrays() (NumPy float32), .records() (pyarrow)
f = replicar.read("no_states.parquet", replay="match.replay")      # resimulates: needs replicar[convert]
for batch in replicar.iter_frames("match.parquet", replay="match.replay", batch_frames=4096): ...
replicar.convert("match.replay", "match.parquet", precision="quantized")            # replicar[convert]
replicar.convert_many(paths, "out/", jobs=16)
```

## 5. Proving it is the same reconstruction

v1 stays the oracle. `replicar-eval` depends on v1 at the tag `v1-final` through a renamed git dependency (e.g.
`replicar_v1 = { package = "replicar", git = ..., tag = "v1-final" }`). Both depend on the same RocketSim
revision, so the comparison runs in one process. **[verify]** that cargo unifies the RocketSim dependency and that
`rocketsim::init` tolerates being called a second time.

1. **States, bit for bit, in memory.** For each of the 120 train and validation replays, v2 must equal v1 exactly
   on:
   - every frame's ball and car state (physics, timers, flags, controls), the pad states and the arena tick;
   - the records, the scoreboard, the play segments and the future columns.

   This compares the converters themselves, before the file encoding, on every frame (play segments or not).
2. **A ladder of configurations**, so that a difference points at one part:
   - the simulator without fits, against v1 with `input_fits`, `air_bvp`, `infer_air_controls_from_lookahead` and
     `align_contacts` off;
   - then each fit switched on;
   - then the defaults;
   - then the evaluators' held-out variants.

   v1 already has an option for every rung (`conversion/mod.rs:57-114`).
3. **Evaluator reports.** `evaluate_corpus`, ported to v2, prints the same numbers as v1's reference run.
4. **Files.**
   - `float32`: rotations within 1e-6 and everything else bit-identical after a write and a read.
   - `quantized`: every value within its quantum.
   - The `resimulation` group: resimulated states bit-identical to the conversion that wrote it, in another process,
     and on a second machine if one is available **[verify]**.
5. **Size and speed** on the fixed replay list: size per group and precision; conversion, resimulation and read
   time; mean and maximum against v1.

An intentional change of output goes in a separate commit with the full protocol: a paired `run_reference.sh`
run; counts and p50/p90/p99 per game size and per replay; the decision made on validation; an entry in RESULTS.md.
The test split stays sealed.

## 6. Stories

Each story is one reviewable commit that passes `cargo test --all-targets` and the parity check of its rung.
C is the complexity. Work happens on a `v2` branch, and the record files are updated at each milestone.

| # | Story | Acceptance | C |
| --- | --- | --- | --- |
| **0** | **Baseline** | | |
| 0.1 | RocketSim to the latest compatible revision (crates.io 0.2.7 or `v3-rust` `dc5d60e`; **[verify]** which is newer and whether they are the same code); ROCKETSIM_NOTES re-run | measured like the 2026-10-04 update | M |
| 0.2 | Tag `v1-final`; v1 reference reports on train and validation; speed baseline | recorded in RESULTS.md | L |
| 0.3 | Determinism: run v1 twice, and alongside `cargo test` (7 headers differed once: RESULTS.md, "Release clean-up") | equal, or the cause found | M |
| **1** | **Skeleton** | | |
| 1.1 | Workspace and crates, `thiserror` error, newtypes | builds; unit tests | M |
| 1.2 | `replicar-eval parity` harness against `replicar_v1` | v1 compared with itself is equal | M |
| **2** | **Observe** | | |
| 2.1 | Attribute constants; actor graph and lifetimes | `hash_observations` equal to v1 on 120 replays | H |
| 2.2 | Players, teams, events, pads, header and diagnostics | same | M |
| **3** | **Time** | | |
| 3.1 | Update chains, lag-free detection, ball runs (v1 `packet_lags.rs`) | update ticks equal to v1 on 120 replays | H |
| 3.2 | Contact alignment as a stage (`contact_alignment.rs`, `ball_evidence.rs`) | aligned lags equal | H |
| **4** | **Execute** | | |
| 4.1 | Simulator: players, updates at their ticks, stepping | rung "no fits" equal | H |
| 4.2 | Car status: spawning cars, wrecks, observed demolitions, sleeping updates | rung "no fits" equal, including the hold lists | H |
| 4.3 | Actions: counters, jump gate, dodge refresh; pads | rung "no fits" equal | M |
| 4.4 | Determinism of the simulator across processes | equal states on 120 replays | L |
| **5** | **Inference** | | |
| 5.1 | `Inference` trait, `Choice`, `Lookahead` with withheld frames; air lookahead and persistence | rung "air lookahead" equal; the masked leak test passes | M |
| 5.2 | Air boundary-value solve (`air.rs`) | rung "air_bvp" equal | H |
| 5.3 | Input fits: ground timing, jump, dodge start and first-update tick, flip cancel (`fits.rs`) | rung "input_fits" equal, then the defaults | H |
| 5.4 | Held-out variants | evaluator rungs equal | M |
| 5.5 | `RecordedInference`; inferences recorded and replayed in memory | replayed equals recorded on 120 replays; volume measured | M |
| **6** | **Annotate** | | |
| 6.1 | Touches, ball contacts, boost pickups | records equal | M |
| 6.2 | Scoreboard, updates, play segments, future | equal | M |
| **7** | **Format** (`replicar-format`) | | |
| 7.1 | Schema with column groups, play-segment rows and `--all-frames`, `float32` precision, writer, atomic write | values equal to v1's Parquet columns on the same frames; opens in pyarrow, polars and DuckDB; sizes recorded | M |
| 7.2 | `quantized` precision with scales in field metadata | errors within the quantum; sizes recorded | M |
| 7.3 | `resimulation` group; resimulate from a file | resimulated equals recorded, from a file, in another process | M |
| 7.4 | `network` and `diagnostics` groups | match v1's `frame_json` content | M |
| 7.5 | Rust reader; restoration from a file (quaternion tolerance) and by resimulation (exact) | all frames | M |
| **8** | **Front ends** | | |
| 8.1 | CLI: convert (file, or folder with `index.parquet`), resimulate, inspect, verify | train converted as a folder equals file-by-file | M |
| 8.2 | Python reader (pure): read, arrays, records, header, quantized decoding | equal to the Rust reader on generated files | M |
| 8.3 | Python native extra: convert, convert_many, resimulate, `iter_frames`; type stubs; wheels | Python tests on 3.12 and newer | M |
| **9** | **Evaluation and release** | | |
| 9.1 | Port `evaluate_corpus` and the tools the protocol uses to `replicar-eval` | reports equal to v1's reference | H |
| 9.2 | Docs (glossary, concepts, file format, evaluation, contributing), examples, benchmark; a test that every schema column, flag and public type is in the glossary | `cargo doc` clean; glossary test passes | M |
| 9.3 | Final check (section 5), CHANGELOG 2.0.0, crates.io and PyPI | all of section 5 | M |

Speed gains taken along the way, where they cost nothing in output:
- the folder mode converts replays in parallel;
- the `resimulation` group removes the fits from every later use;
- contact alignment's first pass may not need to rebuild what the second pass rebuilds **[verify]**
  (`conversion/mod.rs:1075-1085`).

The deeper gains wait for parity; the air boundary-value solve is the largest single cost.

**Later, not in v2.0:**
- an rlpr exporter for RocketSim's developers, so that a ROCKETSIM_NOTES reproduction comes in their own format;
- modes and mutators after VirxEC's `arena_config.rs`;
- sharded corpus files.

## 7. Risks

- **A rewrite drifts.** The parity ladder is the guard: no story is done while its rung differs from v1, and each
  rung isolates one part.
- **Floating-point order.** Bit-identical states require the same operations in the same order. Ported code keeps
  its arithmetic as written, and differences are found rung by rung.
- **Determinism of resimulation.** If RocketSim is not deterministic across processes or machines, resimulation
  degrades to "same build, same machine". Story 4.4 finds out early, and the default output does not depend on it.
- **Cost.** This is a large project; nothing is released before it is complete (the user's choice, 2026-10-06).

## 8. Names

Names are part of the design. Every user-facing name (column, group, flag, command, public type) is defined in
`docs/glossary.md`, together with the naming rules and the v1 to v2 mapping. A name that is not in the glossary
does not ship: story 9.2 adds a test that checks the file schema against it. The main changes from v1:
- **One word per concept.** v1 used "packet", "packet lag", "fresh" and "update age" for one idea; v2 uses
  *update*, *update tick*, *updated* and *ticks/seconds since update*.
- **Ticks say what they count.** `replay_tick` is the replay's whole timeline; `sim_tick` is RocketSim's
  arena count, which stops at pauses.
- **The player is the index.** Per-car arrays are indexed by player, since a player's car changes over a match
  (v1: "car slot").
- **Default names go to the preferred meaning.** `ball_contacts` are found from the ball's motion (the best touch
  evidence); RocketSim's are `simulated_touches`.
- **Internal jargon stays internal.** "Dead pawn shell" and "spawn pose held" become one `car_status` with
  `absent`, `active`, `spawning` and `demolished`, plus `car_status_inferred`.
- **Code names match the docs.** The inference (fitted or recorded) and the simulator, not "planner" and
  "executor"; `resimulate`, not "regenerate".

## 9. Questions for the user

1. **The `future` group's name.** `future` puts the warning in every column name (`future_goal_team`); the
   alternative `outcome` (`outcome_goal_team`) reads more naturally as a training target but hides where the value
   comes from. Recommendation: `future`.
2. **A play segment that ends without a goal** (regulation running out): it is kept, with `future_goal_team`
   null or naming the overtime goal's team (v1 looks across segments to the next goal). Recommendation: keep v1's
   rule, so the next goal is the next goal wherever it is, and `future_seconds_until_segment_end` marks the
   segment's own end.
