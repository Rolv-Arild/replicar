# replicar v2: design and migration plan

Status: proposal, third draft, 2026-10-06, branch `v2-plan`. Nothing here is implemented.

Earlier drafts:
- `ac4b8c4`: a refactor of v1 in place;
- `95b0fd6`: the first ground-up draft.

Sources and conventions:
- Facts about v1 cite files at `master` = `5a7da58`.
- Facts about VirxEC's converter cite `external/replay-to-rocketsim` (`bc61b66`).
- Facts about RocketSim's rlpr format cite `rocketsim_test/src/rlpr/` at our pin `9910c58`.
- The measurements are in RESULTS.md, "Output size and conversion cost (2026-10-06)" and "v2 encodings:
  rotation and lossy state (2026-10-06)", with the scripts that reproduce them.
- **[decide]** marks a choice for the user, and **[verify]** an assumption a story must check before relying on it.

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
- **Decisions from the second round:**
  - **States by default; replay + plan as an option**, and both without making the code a mess. Not every user
    wants the cost, or the dependency, of running the parser and the simulator.
  - **Quaternions** for rotations; their precision is plenty.
  - **A lossy option** if it saves a substantial amount of space. The reconstruction error is far larger than any
    reasonable quantization.

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
| **lossy: 0.01 UU, 0.01 UU/s, 1e-4 rad/s, int16 quaternion** | **1.31** | **1.33** | **2.88** | **1.6–1.8** |
| lossy coarse: 0.1 UU, 0.1 UU/s, 1e-3 rad/s | 1.21 | 1.23 | 2.64 | 1.5–1.6 |

- **Rotations.** They are about a third of a matrix-based file, because a matrix is 9 floats per body with
  incompressible low bits. A quaternion cuts the whole file by about 21%. A matrix rebuilt from it differs from
  RocketSim's by at most 9e-7 per element.
- **The lossy level saves another 30–34%.** Its worst errors are 0.005 UU, 0.005 UU/s and 5e-5 rad/s. The coarse
  level adds only about 8% more, so one lossy level is enough.

**For 140,000 replays** (1.23 MB per replay on average, about 172 GB of replays):

| Output | Total |
| --- | ---: |
| v1 Parquet | about 2.8 TB |
| lossless v2 | about 380–460 GB |
| lossy v2 | about 280–310 GB |

**Time.**

| Stage | Time per replay |
| --- | --- |
| Parse, extract, packet lags | under 0.1 s |
| Simulation with given lags, no fits | 0.15–0.32 s |
| Full default conversion | 2.3–5.5 s |

The fits take 90–95% of a conversion. This is why a plan file (section 3.3) makes regeneration cheap:
35,000–70,000 frames per second.

## 3. Outputs

### 3.1 One format, column groups

**There is one file format.** A replicar file is a single Parquet file with one row per replay frame. Its replay
header (players, teams, slots, versions, configuration, diagnostics, pad layout) is JSON in the file's key-value
metadata. What a file contains is a choice of **column groups**, and the metadata lists the groups it has:

| Group | Default | Contents |
| --- | --- | --- |
| `state` | yes | ball, cars (including the internals that restoration needs), controls, pads |
| `match` | yes | time, scoreboard, events and the other records as `list<struct>` columns, freshness |
| `labels` | yes | future-derived training labels, flagged as such |
| `observations` | no | the replay's own field values with their update frames (what v1 kept only in `frame_json`) |
| `plan` | no | the fits' decisions (section 3.3) |
| `diagnostics` | no | position residuals and other evaluator data |

`--precision exact|compact` sets how `state` is stored:
- `exact`: float32, rotations as quaternions.
- `compact`: the lossy integers of section 2.

The scale of each quantized column is in its field metadata, and every reader decodes it to float32
transparently. So a user never sees the difference except in size and the last decimals.

This keeps the code simple. There is one schema module, one writer that skips the groups it was not asked for,
and one reader. The schema is self-describing (Arrow types, field metadata with units and scales, a format
version), so a plain `pyarrow.parquet.read_table` is enough to use a file. The `replicar` reader only adds
convenience (NumPy arrays, decoding of quantized columns, the header).

### 3.2 The usual files

| Use | Flags | Contains | Reading needs |
| --- | --- | --- | --- |
| **default** | none | `state` + `match` + `labels` | pyarrow (or any Parquet reader) |
| small | `--precision compact` | the same, lossy state | pyarrow |
| regenerable | `--plan` | the default groups + `plan` | pyarrow; replicar + the replay only to regenerate |
| plan only | `--groups match,plan` | no `state` | replicar + the replay |
| research | `--groups +observations,+diagnostics` | everything | pyarrow |

The **default needs neither the parser nor the simulator to read**. A file without `state` is filled in by
`replicar` from the replay: `replicar.read("x.parquet", replay="x.replay")` regenerates the missing group and
returns the same object a full file gives. Python users therefore use one function whichever file they have.

### 3.3 The plan group

The plan group holds what the fits decided, per frame (a `plan_*` set of columns), so that the replay can be
regenerated without fitting:
- the packet ticks: u8 lags per object;
- the timed control changes and presses per car, and the air-control segments, as `list<struct>` with ticks;
- the state edits the fits made (dodge starts and cancels, lag overrides).

The header records the replicar and RocketSim versions, the configuration, and a checksum of a few regenerated
states. A build that would diverge then refuses the file instead of silently producing other states.

Its size is unknown until written. v1's `fitted_inputs` and `packet_lags` tables hold part of it and are
0.13–0.26 MB per replay **[verify]** (story 5.5). Its limits: regeneration needs the same replicar and RocketSim
version, the replay, and 0.15–0.3 s of CPU per replay.

**What rlpr shows about this idea.** RocketSim's own `.rlpr` recordings
(`rocketsim_test/src/rlpr/mod.rs`) store full 120 Hz per-tick car and ball records: 604 bytes per car in v9,
including wheel suspension and impulse records. They are zstd-compressed, about 30 MB for 300 s of 3v3. Their
replay mode (`examples/rlpr_replay/mod.rs`, a struct named `ReplayPlan`) does what our plan group does: it feeds
each tick's recorded controls and restores state only at discontinuities (kickoffs, demolition respawns). So
"controls plus resets" is RocketSim's own way to reproduce a run. Three lessons, and one thing not to copy:
- **Versioned fields with explicit "unknown".** rlpr gates each later field by format version and gives older
  files a sentinel (`TOUCH_FRAME_UNKNOWN`), never a fake default. Our equivalent is Parquet nulls, a format
  version in the metadata, and new columns only appended.
- **Validation before use.** rlpr checks magic, endianness, version, sizes, bool bytes and a bounded zstd output
  before trusting a file. Our reader checks the format version, the group list and the replay hash (for a plan),
  and refuses rather than guesses.
- **Restoration is not continuation.** rlpr restores wheel and contact internals that `ArenaState` does not
  expose. That is why v1 says a restored live arena is not an exact continuation (RESULTS.md, "Soccar state
  restoration"). A plan regenerates from the start of the replay, so it never needs those internals.
- **Not copied: the raw C struct dump.** It is fast to read from C++ or Rust, but layout-dependent and hard to
  read from anything else, which is the user's point. Parquet with Arrow types gives the same compactness
  (rlpr compresses its whole file with zstd too) and stays readable everywhere.

### 3.4 A corpus

`replicar convert replays/ -o out/ --jobs N` writes one file per replay plus `out/index.parquet`. The index has
one row per replay: file name, SHA-256, status or error, frames, duration, map, game size, players and teams,
final score, and the groups and precision written.
- A failed replay becomes a row with its error, not an aborted batch.
- `--skip-existing` resumes an interrupted run.

## 4. The code, from the ground up

### 4.1 Pipeline

```
bytes --parse----> boxcars::Replay
      --observe--> Observations      typed replay fields per frame with update frames; actor lifetimes; events
      --time-----> PacketTimeline    each fresh packet's server tick (lag chains, ball runs, contact alignment)
      --run------> for each frame:   Executor applies packets at their ticks and steps RocketSim;
                                     at each decision point it asks a Planner (fitting, or reading a plan)
      --annotate-> touches, ball contacts, pickups, freshness, scoreboard, labels
      --write----> one file, the column groups asked for
```

Each arrow is a module with a function or a small struct, and each stage's output is a plain data type that can
be tested and printed. Compare v1:
- `run` and most of `annotate` are one function of about 1,830 lines (`conversion/mod.rs:1065-2897`), with about
  40 mutable locals declared before its loop (`1101-1257`).
- Contact alignment is a recursive call of that function (`1075-1085`).

### 4.2 Planner and executor: why both outputs stay simple

The **executor** owns the arena and the slot table. It does what RocketSim needs and nothing more:
- applies each packet's body at its tick;
- holds or releases cars (spawn poses, dead shells, observed demolitions);
- reconciles pads, steps ticks, and applies the controls it is given.

It has no fitting and no lookahead.

The **planner** answers the executor's questions at the moments v1's fits run:
- when a ground control change takes effect;
- when a jump or dodge was pressed, with what direction and cancel;
- which air controls to fly between two packets;
- which tick the first packet after a dodge belongs to.

There is one trait with two implementations:
- **`FittingPlanner`**: v1's fits (`fits.rs`, `air.rs`, the jump/dodge logic of the loop), each in its own module.
  It reads the future only through a `Lookahead` view that knows which frames a masked evaluation withholds.
- **`RecordedPlanner`**: answers from a plan group.

Every answer is a value of one enum, `Decision`. Writing the `plan` group means writing the decisions down as
they are made; reading it means handing them back. So there is no second simulation path, no "recipe mode" in
the executor, and nothing for the two to disagree about. A run without fits is the executor with a planner that
never changes anything. Masked evaluation is a planner configuration (`Holdout`).

The v1 evidence that the split is clean: the main arena changes only through explicit calls (`set_car_state`,
`set_car_controls`, `set_ball_state`, `set_boost_pad_state`, `add_car`: 21 call sites in `conversion/mod.rs`
before line 2898, plus `step_ticks` and `limit_reported_velocities`, which use the same calls). The fits run in
scratch arenas and return values.

**[verify]** that RocketSim is deterministic for identical inputs across processes (story 4.4).

### 4.3 Types

- **Newtypes for identity and time:** `Slot`, `ActorId`, `Lifetime { actor, created_frame }`, `PlayerKey`,
  `TimelineTick`, `ArenaTick`, `FrameIndex`. In v1 they are all `usize` or `i32`, and a tick-rebase bug lived in
  that gap (RESULTS.md, "Audit fix batch 5").
- **Per-car state in one struct:** `CarTrack` per lifetime and `SlotState` per slot. They replace v1's maps keyed
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
crates/replicar          observe, time, execute, plan, annotate; depends on replicar-format, boxcars, rocketsim
crates/replicar-cli      the `replicar` binary: convert (file or folder), regenerate, inspect, verify
crates/replicar-python   pyo3 module (maturin) exposing convert / regenerate
crates/replicar-eval     publish = false: evaluate_corpus, error_budget, rlbot_*, dump_reconstruction, the
                         consistency checks, the v1 parity checks; the sealed test-split guard
python/replicar          pure-Python reader (pyarrow + numpy), installed alone
```

- **Python.** `pip install replicar` installs the pure-Python reader: any platform, no Rust, no simulator.
  `pip install replicar[convert]` adds the native wheel that converts and regenerates. `replicar.read` imports
  the native module only when a file needs regenerating, and otherwise never touches it.
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
let output = Output::default().with_plan().precision(Precision::Compact);   // groups and precision
converter.convert_to_file(&bytes, "match.parquet", &output)?;  // one file, written atomically
let replay = converter.convert(&bytes)?;                       // or frames in memory
let frames = replicar_format::read("match.parquet")?;          // reading needs no simulator
```

```
replicar convert match.replay -o match.parquet [--precision exact|compact] [--plan] [--groups ...]
replicar convert replays/ -o out/ --jobs 16 [--skip-existing] ...   # folder: one file per replay + index.parquet
replicar regenerate match.parquet --replay match.replay -o full.parquet
replicar inspect match.replay | match.parquet
replicar verify match.parquet
  --meshes DIR, else $REPLICAR_MESHES, else ./collision_meshes
```

```python
import replicar                                    # pure Python: reading
f = replicar.read("match.parquet")                 # .header, .arrays() (NumPy float32), .records() (pyarrow)
f = replicar.read("plan_only.parquet", replay="match.replay")      # regenerates: needs replicar[convert]
for batch in replicar.iter_frames("match.parquet", replay="match.replay", batch_frames=4096): ...
replicar.convert("match.replay", "match.parquet", precision="compact")              # replicar[convert]
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
   - the records, the scoreboard and the labels.

   This compares the converters themselves, before the file encoding.
2. **A ladder of configurations**, so that a difference points at one part:
   - the executor without fits, against v1 with `input_fits`, `air_bvp`, `infer_air_controls_from_lookahead` and
     `align_contacts` off;
   - then each fit switched on;
   - then the defaults;
   - then the evaluators' held-out variants.

   v1 already has an option for every rung (`conversion/mod.rs:57-114`).
3. **Evaluator reports.** `evaluate_corpus`, ported to v2, prints the same numbers as v1's reference run.
4. **Files.**
   - `exact`: rotations within 1e-6 and everything else bit-identical after a write and a read.
   - `compact`: every value within its quantum.
   - A plan file: regenerated states bit-identical to the conversion that wrote it, in another process, and on
     a second machine if one is available **[verify]**.
5. **Size and speed** on the fixed replay list: size per group and precision; conversion, regeneration and read
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
| 3.1 | Packet chains, lag-free detection, ball runs (`packet_lags.rs`) | lags equal to v1 on 120 replays | H |
| 3.2 | Contact alignment as a stage (`contact_alignment.rs`, `ball_evidence.rs`) | aligned lags equal | H |
| **4** | **Execute** | | |
| 4.1 | Executor: slots, packets at their ticks, stepping | rung "no fits" equal | H |
| 4.2 | Lifecycle: spawn holds, dead shells, observed demolitions, sleeping packets | rung "no fits" equal, including the hold lists | H |
| 4.3 | Actions: counters, jump gate, dodge refresh; pads | rung "no fits" equal | M |
| 4.4 | Determinism of the executor across processes | equal states on 120 replays | L |
| **5** | **Plan** | | |
| 5.1 | `Planner` trait, `Decision`, `Lookahead` with withheld frames; air lookahead and persistence | rung "air lookahead" equal; the masked leak test passes | M |
| 5.2 | Air boundary-value solve (`air.rs`) | rung "air_bvp" equal | H |
| 5.3 | Input fits: ground timing, jump, dodge start and first-packet tick, flip cancel (`fits.rs`) | rung "input_fits" equal, then the defaults | H |
| 5.4 | Held-out variants | evaluator rungs equal | M |
| 5.5 | `RecordedPlanner`; decisions recorded and replayed in memory | replayed equals recorded on 120 replays; decision volume measured | M |
| **6** | **Annotate** | | |
| 6.1 | Touches, ball contacts, boost pickups | records equal | M |
| 6.2 | Scoreboard, freshness, labels | equal | M |
| **7** | **Format** (`replicar-format`) | | |
| 7.1 | Schema with column groups, `exact` precision, writer, atomic write | values equal to v1's Parquet columns; sizes recorded | M |
| 7.2 | `compact` precision with scales in field metadata | errors within the quantum; sizes recorded | M |
| 7.3 | `plan` group; regenerate from a file | regenerated equals recorded, from a file, in another process | M |
| 7.4 | `observations` and `diagnostics` groups | match v1's `frame_json` content | M |
| 7.5 | Rust reader; restoration from a file (quaternion tolerance) and from a plan (exact) | all frames | M |
| **8** | **Front ends** | | |
| 8.1 | CLI: convert (file, or folder with `index.parquet`), regenerate, inspect, verify | train converted as a folder equals file-by-file | M |
| 8.2 | Python reader (pure): read, arrays, records, header, compact decoding | equal to the Rust reader on generated files | M |
| 8.3 | Python native extra: convert, convert_many, regenerate, `iter_frames`; type stubs; wheels | Python tests on 3.12 and newer | M |
| **9** | **Evaluation and release** | | |
| 9.1 | Port `evaluate_corpus` and the tools the protocol uses to `replicar-eval` | reports equal to v1's reference | H |
| 9.2 | Docs (concepts, file format, evaluation, contributing), examples, benchmark | `cargo doc` clean | M |
| 9.3 | Final check (section 5), CHANGELOG 2.0.0, crates.io and PyPI | all of section 5 | M |

Speed gains taken along the way, where they cost nothing in output:
- the folder mode converts replays in parallel;
- a plan removes the fits from every later use;
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
- **Determinism of regeneration.** If RocketSim is not deterministic across processes or machines, plan files
  degrade to "same build, same machine". Story 4.4 finds out early, and the default output does not depend on it.
- **Cost.** This is a large project, but the milestones are useful on their own. After milestone 7 the new file
  format exists, and it can be written from v1's conversion as a stopgap if the user wants small files for the
  140,000 replays sooner: section 8, question 1.

## 8. Questions for the user

1. **Small files sooner?** The new format (milestone 7, with `exact` and `compact`) could be written from v1's
   conversion first, about two stories of work. You would get 6–10× smaller files for the 140,000 replays before
   the rewrite is done, at the cost of a writer that is later moved onto v2. Do you want that?
2. **Labels in the default groups?** They are cheap, but future-derived; or should they be opt-in?
