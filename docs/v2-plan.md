# replicar v2: design and migration plan

Status: proposal, second draft, 2026-10-06, branch `v2-plan`. Nothing here is implemented. The first draft
(`ac4b8c4`), which refactored v1 in place, is superseded. Facts about v1 cite files at `master` = `5a7da58`.
Facts about VirxEC's converter cite `external/replay-to-rocketsim` (`bc61b66`, git-ignored). The measurements
are in RESULTS.md, "Output size and conversion cost (2026-10-06)", with the scripts that reproduce them.
**[decide]** marks a choice for the user, and **[verify]** an assumption a story must check before relying on it.

## 1. What the user asked for (2026-10-06)

- **Outputs that are simple and small.** JSONL takes too much space, and the Parquet export is eight files per
  replay. The target corpus is about **140,000 replays**, so file size is a first-order constraint. The
  conversion is also too slow to embed in a streaming pipeline.
- **Code that is not opaque**, redesigned from the ground up.
- **The same accuracy.** Speed must be no worse (mean +3%, no replay +10%), and we take obvious gains along the
  way, but speed is not the focus yet.
- **Decisions already taken:**
  - The sealed-split guard moves out of the user-facing CLI into the evaluation tools.
  - `thiserror`, `clap`, `pyo3`/`maturin` and `criterion` may be added.
  - Aim for crates.io and PyPI releases.
  - Update RocketSim to the latest compatible revision **before** the work, and run every comparison on that
    one revision.
  - Leave the collision meshes to the user.

## 2. What the measurements say

Three train replays were measured, one per game size, converted with the v1 release build. RESULTS.md has the
table and the method.

**Size.**
- A v1 Parquet export is **15–17.5× the replay** (1v1: 12.45 MB for a 0.83 MB replay; 3v3: 28.1 MB for
  1.61 MB). JSONL is about 150×.
- **72% of the Parquet file is the `frame_json` column**, a complete JSON copy of every frame.
- Some of what only that column holds is needed. As typed columns:
  - the state fields still needed to restore a RocketSim car (timers, flags, previous controls): 0.2–0.6 MB;
  - the observations with their update frames: 0.9–2.8 MB;
  - the evaluator's position residuals: 1.2–2.7 MB.
- The rest of `frame_json` repeats what typed columns already hold, or repeats constants on every frame:
  boost-pad positions, and Dropshot and Heatseeker fields that never change in soccar.

| Lossless encoding of the state (typed columns) | 1v1 | 2v2 | 3v3 |
| --- | ---: | ---: | ---: |
| v1 file | 12.45 MB | 12.09 MB | 28.14 MB |
| without `frame_json` | 3.61 | 3.48 | 7.55 |
| + rotations as quaternions | 2.69 | 2.58 | 5.83 |
| + BYTE_STREAM_SPLIT float encoding, zstd 9 | 1.75 | 1.72 | 3.95 |
| + the remaining car-state fields | **1.97 (2.4× replay)** | **1.99 (2.6×)** | **4.51 (2.8×)** |

Quantizing car state to 0.01 UU and storing it as delta-coded integers halves the car columns again; this is
lossy (3v3 cars: 2.72 to 1.52 MB). Frames outside play (countdown, goal pause, kickoff hold) are 16–23% of all
frames and cost little, because they barely change.

**Time.**

| Stage | Time per replay |
| --- | --- |
| Parse | 0.01 s |
| Observation extraction | 0.02–0.04 s |
| Packet-lag inference | 0.01–0.04 s |
| Simulation with given lags, no fits | **0.15–0.32 s** |
| Full default conversion | **2.3–5.5 s** |

**About 90–95% of the time is the fits** (input timing, the air boundary-value solve, contact alignment), not
RocketSim.

**For 140,000 replays.** A replay averages 1.23 MB over the 120 development replays, so the replays come to about
172 GB. v1 Parquet would be about 2.8 TB, and a lossless v2 state file about 410–480 GB. Converting costs about
4 s × 140,000 = 150 CPU hours with today's fits.

**The second idea of this draft: store the fits' decisions, not the states.**
- **Cost.** Replaying recorded decisions costs what the simulation without fits costs: 0.15–0.32 s per replay,
  about 35,000–70,000 frames per second. That is fast enough to generate states on the fly in a training
  pipeline, and the decisions are small.
- **Why it is feasible.** v1 already changes the main arena only through explicit calls (`set_car_state`,
  `set_car_controls`, `set_ball_state`, `set_boost_pad_state`, `add_car`: 21 call sites in `conversion/mod.rs`
  before line 2898, plus `step_ticks` and `limit_reported_velocities`, which go through the same calls). The fits run in scratch arenas and hand back values.
- **What must be verified.** RocketSim must be deterministic for identical inputs on the same build
  **[verify]** (story 4.4).

## 3. Outputs

### 3.1 Principles

- **One file per replay.** Everything about a replay lives in one Parquet file:
  - the per-frame columns;
  - the variable-length records (events, touches, contacts, pickups, fitted inputs), as `list<struct>` columns
    on the frame they belong to;
  - the replay header (players, teams, slots, versions, options, diagnostics), as JSON in the file's key-value
    metadata.

  v1 split the records into seven files because a frame row had only fixed-width lists (RESULTS.md, "Parquet
  columns and record tables"). Variable-length list columns hold them directly, and empty lists should cost
  almost nothing **[verify]**.
- **Typed columns only.** No JSON copy of the frame. Anything a user needs is a column. Anything only an
  evaluator needs is not in the default file.
- **Profiles instead of everything.** A file states which profile it has:
  - `default`: state, controls, scoreboard, events, freshness, labels;
  - `+observations`: the raw replay fields with their update frames, as exact integers where the replay
    quantizes them (positions appear with two decimals, e.g. `512.04`) **[verify]**;
  - `+diagnostics`: the position residuals.
- **Same semantics as v1.**
  - Null means unknown, never zero.
  - Observed, inferred and future-derived values stay apart: the labels are a column group with a `label_`
    prefix and a header flag.
  - The 120 Hz timeline and the arena tick stay separate columns.
- **Encodings chosen per column:** BYTE_STREAM_SPLIT for floats, dictionary encoding for small enumerations,
  delta encoding for ticks and frame numbers, and zstd level 9. **[verify]** that the `parquet` 60 writer
  supports BYTE_STREAM_SPLIT for `f32`; the measurements above used PyArrow's.

### 3.2 The state file

`match.parquet` has one row per replay frame. The column groups (names are a proposal):

| Group | Columns | Notes |
| --- | --- | --- |
| time | `frame` u32, `replay_time` f32, `timeline_tick` u32, `arena_tick` u32 | |
| ball | `ball_position`, `ball_velocity`, `ball_angular_velocity` f32[3], `ball_quaternion` f32[4] | |
| cars (fixed list per slot) | `car_position`, `car_velocity`, `car_angular_velocity` f32[slots × 3], `car_quaternion` f32[slots × 4], `car_boost`, `car_present`, `car_demolished`, `car_on_ground` | |
| car internals | `car_jump_ticks`, `car_flip_time`, `car_air_time`, flags (`has_jumped`, `has_double_jumped`, `has_flipped`, …), `car_previous_controls` | what exact restoration needs (0.2–0.6 MB) |
| controls | `control_axes` f32[slots × 5], `control_buttons` bool/u8[slots × 3] | the action applied from this state, as in v1 |
| pads | `pad_active` (a bitmask per frame), `pad_cooldown` | pad positions once, in the metadata |
| scoreboard | `scores`, `scoreboard_period`, `scoreboard_clock_state`, `scoreboard_seconds_remaining`, `scoreboard_overtime_seconds` | |
| records | `events`, `touches`, `ball_contacts`, `boost_pickups`, `fitted_inputs` as `list<struct>` | v1's record tables, folded in |
| freshness | `ball_fresh`, `car_fresh`, `ball_packet_lag`, `car_packet_lag` (u8 ticks), update ages, `ping_raw` | v1's `packet_lags` table becomes a u8 column |
| labels | `label_episode`, `label_episode_seconds_remaining`, `label_next_scoring_team`, `label_seconds_until_next_goal` | future-derived; drop as a block |

**[decide] Rotation.** Quaternions save about a quarter of the state columns. But a matrix rebuilt from a
quaternion is not bit-identical to RocketSim's, so a v2 state file restores a RocketSim state only to float
rounding (about 1e-7). v1's restores it exactly (80,246 of 80,246 exact round trips: RESULTS.md, "Soccar state
restoration"). Exact restoration stays available through the plan file (3.3). Recommended: quaternions.

**[decide] Lossy profile.** A `compact` profile with integer-quantized state (0.01 UU, 0.01 UU/s, 1e-4 rad/s)
would be about 1.5–1.8× smaller again, and is cheap to add later. The converter's own error is far above the
quantum (p90 3.5 UU on the test split). It stays out of the first release unless you want it.

### 3.3 The plan file

`match.plan.parquet` holds what the fits decided, so that `replicar` can regenerate the states of
`match.replay` without fitting:
- the packet ticks (a u8 lag per object per frame);
- the timed control changes and presses per car;
- the air-control segments;
- the state edits the fits made (dodge starts and cancels, lag overrides).

Its header holds the replay's SHA-256, the replicar and RocketSim versions, the configuration, and a checksum of
a few regenerated states. A mismatched build is then refused instead of silently diverging.

Its size is unknown until it is written. v1's `fitted_inputs` and `packet_lags` tables, which hold part of it,
are 0.13–0.26 MB per replay, and the air segments come on top **[verify]** (story 5.5).

**The trade-off.**
- Against: a plan file is tied to one replicar and RocketSim version. Reading it needs the replay and the
  `replicar` package, and regenerating costs 0.15–0.3 s of CPU per replay.
- For: storage is a small fraction of the state file, and generation streams.

**[decide]** Which output should the 140,000 replays use first? Recommendation: plan files as the stored form
(together with the replays, which you keep anyway), and state files written on demand or for subsets.

### 3.4 A corpus

`replicar convert replays/ -o out/ --jobs N` writes one file per replay plus `out/index.parquet`. The index has
one row per replay: file name, SHA-256, status or error, frames, duration, map, game size, players and teams,
final score.
- A failed replay becomes a row with its error, not an aborted batch.
- `--skip-existing` resumes an interrupted run.
- Sharding many replays into a few files (with a `replay` column) can come later, if a training loader wants it.

## 4. The code, from the ground up

### 4.1 Pipeline

```
bytes --parse----> boxcars::Replay
      --observe--> Observations      typed replay fields per frame with update frames; actor lifetimes; events
      --time-----> PacketTimeline    each fresh packet's server tick (lag chains, ball runs, contact alignment)
      --run------> for each frame:   Executor applies packets at their ticks and steps RocketSim;
                                     at each decision point it asks a Planner (fitting or recorded)
      --annotate-> touches, ball contacts, pickups, freshness, scoreboard, labels
      --write----> state file | plan file | in-memory frames
```

Each arrow is a module with a function or a small struct, and each stage's output is a plain data type that can
be tested and printed. Compare v1:
- All of `run` and most of `annotate` are one function of about 1,830 lines (`conversion/mod.rs:1065-2897`),
  with about 40 mutable locals declared before its loop (`1101-1257`).
- Contact alignment is a recursive call of that function (`1075-1085`).
- The `time` stage is spread over `packet_lags.rs`, `contact_alignment.rs` and the loop.

### 4.2 Planner and executor

This is the central split.

The **executor** is small and does what RocketSim needs:
- it owns the arena and the slot table;
- it applies each packet's body at its tick;
- it holds or releases cars (spawn poses, dead shells, observed demolitions);
- it reconciles pads, steps ticks, and applies the controls it is given.

It contains no fitting and no lookahead.

The **planner** answers the executor's questions at the moments v1's fits run:
- when a ground control change takes effect;
- when a jump or dodge was pressed, with what direction and cancel;
- which air controls to fly between two packets;
- which tick the first packet after a dodge belongs to.

It has two implementations:
- `FittingPlanner`: v1's fits (`fits.rs`, `air.rs`, the jump/dodge logic of the loop), each in its own module.
  It reads the future only through a `Lookahead` view that knows which frames a masked evaluation withholds.
  Today the withheld check is spread over `frame_withheld` and `options.withheld_frames`.
- `RecordedPlanner`: answers from a plan file. Recording a plan just means writing down the fitting planner's
  answers.

Masked evaluation then becomes a planner configuration (`Holdout`), not a set of conversion options. A run without
fits is the executor with a planner that never changes anything; in v1 that is `input_fits = false`,
`air_bvp = false`, `infer_air_controls_from_lookahead = false`.

### 4.3 Types

- **Newtypes for identity and time:** `Slot`, `ActorId`, `Lifetime { actor, created_frame }`, `PlayerKey`,
  `TimelineTick`, `ArenaTick`, `FrameIndex`. In v1 a slot, an actor id and a frame index are all `usize` or
  `i32`, and a tick-rebase bug lived in exactly that gap (RESULTS.md, "Audit fix batch 5").
- **Per-car state in one struct:** `CarTrack` per lifetime and `SlotState` per slot. They replace v1's maps keyed
  by `(actor, created_frame)` tuples (`conversion/mod.rs:1105-1257`): `spawn_started`, `spawn_held`,
  `spawn_demolished`, `gated_jump_active`, `last_dodge_raw`, `last_double_raw`, `last_counters`,
  `ground_counters`, `car_shifts`, `handled_dodges`, `flip_cache`, `flip_last`, `lag_overrides` (a `RefCell`
  today), `dead_shells`, `demo_hold_until`, `last_contact_tick` and `slot_bodies`.
- **Enums for v1's strings:** `LagSource`, `FittedInput { Jump, Dodge { pitch, yaw, cancel }, Air { span } }`,
  `HoldSource`, `GameState`. The game state is compared as `== "Active"` today (e.g. `ball_evidence.rs:81`).
- **Network property names as constants**, as VirxEC does (`src/attributes.rs`). `observations.rs` has 44 inline
  literals.
- **One `thiserror` error type.** No `Box<dyn Error>` or string errors in the library (v1 has 7 such sites).

### 4.4 Crates

```
crates/replicar          library: observe, time, execute, plan, annotate, write, read, restore
crates/replicar-cli      the `replicar` binary: convert (file or folder), regenerate, inspect, verify
crates/replicar-python   pyo3 module + python/replicar package (maturin)
crates/replicar-eval     publish = false: evaluate_corpus, error_budget, rlbot_*, dump_reconstruction, the
                         consistency checks, the v1 parity checks; the sealed test-split guard lives here
```

The library's modules mirror the pipeline: `observe/`, `time/`, `execute/`, `plan/` (with `fits/` and `air/`),
`annotate/`, `format/` (the Arrow schema) and `restore.rs`.
- No function over about 150 lines.
- Tests sit next to the code they test. 41 of v1's 91 tests are in `conversion/mod.rs`, testing code in other
  files.
- Taken from VirxEC:
  - a `Converter` builder;
  - the `rocketsim` re-export instead of a separate `glam` pin;
  - crate docs that explain the timing model and the event streams first;
  - `examples/`, a criterion benchmark and a public-API test.

### 4.5 Using it

```rust
let meshes = replicar::Meshes::load("collision_meshes")?;      // checks the soccar meshes; rocketsim::init once
let converter = replicar::Converter::new(&meshes, Config::default());
let replay = converter.convert(&bytes)?;                       // frames in memory
replay.write("match.parquet", Profile::Default)?;              // one file, written atomically
converter.record(&bytes)?.write("match.plan.parquet")?;        // decisions only
for frame in converter.regenerate(&bytes, &plan)? { ... }      // streams, no fitting
```

```
replicar convert match.replay -o match.parquet [--profile default|observations|diagnostics] [--octane-hitbox]
replicar convert replays/ -o out/ --jobs 16 [--skip-existing] [--plan]   # folder: one file per replay + index.parquet
replicar regenerate match.replay match.plan.parquet -o match.parquet
replicar inspect match.replay | match.parquet                            # header, players, slots, diagnostics
replicar verify match.parquet                                            # restoration and schema checks
  --meshes DIR, else $REPLICAR_MESHES, else ./collision_meshes
```

```python
import replicar
replicar.set_meshes("collision_meshes")
replay = replicar.convert("match.replay")            # GIL released; .header, .arrays() (NumPy), .records() (pyarrow)
replicar.read("match.parquet")                       # same object from a file; pure pyarrow, no Rust needed
for batch in replicar.regenerate("match.replay", "match.plan.parquet", batch_frames=4096):
    ...                                              # NumPy batches for a training loop
replicar.convert_many(paths, "out/", jobs=16, plan=True)
```

## 5. Proving it is the same reconstruction

v1 stays the oracle. `replicar-eval` depends on v1 at the tag `v1-final` through a renamed git dependency (e.g.
`replicar_v1 = { package = "replicar", git = ..., tag = "v1-final" }`). Both depend on the same RocketSim
revision, so the comparison runs in one process. **[verify]** that cargo unifies the RocketSim dependency and that
`rocketsim::init` tolerates being called a second time.

1. **States, bit for bit.** For each of the 120 train and validation replays, v2 must equal v1 exactly on:
   - every frame's ball and car state (physics, timers, flags, controls), the pad states and the arena tick;
   - the records (events, touches, contacts, pickups, fitted inputs, packet lags);
   - the scoreboard and the labels.
2. **A ladder of configurations**, so that a difference points at one part:
   - the executor without fits, against v1 with `input_fits`, `air_bvp`, `infer_air_controls_from_lookahead` and
     `align_contacts` off;
   - then each fit switched on;
   - then the defaults;
   - then the evaluators' held-out variants (`flip_cancel_holdout`, `fit_on_next_packet` off, withheld frames).

   v1 already has an option for every rung (`conversion/mod.rs:57-114`).
3. **Evaluator reports.** `evaluate_corpus`, ported to v2, must print the same numbers as v1's reference run
   (default and `--aligned-targets`, train and validation). If the states are identical this follows
   automatically, but it is still run, because the evaluator itself is ported.
4. **Plan replay.** States regenerated from a plan file must equal the states of the conversion that recorded it,
   on all 120 replays, in a second process, and on a second machine if one is available. **[verify]**
   cross-machine determinism. The plan file's checksum makes a mismatch loud.
5. **Size and speed** on the fixed replay list in RESULTS.md: file size per profile; conversion, recording and
   regeneration time; mean and maximum against v1.

An intentional change of output (a v1 behaviour found to be wrong during the rewrite) goes in a separate commit
with the full protocol: a paired `run_reference.sh` run against v1; counts and p50/p90/p99 per game size and per
replay; the decision made on validation; an entry in RESULTS.md. The test split stays sealed.

## 6. Stories

Each story is one reviewable commit that passes `cargo test --all-targets` and the parity check of its rung.
C is the complexity. Work happens on a `v2` branch, and the record files are updated at each milestone.

| # | Story | Acceptance | C |
| --- | --- | --- | --- |
| **0** | **Baseline** | | |
| 0.1 | RocketSim to the latest compatible revision (crates.io 0.2.7 or `v3-rust` `dc5d60e`; **[verify]** which is newer and whether they are the same code); ROCKETSIM_NOTES re-run | measured like the 2026-10-04 update | M |
| 0.2 | Tag `v1-final`; v1 reference reports on train and validation; speed baseline | recorded in RESULTS.md | L |
| 0.3 | Determinism: run v1 twice, and alongside `cargo test` (7 headers differed once in such a run: RESULTS.md, "Release clean-up") | equal, or the cause found | M |
| **1** | **Skeleton** | | |
| 1.1 | Workspace, four crates, CI commands, `thiserror` error, newtypes | builds; unit tests | M |
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
| 4.4 | Determinism of the executor across processes | equal states, 120 replays | L |
| **5** | **Plan** | | |
| 5.1 | Planner interface and `Lookahead` with withheld frames; air lookahead and persistence | rung "air lookahead" equal; the masked leak test passes | M |
| 5.2 | Air boundary-value solve (`air.rs`) | rung "air_bvp" equal | H |
| 5.3 | Input fits: ground timing, jump, dodge start and first-packet tick, flip cancel (`fits.rs`) | rung "input_fits" equal, then the defaults | H |
| 5.4 | Held-out variants | evaluator rungs equal | M |
| 5.5 | `RecordedPlanner` and the plan file | regenerated equals recorded on 120 replays; plan size measured | M |
| **6** | **Annotate** | | |
| 6.1 | Touches, ball contacts, boost pickups | records equal | M |
| 6.2 | Scoreboard, freshness, labels | equal | M |
| **7** | **Formats** | | |
| 7.1 | State file schema and writer, profiles, atomic write | values equal to v1 Parquet column by column; size measured per profile | M |
| 7.2 | Reader (Rust and pure Python) and restoration | restores every frame within the rotation tolerance; exactly from a plan | M |
| **8** | **Front ends** | | |
| 8.1 | CLI: convert (file, or folder with `index.parquet`), regenerate, inspect, verify | train converted as a folder equals file-by-file | M |
| 8.2 | Python: convert, read, regenerate (batches), convert_many; type stubs; wheels | Python tests; arrays equal to the Rust reader's | M |
| **9** | **Evaluation and release** | | |
| 9.1 | Port `evaluate_corpus` and the tools the protocol uses to `replicar-eval` | reports equal to v1's reference | H |
| 9.2 | Docs (concepts, output format, evaluation, contributing), examples, benchmark | `cargo doc` clean | M |
| 9.3 | Final check (section 5), CHANGELOG 2.0.0, crates.io and PyPI | all of section 5 | M |

Speed gains taken along the way, where they cost nothing in output:
- the folder mode converts replays in parallel;
- the plan file removes the fits from every later use;
- contact alignment's first pass may not need to rebuild what the second pass rebuilds **[verify]**
  (`conversion/mod.rs:1075-1085`, `contact_alignment.rs`).

The deeper gains wait for parity; the boundary-value solve is the largest single cost (RESULTS.md, "Code review
before the freeze, and what a conversion costs").

## 7. Risks

- **A rewrite drifts.** The parity ladder is the guard: no story is done while its rung differs from v1, and each
  rung isolates one part.
- **Floating-point order.** Bit-identical states require the same operations in the same order. Ported code keeps
  its arithmetic as written, and differences are found rung by rung, not at the end.
- **Determinism of plan replay.** If RocketSim is not deterministic across processes or machines, the plan file
  degrades to "same build, same machine". Story 4.4 finds out early.
- **Cost.** This is a larger project than the first draft's refactor, but the milestones are useful on their own:
  the plan file exists after story 5.5, and the state file after milestone 7.

## 8. Questions for the user

1. **Plan file or state file** as the stored form for the 140,000 replays (3.3)? Recommendation: plan files, with
   state files on demand.
2. **Quaternions** in the state file, with exact restoration only from plan files (3.2)?
3. **Observations** (raw replay fields with freshness) in an opt-in profile rather than the default file?
4. **A lossy compact profile**: wanted now, later, or never?
5. **Python:** the oldest version to support (the measurements used 3.11)?
