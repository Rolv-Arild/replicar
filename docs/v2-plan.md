# replicar v2: design and migration plan

Status: proposal, 2026-10-06, branch `v2-plan` from `master` (`5a7da58`). Nothing here is implemented.
Every statement about the current code cites a file and line at `5a7da58`. Every statement about VirxEC's
converter cites `external/replay-to-rocketsim` at `bc61b66`, the upstream tip of 2026-09-27, cloned there
for reference and git-ignored. Items marked **[decide]** need the user. Items marked **[verify]** are
assumptions to check before the story that depends on them starts.

## 1. What v2 is for

v2 should make the same reconstruction (the same accuracy and speed) much easier to use and to read:

- **Python/ML users**: `pip install` a package, then `replicar.convert("match.replay")` returns arrays and
  tables. No `sys.path` edits and no subprocess.
- **Rust users**: a small, documented API (`Converter`, `Config`, a few output types). It accepts bytes,
  never panics on a bad replay, and returns typed errors.
- **CLI users**: one `replicar` binary with subcommands, batch conversion of a folder, and sensible defaults.
- **Contributors**: a pipeline of named stages, each in a file short enough to read in one sitting, with
  the per-car state in one struct instead of forty maps. Evaluators and diagnostics live in their own
  crate.

### Done means

1. **Accuracy.** On all 120 train and validation replays, with the same RocketSim revision and toolchain
   on both sides, v2's default JSONL **frame records are byte-identical** to v1's (SHA-256 per replay).
   The header may differ only in fields this plan names (section 6). Where we choose to change the output
   (section 6.3), `evaluate_corpus` (default and `--aligned-targets`, train and validation) shows no
   material regression against the v1 reference, judged on validation. This follows the user's rule of
   2026-10-06: aim for byte-identical, but do not bend over backwards for it.
2. **Speed.** Mean and maximum wall time per replay no worse than v1 within noise. The v1 figures to
   beat: mean 9 s and maximum 24 s over the 120 replays with four converting in parallel; one 3v3
   replay 11.4 s, one 1v1 3.3 s (RESULTS.md:1684). Measured with `python/benchmark_conversion.py` on a
   fixed replay list (proposed threshold: mean +3%, no replay +10%; **[decide]**).
3. **Usability.** The four entry points in section 4 work as written in the README, from a clean checkout
   and from a pip install, with tests.
4. **Readability.** No function over about 150 lines in the library (today one is about 1,830). Per-car
   state lives in a struct. No string-typed enums in the public API.

### Out of scope

- No algorithmic change during the rewrite. Physics, timing inference, fits and their constants move
  verbatim; the golden gate (section 6) enforces it. Improvements come after parity, as ordinary measured
  changes (section 9).
- No new game modes. VirxEC's mode and mutator handling (`src/arena_config.rs`) is a later, separate
  feature (PLAN.md "Next action").
- The test split stays sealed. v2 is verified on train and validation only.

## 2. What VirxEC's converter does better, and what we leave alone

VirxEC's crate is about 7.2k lines; ours is about 16.5k lines of library and 29k with the tools. The size
difference is mostly algorithm, not structure: VirxEC places packets by rounding frame time
(`src/timing.rs:34`) and has no packet-lag inference, input-timing fits, contact alignment or
boundary-value air solve. The accuracy numbers in RESULTS.md come from those parts, so we keep them. Its
structure is still worth taking.

| VirxEC practice | Where | Adopt? |
| --- | --- | --- |
| One `Converter` value with builder methods; `convert_bytes(&[u8])` and `convert_replay(&Replay)` | `src/converter.rs:34-90` | **Yes**, with our `Config` (section 4.1) |
| `thiserror` error enum with structured variants (`MissingActor(ActorId)`, `NonFiniteRigidBody { actor_id, field }`) | `src/error.rs` | **Yes**. Ours mixes a hand-written enum (`conversion/mod.rs:30-54`) with `Box<dyn Error>` and string errors (7 places in `src/`, e.g. `parquet_export.rs:526-533`) |
| One module per concern: `actor/`, `phys.rs` (per-field freshness), `controls.rs`, `body.rs`, `timing.rs`, `metadata/` | `src/lib.rs:30-41` | **Yes**: the module tree in section 3 |
| Every network property name as a `const` in `attributes.rs` | `src/attributes.rs` (88 lines) | **Yes**: `observations.rs` has 44 inline `"TAGame.…"`/`"Engine.…"` literals |
| Re-export `rocketsim` so users get matching glam types | `src/lib.rs:55` | **Yes**: we depend on `glam` separately (`Cargo.toml:18`, 47 `glam::` uses), and RocketSim re-exports glam at our pin (`rocketsim/src/lib.rs:56`, `pub use glam_inc::*`) |
| Crate docs that explain the timing model and the two event streams up front | `src/lib.rs:1-28` | **Yes**: our `lib.rs:1-5` still says "the conversion pipeline is being built incrementally" |
| `examples/` for runnable usage, `benches/` with criterion, `tests/api.rs` for the public surface | `examples/`, `benches/parse_replays.rs`, `tests/api.rs` | **Yes**. We have 27 binaries in `src/bin` and no example or benchmark |
| `clippy::pedantic` as warnings, rustfmt config | `Cargo.toml:21-27`, `rustfmt.toml` | **Yes**, phased in (pedantic over 16k lines is a story of its own) |
| `rocketsim` from crates.io (`0.2.0`) | `Cargo.toml:17` | **[decide]**. crates.io has `rocketsim` 0.2.7 from ZealanL's repository (`cargo info rocketsim@0.2.7`). Depending on it would let replicar itself go to crates.io (our git pin forbids it: `Cargo.toml:11-12`). It must be the latest compatible revision (user rule, 2026-10-06) and needs its own measured update step |
| `predicted_states` beside `states`: the simulation just before each correction, kept for audits | `src/converter.rs:246` | **Partly**. Our `position_residuals` carry the same information more compactly; v2 keeps them and offers the pre-correction state through the eval feature |
| Parallel per-frame vectors in the output (`states`, `frames`, `frame_metadata`, `cars`, …) | `src/converter.rs:239-276` | **No**. One `Frame` struct per frame reads better in Rust; the Python side gets columns anyway |
| Timestamp-rounded ticks, kickoff `prediction_valid` heuristic, rlgym-tools-style edge shifts | `src/converter.rs:134-176`, `AGENTS.md` | **No**: less accurate than what we measured (RESULTS.md) |

Its `ActorTracker` also holds about 20 per-actor maps (`src/actor.rs:25-50`). What makes VirxEC's code
nicer is clear boundaries between modules, not an absence of state, and that is what v2 copies.

## 3. Where v1 hurts (evidence)

1. **One function holds the converter.** `convert_observations_with` runs from `conversion/mod.rs:1065` to
   `2897`, about 1,830 lines. Before its frame loop it declares about 40 mutable locals, most of them maps
   keyed by `(actor_id, created_frame)` tuples (`1101-1257`). Inside the loop, the per-car body of the
   phase loop is about 780 lines (`1566-2350`), stepping goes through a local `advance_to!` macro
   (`1471-1521`), and one closure reads a `RefCell` (`lag_overrides`, `1250`). A reader has to hold all
   of this in their head to change any one rule.
2. **Contact alignment is hidden recursion.** With `align_contacts`, the function computes lags, clones the
   options with `external_packet_lags` set, and calls itself (`1075-1085`). This two-pass structure is the
   most important timing step, and the code doesn't show it as one.
3. **`ConvertOptions` mixes three kinds of switch** (`57-114`): user choices (`use_loadout_hitboxes`,
   `seed`), evaluator hold-outs that only make sense in a masked run (`withheld_frames`,
   `fit_on_next_packet`, `flip_cancel_holdout`, `infer_dodge_first_packet_tick`) and experiment hooks
   (`external_packet_lags`, documented as "Experiments only"). It also lets you build invalid
   combinations: `zero_packet_lag`, `infer_packet_lag` and `external_packet_lags` are three ways to set
   one thing, and `align_contacts` silently does nothing unless lags are inferred. `impl ConvertOptions {}`
   is empty (`116`). The mesh path is in the options, and `rocketsim::init` runs inside every conversion
   (`1091`).
4. **The output types are wide and stringly typed.** `ConvertedFrame` has 20 public fields (`249-300`) that
   mix state, simulated events, replay evidence and provenance. Enums are `&'static str`:
   `AppliedPacketLag::source` (`"chain"`, `"frame_median"`, `"default"`), `FittedInput::kind`, and
   `DeadShellHold::source`. The game state is compared as a string, `== "Active"` (`ball_evidence.rs:81`,
   among others). Identity is raw `i32`/`usize` and `String` player keys, so a slot, an actor id and a
   frame index have the same type. A past tick bug lived in exactly that gap (RESULTS.md "Audit fix batch
   5: tick rebase").
5. **Export runs its own conversion.** `write_parquet_with_tables` takes replay bytes and converts
   internally (`parquet_export.rs:522-533`), so an existing `ConversionOutput` can't be written to
   Parquet. JSONL goes the other way: it materializes the whole output first (`convert_replay.rs:97-103`).
   The atomic publish (write to temporary names, back up, rename, roll back) is about 100 lines inside the
   CLI binary (`convert_replay.rs:86-190`), so Python or library users can't reuse it.
6. **Python access is two script files.** `python/replicar.py` (JSONL) and `python/replay_columnar.py`
   (Parquet) are loaded with `sys.path.insert(0, "python")` (README "Reading the output in Python"). There
   is no package, and converting from Python means launching the CLI.
7. **The public API carries tool-only items.** `rebase_tick`, `step_arena_tick`, `hitbox_config`,
   `quaternion` and `rotation_error_degrees` are `pub` for the binaries and tools. `activation_torque`,
   `controls_from_observation` and `check_soccar_meshes` are `pub` with no user outside `conversion/` at
   all. `pub use air::*` and `pub use packet_lags::*` (`24-26`) export the solvers wholesale.
8. **Tests live far from the code they test.** 41 of the 91 tests are in `conversion/mod.rs` (`2898-6105`,
   about 3,200 lines), testing fits, air solves and lag inference that live in other files.
9. **Twenty-seven binaries share one crate.** `cargo build` compiles the user's converter together with 26
   evaluators, calibrations and diagnostics, and all of them shape the library's public surface (point 7).
   The README lists only some of them.
10. **One possible nondeterminism.** During the release clean-up, 7 headers differed in a run made while
    the test suite ran at the same time, then matched when re-run alone (RESULTS.md, "Release clean-up").
    This was never reproduced. A golden gate needs determinism, so story 0.2 looks into it first. One
    candidate to check is iteration over `std::collections::HashMap` (random order) wherever the order
    reaches output **[verify]**.

## 4. The v2 surface

### 4.1 Rust

This is a design sketch of the proposed API; it has not been compiled. The RocketSim calls it relies on
exist at our pin: `rocketsim::init(path, silent)` (`conversion/mod.rs:1091`), `Arena::new_with_config`,
and `ArenaState`.

```rust
use replicar::{Converter, Config, Meshes};

let meshes = Meshes::load("collision_meshes")?;     // checks the soccar *.cmf files, calls rocketsim::init once
let converter = Converter::new(&meshes, Config::default());
let replay = converter.convert(&std::fs::read("match.replay")?)?;   // -> Reconstruction

for frame in &replay.frames {
    let state = &frame.state;                         // rocketsim::ArenaState at the frame time
    let clock = &frame.scoreboard;
    for goal in frame.events.replay.goals() { /* observed */ }
    for touch in &frame.events.ball_contacts { /* from ball packets */ }
}
replay.write_parquet("match.parquet")?;               // atomic: complete output or nothing, with record tables

// Streaming: snapshot memory bounded to one frame (the observations stay resident, as in v1)
converter.convert_into(&bytes, replicar::export::ParquetSink::create("match.parquet")?)?;
```

**`Config`** has the user-facing knobs only. Each field of v1's `ConvertOptions` maps to v2 like this:

| v1 field (`conversion/mod.rs:57-114`) | v2 |
| --- | --- |
| `collision_meshes` | `Meshes` (a loaded resource, not an option) |
| `seed` | `Config::seed` |
| `use_loadout_hitboxes` | `Config::hitboxes: Hitboxes::{Loadout, Octane}` |
| `infer_packet_lag`, `zero_packet_lag`, `align_contacts` | `Config::packet_timing: PacketTiming::{Inferred { align_contacts: bool }, LagFree, FrameTime}`. Invalid combinations can no longer be written |
| `external_packet_lags` | `PacketTiming::External(Arc<PacketLags>)`, only with the `eval` feature |
| `input_fits` | `Config::input_fits: bool` |
| `infer_air_controls_from_lookahead`, `air_bvp` | `Config::air: AirConfig { lookahead: bool, boundary_value: bool }`. These stay two switches because they are layered, not alternatives: lookahead first, then the past-persistence fallback that has no switch, then the boundary-value schedule at fresh packets (`1955-2226`) |
| `block_sim_pad_pickups` | `Config::simulated_pad_pickups: bool` (inverted, default `false`) |
| `disable_simulated_demolitions` | `Config::simulated_demolitions: bool` (inverted, default `false`) |
| `withheld_frames`, `fit_on_next_packet`, `flip_cancel_holdout`, `infer_dodge_first_packet_tick` | `eval::Holdout` (feature `eval`). Its default reproduces today's offline defaults; evaluators build the held-out variants |

**Output types.** `Frame { time, state, scoreboard, events, inferred, freshness, observed }`:

- `time: FrameTime { index, replay_time, timeline: TimelineTick, arena: ArenaTick }`. The two tick kinds
  are distinct newtypes, so mixing them doesn't compile (AGENTS.md: keep the 120 Hz timeline separate from
  packet cadence).
- `events: FrameEvents { replay, simulated, touches, ball_contacts, boost_pickups }`. These stay separate
  streams, as v1 and VirxEC both insist.
- `inferred: Inferred { packet_lags, fitted_inputs, holds: Holds { dead_shells, spawn_pose, sleeping_velocity, demolitions } }`.
- Enums replace the strings: `LagSource::{Chain, FrameMedian, Default}`,
  `FittedInput::{Jump, Dodge { pitch, yaw, cancel }, Air { span_ticks }}` and
  `HoldSource::{Observed, Inferred}`. Serde attributes (`rename`, `tag`, `skip_serializing_if`) reproduce
  today's JSON exactly. This is how the byte-identical goal survives the type change.
- Newtypes: `Slot`, `ActorId`, `Lifetime { actor, created_frame }`, `PlayerKey`.

**Errors**: one `replicar::Error` (`thiserror`) with `Parse`, `NoNetworkFrames`, `UnsupportedMode`,
`Meshes`, `InvalidTime { frame, time }`, `NoCarSlots`, `Io` and `Export(parquet/arrow)` variants. No
`Box<dyn Error>` in the library.

### 4.2 CLI

```
replicar convert match.replay -o match.parquet            # or .jsonl; record tables beside it by default
replicar convert replays/ -o out/ --jobs 8 --skip-existing # batch; one failure doesn't stop the rest
replicar inspect match.replay                              # header, players, slots, diagnostics (text or --json)
replicar verify match.parquet                              # restoration check (today: verify_state_restoration)
  common: --meshes DIR (else $REPLICAR_MESHES, else ./collision_meshes), --octane-hitbox, --no-tables
```

Argument parsing with `clap` (derive) **[decide: new dependency]**. Batch mode converts replays in parallel
and writes a summary of failures at the end, since the conversion is single-threaded per replay.

**[decide] The sealed-split guard.** Today `convert_replay` refuses any input path with a component named
`test` (`lib.rs:19-49`, `convert_replay.rs:85`). That is right for this project's protocol and wrong for a
user whose folder happens to be called `test`. Proposal: the guard moves to `replicar-eval` and the
repository's scripts, where the protocol runs, and the published `replicar` CLI drops it. This changes the
test-split protection, so it needs the user's agreement.

### 4.3 Python

```python
import replicar                                   # pip install replicar (maturin wheel)
replicar.set_meshes("collision_meshes")           # or REPLICAR_MESHES
replay = replicar.convert("match.replay")         # bytes or path; releases the GIL while converting
replay.header                                     # dict, as read_columnar_header returns today
arrays = replay.arrays()                          # same keys and dtypes as load_columnar_numpy
tables = replay.tables()                          # dict of pyarrow.Table, as read_record_tables
replay.write("match.parquet")                     # same atomic export as the CLI
old = replicar.read("match.parquet")              # existing exports, pure Python (today's loaders)
replicar.convert_many(paths, out_dir, jobs=8, skip_existing=True)
```

The in-memory path reuses what we have instead of adding a second schema. Rust writes the Parquet export
into a byte buffer (the writer needs `Write + Send`, not `File`), and Python reads it with pyarrow and the
existing column logic from `replay_columnar.py`. One schema, one implementation, already tested against
JSONL (`verify_direct_parquet.py`). The pure-Python loaders merge into the package as `replicar.read`.
Type stubs (`.pyi`) ship with it.

Collision meshes can't ship in the wheel: they are extracted from the game, and the README says to supply
them. **[decide]** whether to document a download or dump route (for example the user's
`RLArenaCollisionDumper`) or leave it to the user as today.

## 5. The v2 layout

```
Cargo.toml                       workspace
crates/replicar/                 the library (publishable if rocketsim comes from crates.io)
  src/lib.rs                     crate docs: pipeline, timing model, observed/inferred/future-derived vocabulary
  src/converter.rs               Converter, Config, Meshes; runs the stages in order (target under 300 lines)
  src/error.rs                   Error
  src/ids.rs, src/time.rs        Slot, ActorId, Lifetime, PlayerKey; TimelineTick, ArenaTick, FrameTime
  src/observe/                   stage 1, from observations.rs (1,474 lines)
    attributes.rs                every network property name as a const
    actors.rs                    actor graph, lifetimes, player links, ID reuse
    header.rs  events.rs  mod.rs Observations, ObservedFrame, Value<T>, Source
  src/analyze/                   stage 2: whole-replay, offline, before the main simulation
    timing/                      packet_lags.rs (802), lag-free detection, ball runs; contact_alignment.rs (398)
    ball_evidence.rs  scoreboard.rs
    pads.rs                      pad name -> pad index votes (conversion/mod.rs:1125-1180 today)
  src/reconstruct/               stage 3: the simulation
    mod.rs                       Reconstructor::run: the per-frame loop as named steps (section 5.1)
    plan.rs                      FramePlan: object lags, phases, control switches (1384-1467 today)
    stepping.rs                  step_ticks and advance_to (896-985, 1471-1521 today)
    slots.rs                     SlotTable: player -> slot, hitbox, loadout changes
    car.rs                       CarTrack: everything kept per car lifetime (section 5.2)
    lifecycle.rs                 spawn-pose holds, dead shells, demolition holds
    actions.rs                   jump/dodge counters, jump gate, dodge refresh
    pads.rs                      pad cooldowns, blocked simulated pickups
    lookahead.rs                 Lookahead: the only door to future packets; refuses withheld frames by construction
    fits/                        from fits.rs (1,392): ground timing, jump, dodge start, flip cancel; ScratchArenas pool
    air/                         from air.rs (942): inverse model, span solve, boundary-value solve
  src/annotate/                  stage 4: from the exported poses
    touches.rs  contacts.rs  pickups.rs  freshness.rs  labels.rs
  src/model/                     output types and their serde shape (frozen against the golden files)
  src/export/                    stage 5
    sink.rs                      FrameSink trait; VecSink
    jsonl.rs                     from serialization.rs
    parquet/                     from parquet_export.rs (1,101) and parquet_tables.rs (1,205)
    atomic.rs                    publish-on-complete, from convert_replay.rs:86-190
  src/restore.rs                 from restoration.rs
  examples/  benches/  tests/
crates/replicar-cli/             the `replicar` binary
crates/replicar-python/          pyo3 bindings + python/replicar/ package (maturin)
crates/replicar-eval/            publish = false: evaluate_corpus, error_budget, rlbot_*, dump_reconstruction,
                                 check_scoreboard, count_demolitions, consistency_counts, diagnostics; the sealed guard
docs/                            getting-started, concepts, output-format, evaluation, contributing
```

The library's `eval` feature exposes what the evaluators need (`Holdout`, `PacketTiming::External`,
pre-correction states, the solvers) under `replicar::eval`. It stays out of the default public surface.

### 5.1 The per-frame loop, as it should read

```rust
for frame in observations.frames() {
    let clock = self.clock.advance(frame)?;              // timeline tick, gap, simulated? (1259-1300 today)
    let plan = FramePlan::new(frame, &clock, &self.timing, &self.cars);   // lags, phases, control switches
    for phase in plan.phases() {
        self.step_to(phase.tick, &plan.switches);         // advance_to!
        self.apply_ball_packet(frame, phase);
        for car in phase.cars() { self.cars.track(car).apply_packet(&mut self.arena, car, &ctx)?; }
    }
    self.step_to(plan.end, &plan.switches);
    self.finish_interval(frame, &clock);                  // demolitions, dodge refresh, pads (2360-2570 today)
    let frame_out = self.annotate(frame, &clock);         // touches, contacts, pickups, freshness (2576-2832)
    sink.frame(&frame_out)?;
}
```

`CarTrack::apply_packet` is the 780-line body of today's per-car loop, split into the steps its comments
already name: slot resolution, body and spawn pose, spawn hold, sleeping packet, dead shell, boost, dodge
and double-jump counters, the jump gate, air controls (lookahead, then persistence), the boundary-value
schedule, and the input fits.

### 5.2 Per-car state in one struct

These v1 maps move into `CarTrack` (keyed by `Lifetime`) or `SlotState` (keyed by `Slot`). Line numbers
are their declarations in `conversion/mod.rs`:

- Per lifetime: `spawn_started`, `spawn_demolished`, `gated_jump_active`, `last_dodge_raw`,
  `last_double_raw`, `last_counters`, `ground_counters`, `car_shifts`, `handled_dodges`, `flip_cache` and
  `flip_last` (keyed by lifetime plus frame), and `lag_overrides`, whose `RefCell` goes away (1105-1257).
- Per slot: `spawn_held`, `dead_shells`, `demo_hold_until`, `last_contact_tick`, `slot_bodies`.
- Shared, replay-wide: `pad_actor_to_index`, `pad_cooldowns`, `pad_name_to_index`, `last_pad_counter`
  (to `reconstruct/pads.rs` and `analyze/pads.rs`); `recent_poses` and `recent_touch_ticks` (to
  `annotate`); the scratch arenas `ground_scratch` and `flip_scratch` (to `fits::ScratchArenas`).

A slot's or a car's whole state is then visible in one `Debug` print, which also helps the frame-by-frame
window inspections AGENTS.md asks for.

## 6. How parity is checked

### 6.1 The golden gate (every story)

- **Story 0.1** writes `replicar-eval golden`. For each of the 120 train and validation replays, it records
  the SHA-256 of the default JSONL frame lines, the header with the `options` block removed, and the
  Parquet column contents. They go in `target/golden/<commit>/manifest.json` (hashes only; no replay
  content; ignored by git). It extends the existing `hash_observations` binary, which already hashes
  extracted observations for parser changes (`src/bin/hash_observations.rs:1-4`).
- Before every story is committed, the manifest of the v1 tag `v1-final` (made in story 0.1) must equal
  the manifest of the story's tree. Both are built with the same RocketSim revision and toolchain. Any
  difference blocks the story until it is explained.
- Floating-point order matters: moving code must keep expression order. The gate catches it when it
  doesn't.

### 6.2 Headers

v2 serializes a **v1-compatible options record**: the v1 field names and values, derived from `Config`
and `Holdout`. This keeps the header's `options` block and the record tables' `options_sha256` identical,
so even headers can stay byte-identical. When we later want to show the v2 names, that is a schema-v2
change made on purpose (section 9), not a by-product of the rewrite.

### 6.3 Intentional output changes

Stories in this plan should not need any. If one turns out to be preferable (for example a behaviour that
only the old structure produced and that the evidence says is wrong), it goes in its own commit. That
commit gets the full protocol: `scripts/run_reference.sh` (default and aligned, train and validation)
against the v1 reference at the same RocketSim revision; counts and p50/p90/p99 pooled, per game size and
per replay; windows inspected for material changes; the decision made on validation; and an entry in
RESULTS.md.

### 6.4 Speed

`python/benchmark_conversion.py` on a fixed list (3 replays per game size from train, one warm-up, three
repetitions), v1 and v2 built from `git archive`, plus the corpus wall time from `run_reference.sh`.
Checked at the end of milestones 3, 5 and 8.

### 6.5 RocketSim

The rewrite keeps one RocketSim revision from start to finish, so every comparison runs on the same
version (user rule, 2026-10-06). Updating RocketSim to the latest compatible revision (`dc5d60e` on
`v3-rust` at 2026-10-06, or crates.io 0.2.7 **[verify]** whether these are the same code) is a separate
step before story 0.1 or after story 8.3. It gets its own measurement and a re-run of the issues in
ROCKETSIM_NOTES.md. Recommended: **before**. The golden reference is then made on the newest RocketSim,
and v2 never has to straddle an update.

## 7. Stories

Each story fits one reviewable commit (at most about 300 changed lines of logic, apart from moved code; at
most about 3 files of new logic) and passes `cargo test --all-targets` and the golden gate. Complexity:
L(ow), M(edium), H(igh). Work happens on a `v2` branch. The record files (PLAN, RESULTS, README) are
updated at each milestone.

**Milestone 0: baseline**

| # | Story | Acceptance | Risk | C |
| --- | --- | --- | --- | --- |
| 0.1 | `golden` subcommand and `v1-final` tag; manifest for 120 replays | Two runs of the same build give equal manifests | Long runtime (about 9 min with four parallel) | M |
| 0.2 | Determinism check: run the golden build twice in parallel with `cargo test`; look into the unreproduced header differences (section 3, point 10) | 3 repeated runs equal; cause found or ruled out | May not reproduce again | M |
| 0.3 | Speed baseline on the fixed list | Recorded in RESULTS.md | Machine noise | L |

**Milestone 1: workspace without behaviour change**

| # | Story | Acceptance | Risk | C |
| --- | --- | --- | --- | --- |
| 1.1 | Cargo workspace; library moves to `crates/replicar` unchanged | Golden equal; all tests pass | Paths in tests (`CARGO_MANIFEST_DIR` + `replays/`) | M |
| 1.2 | Binaries move to `crates/replicar-eval` (multi-bin first, subcommands later); `run_reference.sh` and README updated | Every binary builds; golden equal | Tools using `pub` internals: gather them behind the `eval` feature | M |
| 1.3 | `glam` usage through `rocketsim`'s re-export; drop the direct dependency if every type used is re-exported **[verify]** | Builds; golden equal | Version skew if the re-export differs | L |

**Milestone 2: types**

| # | Story | Acceptance | Risk | C |
| --- | --- | --- | --- | --- |
| 2.1 | `ids.rs`, `time.rs` (newtypes, `TimelineTick`/`ArenaTick`) | Unit tests; golden equal | Wide but mechanical change | M |
| 2.2 | Enums for `LagSource`, `FittedInput`, `HoldSource`, `GameState`, with serde shapes matching v1 | Serde round-trip tests against v1 JSON snippets; golden equal | A serde detail (field order, skip rules) differs | M |
| 2.3 | `thiserror` `Error`; export functions return it | No `Box<dyn Error>` in the library | Messages change (CLI text only) | L |

**Milestone 3: configuration and stages before the simulation**

| # | Story | Acceptance | Risk | C |
| --- | --- | --- | --- | --- |
| 3.1 | `Config`, `PacketTiming`, `AirConfig`, `eval::Holdout`; v1 options record for the header | Mapping table of section 4.1 tested both ways; golden including headers | A default flips by mistake (the inverted booleans) | M |
| 3.2 | `Meshes` resource; `rocketsim::init` once per process | Missing-mesh test still passes (`conversion/mod.rs:3911`) | `init` is global in RocketSim: two different mesh paths in one process **[verify]** behaviour | L |
| 3.3 | `analyze/pads.rs` (pad name votes) | Unit test on a synthetic replay; golden | HashMap iteration in the vote (`1170-1179` sorts, **[verify]** ties) | L |
| 3.4 | Contact alignment as an explicit stage in place of the recursive call (`1075-1085`) | Golden; the stage order is visible in `converter.rs` | The second pass's options differ subtly (`align_contacts = false`) | M |

**Milestone 4: the simulation loop (highest risk; one group of state per story)**

| # | Story | Acceptance | Risk | C |
| --- | --- | --- | --- | --- |
| 4.1 | `Reconstructor` struct holds the arena and the replay-wide state; the loop body unchanged inside a method | Golden | Borrow checker around `on_frame` and the arena | M |
| 4.2 | `FrameClock` and `FramePlan` (lags, phases, switches) | Golden; unit test of phase order | Order of equal lags (`sort_unstable_by` + `dedup`, `1387-1388`) | M |
| 4.3 | `stepping.rs`: `advance_to!` becomes a method | Golden | Captures of the macro | M |
| 4.4 | `CarTrack` with the spawn-pose hold state | Golden; the spawn tests (`3520-3910`) pass | Hold release rules across withheld frames | H |
| 4.5 | Dead shells and demolition holds into `lifecycle.rs` | Golden; shell tests (`3128-3519`) pass | Hold start after the interval | H |
| 4.6 | Counters, jump gate, dodge refresh into `actions.rs` | Golden; jump and dodge tests pass | Edge semantics across intervals | H |
| 4.7 | Air control layering and the boundary-value schedule into `car.rs`/`air/` | Golden; air tests pass | Held-jump rising-edge rule (RESULTS "Before the freeze") | H |
| 4.8 | Input fits behind `fits::FitContext` with a `ScratchArenas` pool and `Lookahead`; `RefCell` removed | Golden; fit tests pass; `Lookahead` refuses withheld frames (existing leak test) | Cache keys; withholding coverage | H |
| 4.9 | `finish_interval`: demolitions, sleeping packets, pads | Golden | Pad cooldown ordering | M |

**Milestone 5: after the simulation, and export**

| # | Story | Acceptance | Risk | C |
| --- | --- | --- | --- | --- |
| 5.1 | `annotate/`: touches, ball contacts, boost pickups, freshness | Golden | Recent-pose window bounds | M |
| 5.2 | `FrameSink`; JSONL and Parquet as sinks; `Reconstruction::write_*` | Golden for both formats; `verify_direct_parquet.py` passes | Parquet slot widths need `car_slot_count` before the pass (`parquet_export.rs:538`) | M |
| 5.3 | `export/atomic.rs` from the CLI | The CLI's rollback behaviour has tests (it has none today **[verify]**) | Windows rename semantics | M |
| 5.4 | Tests move next to their modules; synthetic `testkit` builder for `Observations` | Test count not lower (91); `cargo test` time recorded | Duplicate helpers | M |
| 5.5 | Speed check (section 6.4) | Within budget | — | L |

**Milestone 6: CLI**

| # | Story | Acceptance | Risk | C |
| --- | --- | --- | --- | --- |
| 6.1 | `replicar convert` (single file), `inspect`, `verify` | Same files as `convert_replay`; help text | — | M |
| 6.2 | Batch mode with `--jobs`, `--skip-existing`, failure summary | Converting train in a batch equals single-file runs | Memory with many jobs (replay-sized observations each) | M |

**Milestone 7: Python**

| # | Story | Acceptance | Risk | C |
| --- | --- | --- | --- | --- |
| 7.1 | `replicar-python` crate (pyo3, maturin) with `convert` returning Parquet bytes | `replay.arrays()` equals `load_columnar_numpy` of the CLI's file | pyo3/maturin versions **[decide]**; GIL release | M |
| 7.2 | Package `replicar` (merging `replicar.py`, `replay_columnar.py`), `read`, `convert_many`, stubs | The existing Python tests pass against the package | Import path change for current users (keep the old modules as shims for one release) | M |

**Milestone 8: documentation and release**

| # | Story | Acceptance | Risk | C |
| --- | --- | --- | --- | --- |
| 8.1 | Crate docs, `docs/` pages, `examples/` (convert, stream, inspect events), criterion bench | `cargo doc` without warnings; examples run | — | M |
| 8.2 | `clippy::pedantic` as warnings, module by module | Clean or allowed with reasons | Churn; golden guards it | M |
| 8.3 | Final verification: golden, `run_reference.sh` against v1, speed; CHANGELOG 2.0.0 | Section 1 "Done means" met | — | M |

## 8. Risks

- **Hidden order dependencies.** Moving state out of one function can change the order of side effects on
  the arena, for example a car's controls set before or after another car's packet. The golden gate
  catches it, and stories in milestone 4 are small so a difference points at a few lines.
- **Withholding leaks.** Today the withheld check is scattered (`frame_withheld`, `options.withheld_frames`).
  In v2 the `Lookahead` type is the only way to read future packets, and it carries the mask, which makes
  the property structural. The existing leak test (cutting the replay after a window leaves masked
  predictions unchanged, TEST_PROTOCOL.md section 2) must pass unchanged.
- **Scope creep.** Every "while we're here" improvement goes to section 9, not into a refactor story.
- **Parallel record files.** RESULTS.md and the protocol cite v1 binary names and lines. The README and
  TEST_PROTOCOL keep a short "v1 name → v2 command" table, and the test-assessment tag stays reproducible
  as v1.

## 9. After parity (candidates, each measured on its own)

- Speed: the air boundary-value solve is about half the base cost; a coarse-to-fine ground shift search
  and an LM stall rule are open (RESULTS.md:1684, PLAN.md backlog item 8).
- Schema v2: v2 names in the header options, enums as strings in Parquet dictionaries, and a published
  JSON Schema for the frame record.
- Modes and mutators, after VirxEC's `arena_config.rs`: detect from the replay, simulate when RocketSim
  supports them, refuse clearly otherwise.
- crates.io and PyPI releases, if the user wants them **[decide]**.

## 10. Questions for the user

1. **Sealed-split guard** in the published CLI: move it to the eval crate (recommended) or keep it?
2. **New dependencies**: `thiserror`, `clap`, `pyo3` + `maturin`; `criterion` for development. All are
   standard; each is a new pin.
3. **Publishing**: crates.io (needs `rocketsim` from crates.io) and PyPI, or local and git installs only?
4. **RocketSim update** before the golden reference (recommended) or after v2?
5. **Speed budget**: mean +3% and no replay +10% acceptable?
6. **Meshes for pip users**: document a route, or leave it as today?
