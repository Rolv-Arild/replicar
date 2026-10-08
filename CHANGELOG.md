# Changelog

## Unreleased

- The interval that ends at a goal frame is simulated: the ticks in which the ball crosses the line were left out, and the goal frame's row repeated the previous `sim_tick`.
- A jump, double jump or dodge whose impulse the frame's update already shows is pressed before that update, so the controls show it (it was set as flags only): presses that took effect in the game and are missing from the file, human 1.3-2.0% -> 0%, bots 5.1% -> 0.8% (RocketSim recordings).
- Dodges whose takeoff reaches the updates after the counters (speed flips at kickoff) are pressed when the car leaves the ground instead of being lost, and a flip reset no longer gets undone by the counter rule: flips set without a simulated flip, kickoff 14.6% -> 2.2%, open play 10.0% -> 3.8%; held-out one-step rotation better in 43 of 60 validation replays, worse in none.
- Folder conversion records a panicking replay as an error row instead of ending the run.
- `replicar.rocketsim`: `jump_time` is 0 until the car has jumped, as in the bindings; the guide lists the fields whose meaning differs between the Rust port and the C++ bindings, and the bindings' air throttle while boosting.

## 2.0.0 (2026-10-07)

A rewrite of version 1: every stage and the evaluator's held-out reports were checked equal to version 1's on the
120 development replays (RESULTS.md, the "v2:" sections) before the deliberate changes below. The file holds much more.

- A Cargo workspace: `replicar-format` (the file, readable without the parser or the simulator), `replicar` (the
  library), `replicar-cli` (the `replicar` command), `replicar-python` and the `replicar` Python package.
- One Parquet file per replay instead of a main file with `frame_json` and seven record tables: 2.1-2.5 times the
  replay instead of 15-17.5. Rows only for frames in play segments (`--all-frames` for the rest), per-player
  columns, column groups (`state`, `game`, `updates`, `future` by default; `resimulation`, `network`, `diagnostics`
  on request), `float32` or `quantized` precision. JSON Lines output is gone.
- Renamed concepts (docs/glossary.md, "v1 to v2"): packet lag is the update tick, freshness the `updates` group,
  episodes are play segments, and the next-goal labels became `future_segment_end`, which never looks past the
  frame's own segment.
- Resimulation from a file's `resimulation` group and the replay, about nine times faster than converting;
  RocketSim states restored from a file; folder conversion in parallel with an index; a Python reader with NumPy
  arrays and a native extra to convert.
- RocketSim from crates.io (`0.2.7`), measured neutral against `9910c58`.
- mimalloc as the allocator of the command and the Python module: folder conversion scales past 16 threads (the Windows system allocator serialized them).
- A flipping car's air controls are solved in segments of about eight ticks instead of four, 15% faster with the same held-out accuracy.
- Inputs between frames, checked against the true per-tick inputs of RLBot and RocketSim recordings: a jump is held while its counter is active (it was released at the first update showing it), analog throttle and steer ramp across a change instead of stepping, the steer switches in the air too, and the ground control timing fit simulates the ball instead of refusing near it. Held-out one-step car velocity p90 about 11% lower and position p90 about 12% lower on validation, velocity better in every replay (RESULTS.md, "v2: inputs between frames").
- A row per simulated 120 Hz tick by default (`frame_row` marks the replay frames' own ticks), every N-th tick
  (`--tick-step N`) or a row per frame (`--rows frames`). A row's controls are those RocketSim applied in the step
  after its tick, so the fitted press timing, control timing and air and ground schedules are in the file; in a row
  per frame this also changes the controls of cars on per-tick schedules (3-11% of a car's frame rows).
- Where each row's controls came from: `car_<i>_air_controls_source` and `car_<i>_ground_controls_source`.
- The players' match statistics as they happen (`stat_events`: goal, assist, save, shot, demolition and, from the
  September 2026 builds, epic save, clear, center, aerial hit, first touch, crossbar, bicycle and juggle hits, flip
  reset, demolished), goals with their scorer and assister, each player's `final_stats`, and the statistics the
  replay's build counts (`counted_stats`: 0 is then known, otherwise unknown).
- `players_table()` and `long()` in the Python reader, `replicar inspect --players`.
- `replicar.rocketsim` (a row as a state of mtheall's RocketSim bindings) and `replicar.rlgym` (a row as an RLGym
  `GameState`); neither steps anything.
- Linux: tested, with CI on Linux and Windows and a release workflow for the wheels (Linux, Windows, macOS) and the
  command. The header records the `platform`: the platforms' maths libraries differ in the last bit, so a file is
  resimulated only where it was converted (ROCKETSIM_NOTES.md).
- Memory: the writer encodes 16,384 rows at a time and a conversion keeps only the rows its file will have (a long
  3v3 replay: 330 MB with tick rows, 210 MB with frame rows).

## 1.0.1 (2026-10-04)

- Renamed from `replay-to-rocketsim` to `replicar` (replica car, replicate, and network replication, which is what a replay records). The Rust crate is now `replicar` and the JSON Lines Python loader `python/replicar.py`; the output format is unchanged.

## 1.0.0 (2026-10-04)

First release.

- Converts standard soccar replays into one RocketSim state per network frame: replay packets placed on their inferred server tick, RocketSim (`9910c58`) simulating every 120 Hz tick in between, and unrecorded inputs (control, jump and dodge timing, flip cancels, air pitch/yaw/roll) fitted against later packets.
- Exports JSON Lines or Parquet with seven record tables, observations with their freshness, replay events, the scoreboard with a fitted fractional clock, freshness columns, observed ping, and future-derived training labels. Python loaders for both formats; exact restoration of a RocketSim state from an exported frame.
- Evaluated once on 60 sealed test replays (tag `test-assessment-1`; RESULTS.md, "Test-split assessment"): every replay converted and the errors matched the development splits. The conversion state is unchanged since that run.
- Release clean-up (since tag `pre-cleanup`): the one-off experiment tools and the measured-and-rejected options were removed, `ConvertOptions` went from 63 to 22 fields, `conversion.rs` was split into modules, and the code is formatted and clippy-clean. Default output is byte-identical to before the clean-up on all 120 development replays.
