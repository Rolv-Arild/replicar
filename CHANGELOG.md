# Changelog

## Unreleased

Conversion output changes (inputs in the air, and on the ground through the jump and ground timing fits). The air timings read the update the interval ends at, so they are off where the air lookahead or the in-sample fits are and do not change the evaluator's held-out results; the two fit changes run there too and change them slightly (better).

- The boost of an airborne car switches at the tick the update's velocity shows (it switched at the update): air boost press error 2.76 -> 1.46 ticks on a host replay's bots. Counted as `air_boost_switches`; `SimulationOptions::air_boost_timing` turns these timings off.
- A held jump in the air is released at the tick the update's velocity along the car's up axis shows (`jump_releases_timed`); a double jump or dodge the update already shows is pressed at the step its position shows (`air_presses_timed`).
- The dodge start fit also matches the first update after the dodge: flip start error on a bot match 4.0 -> 1.7 ticks, on client replays about 0.5 ticks lower; the replay's dodges with a simulated flip 98.7 -> 98.9% (train).
- Near the ball or another car the air timings above still run unless the car can touch it in the interval (a copy of the arena stepped through it); they were off within 400 UU of the ball and 300 UU of a car. Air boost press error on a human `.rlpr` match 1.82 -> 0.38 ticks.
- An airborne car's throttle switches at the tick the update's velocity shows, when it is not boosting (`air_throttle_switches`).
- The jump timing fit also tries moving the controls of the jump's frame with the jump (bots press throttle, steer and handbrake with it): ground throttle change error on a host replay's bots 0.57 -> 0.15 ticks, handbrake press 1.82 -> 1.25. This one changes the evaluator's held-out results too (slightly better).
- The ground timing fit no longer misses a control change whose previous frame lies outside its window (more of a client replay's changes are fitted).
- A jump released while the car's wheels are still on the ground (a jump a few ticks old) is released at the tick a simulation of the interval matches to the update (it was released at the update): bots' ground jump releases exact 26-30% -> 74-76% on the host replays.
- No spurious ground jump press from a jump counter re-sent with its even value (on curved walls) or a jump the simulation already flew before a dodge and a landing. This one changes the evaluator's held-out results (slightly better).
- On the ground a boost the replay shows for no longer than RocketSim's 0.1 s latch may be released earlier inside it, where the ground timing fit finds that fits better (RocketSim drives at full throttle while the boost control is down); the network's boost presses it again as before, so a feathered boost keeps its re-presses; held-out results move too.
- Near the ball or another car, the air boost switch, the double-jump and dodge press and the air jump release are timed by simulating the interval with the ball and the other cars (they fell back to the rule): air near the ball four frames ahead 1.75 -> 1.07 UU p90 on a host replay.
- The ground timing fit tries control shifts from -24 to +40 ticks (from -8) and fits an interval whenever a shift can move a change into it: a client's replay shows other cars' control changes up to 40 ticks before their effect. On the client recordings ground steer and throttle labels are closer to the true inputs; about 5% slower conversions.
- The jump timing fit also times a jump whose counter a client's replay shows in the frame of the update with the takeoff (or already at the update, the car still on the ground), and one seen only as the counter rising by two between frames; a candidate that would press with the car in the air is dropped; when it keeps the other controls on the midpoint rule, its schedule sets only the jump for a car with a fitted control shift, so they follow the shift and the ramps. Ground jump press error on the client recordings 3.02 -> 2.72 ticks (bots) and 5.21 -> 4.58 (humans); held-out evaluator results better. The resimulation group stores such a schedule's entries with the jump alone (the other controls "unchanged"); earlier versions cannot resimulate a file with one.
- Near the speed limit an airborne car's boost switch and double-jump or dodge press are timed by simulating the interval (the boost switched at the update): air boost release error on the host replays' bots 2.66 / 2.51 -> 2.32 / 2.03 ticks, humans 1.11 / 0.87 -> 0.57 / 0.44.
- A planned dodge sets only the jump, pitch, yaw and roll; throttle, steer, boost and handbrake follow their switches (they were held as when the dodge was planned): air steer error of the host replays' bots 0.0335 / 0.0398 -> 0.0302 / 0.0363, handbrake disagreement of the client recordings' humans 2.4% / 2.2% -> 2.0% / 2.0%.
- A car that shows its controls ahead of their effect (a remote player whose inputs the server buffers, about one player per match in the corpus: a fitted control shift of 6 ticks or more in most of its frames) has its airborne boost and throttle follow its shifted switches (they switched at the update), and a held jump released its shift after the counter shows the release. Held-out one-step car velocity p90 37.06 / 38.08 -> 35.41 / 36.47 UU/s (train / validation); on the client recordings the humans' air boost press error 3.69 / 4.67 -> 2.36 / 2.84 ticks.
- RocketSim 0.3.3 (any compatible release from 0.3.3; 0.3.2 is no longer accepted): it added a field to `BoostPadState` and dropped `Copy`, so replicar did not build with it and the daily dependency check had rejected it. Conversions are nearly unchanged (held-out pooled rows in the fourth digit).
- A leading car's air controls taken from its steer (where nothing solved them) follow its shifted steer.
- A leading car's handbrake in the air follows its shifted switch too (it picks roll or yaw for the steer), and a double jump its counter shows before any update does is pressed where a later update places it, or its control shift after the counter's press at the latest (it was pressed before the counter's update). Held-out one-step car velocity p90 35.41 / 36.47 -> 35.16 / 36.27 UU/s (train / validation).
- The ground timing fit of a car whose control shift is at least 6 ticks also reads the earlier frames a shift can move into the span, and the jump timing fit that moves the jump frame's other controls moves every later frame with them. Held-out one-step car velocity p90 35.16 / 36.27 -> 34.29 / 35.19 UU/s (train / validation); client humans' steer |error| 0.069 -> 0.065 / 0.063; conversion about 7% slower.
- The air boost, throttle and jump release timings leave a car whose ground schedule writes those controls to it; a double jump or dodge timed by simulation gets the interval's switches and counts a confirmed midpoint rule as a timing.
- Fixed: a panic (`f32::clamp`) in the analytic jump release when the jump must push through the whole interval.

## 2.1.1 (2026-10-26)

- A kickoff's segment starts at the cars' release: its first row is every car at rest on its spawn with the controls it starts with (clock phase `kickoff`), simulated to the first frame in play from the release tick that best reproduces the cars' first updates. It was the first frame in play, the cars already 1-3 ticks into the drive. Counted as `kickoffs_from_release` in the simulation diagnostics.

## 2.1.0 (2026-10-23)

RocketSim 0.3 (any compatible release from 0.3.2; 0.2 releases are no longer accepted). Conversion output changes. Rust users: `replicar::restore` and the conversion callback now hand out RocketSim 0.3 types, so code that uses them with RocketSim 0.2 needs updating.

- RocketSim 0.3.2 (and glam 0.34, which must match it). Against 0.2.12 the conversions are unchanged to the last digit on every check, about 4% slower. RocketSim 0.3 keeps a car's contact state private: `replicar::restore` builds `CarState`s through its setters; put one into an arena with `Arena::set_car_state_full` (`set_car_state` now takes a `CarCore` and clears the contacts).
- `flip_time` as RocketSim counts it since 0.2.12 (from the end of the flip, and on the ground; the pitch is locked while it is below 0.3 s): an observed flip reset no longer locks the car's pitch for 0.3 s.
- Flip resets the simulation misses are inferred (downstream feedback item 18): a double jump or dodge the counters show for a car whose flip RocketSim says is used up gets the reset first, so it flips with its direction. The replay's dodges with a simulated flip: 96.3% -> 97.8% (train), 96.5% -> 97.7% (validation); airborne rows with a flip after the double jump 0.27% -> 0.03%. Counted in the header's simulation diagnostics as `flip_resets_inferred`.
- The jump of a jump-and-dodge whose dodge a fit plans, or whose dodge counter rises in the takeoff's frame, is pressed: takeoffs without a jump input, kickoff 2.9% -> 1.8%, open play 3.4% -> 2.7% (train).
- The daily dependency workflow opens an issue when a newer release is outside the requirement (as RocketSim 0.3 was).
- Jumps off walls and the ceiling are pressed (the takeoff is recognised along the car's up axis, not the world's): open-play takeoffs without a jump input 2.7% -> 1.6% (train); jump presses missed against the true inputs 8.8% -> 4.3% and 8.3% -> 3.5% (LAN client bots).
- A dodge or double jump planned while the jump is still held releases it first: dodges set as flags without a flip 1.1% -> 0.3% (train), 1.3% -> 0.3% (validation).
- The air fits know RocketSim's pitch lock after a flip: airborne roll labels closer to the true inputs (for example 0.169 -> 0.159 mean error on a human's recordings).
- `evaluate --mask-block N`: masked windows every N frames (default 100) for denser masked samples.

## 2.0.4 (2026-10-14)

Output unchanged for standard replays except through the RocketSim release (0.2.11 in the lock).

- boxcars: any compatible release from 0.12.1 (the newest is assumed best), instead of exactly 0.12.0. 0.12.1 decodes `TAGame.Car_TA:MaxNumJumps`, so the private matches that failed to parse at frame 0 (5 of 140,328 downstream replays) now convert; the attribute is sent only by matches with a jump mutator (3-5 jumps), which the diagnostics now report as non-standard.
- Daily dependency workflow: a newer RocketSim or boxcars release is tested, committed to the lock and built into the latest release's wheels (with a wheel build number) without a replicar release. The release workflow can publish to PyPI by trusted publishing (off until configured).
- Builds with RocketSim releases that add fields to its state types (0.2.11 added two to `CarState` and broke the build of 2.0.3 when Cargo picked it); the lock follows RocketSim 0.2.11.

## 2.0.3 (2026-10-09)

Conversion output changes with the RocketSim release (each file records it); later RocketSim releases need no replicar release.

- RocketSim: any compatible release from 0.2.9 (the newest is assumed best), instead of exactly 0.2.7. Each file's header records the release that simulated it, read from the build's `Cargo.lock`; CI and the release builds take the newest release. 0.2.9 against 0.2.7: held-out one-step car velocity and position medians about 7% lower, tails unchanged; closer to the game for airborne cars (true-input one-step velocity p90 about 10-15% lower).

## 2.0.2 (2026-10-09)

Fixes from the downstream project's feedback on 2.0.1. Conversion output changes; 2.0.1 was not deterministic, so convert its files again with 2.0.2.

- Conversion is deterministic again: 2.0.1 could give different states for the same replay in different processes (a deferred dodge looked its car up in a hash map). A test converts replays twice and compares their states.
- First jumps of quick jump-and-dodge combinations (speed flips at kickoff) are pressed: a takeoff the updates show while the simulated car is still on the ground gets its jump. Takeoffs without a jump input, kickoff 12.3% -> 2.4%, open play 4.4% -> 3.2%.
- `final_scores` is never null: a team the replay never put on the scoreboard has the goals its play segments end in.
- Folder conversion rewrites `index.parquet` every 100 replays, so an interrupted run keeps most of its index.

## 2.0.1 (2026-10-09)

Fixes from a downstream project's feedback (RESULTS.md, "User feedback from a downstream project"). Conversion output changes: files made with 2.0.0 should be converted again.

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
