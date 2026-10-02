# Replay to RocketSim

This Rust project parses Rocket League soccar replays with `boxcars` and reconstructs one RocketSim state per replay network frame. It also retains replay observations that sit outside RocketSim state: score, clock, player statistics, observed actions, and events. See [PLAN.md](PLAN.md) for the field map and open work, [RESULTS.md](RESULTS.md) for measured accuracy, [ROCKETSIM_NOTES.md](ROCKETSIM_NOTES.md) for apparent RocketSim inaccuracies found along the way, and [AGENTS.md](AGENTS.md) for contributor and agent guidance.

## Setup and conversion

Place the supplied RocketSim meshes under `collision_meshes/soccar/`. Run from the repository root:

```powershell
cargo test
$replay = (Get-ChildItem replays/train/1v1/*.replay | Select-Object -First 1).FullName
cargo run --release --bin convert_replay -- $replay target/example.jsonl
```

The CLI accepts an optional third argument for the mesh directory. Use an `.jsonl` output path for JSON Lines or `.parquet` for direct Rust Parquet output. JSONL has one versioned `header` record followed by one `frame` record per network frame. Both formats retain the replay SHA-256, dependency revisions, conversion options, actor-to-car slots, diagnostics, complete RocketSim state, observations with source-frame freshness, events, and position residuals. Parquet also has typed columns for dense Python reads. Player and car observations retain raw loadout body product IDs; known IDs select RocketSim hitboxes, while unknown or absent IDs use Octane and remain visible in diagnostics. The [body catalog](data/README.md) documents the supplied item snapshot, source-backed mappings, and unresolved products. Header final scores must never be used as the score at an earlier frame.

To reproduce the development accuracy report, run `cargo run --release --bin evaluate_corpus -- replays/train target/train-conversion-metrics.json` and use `replays/validation` for the held-out development check. The evaluator records one-step pre-correction residuals and four-frame masked predictions for position, linear velocity, rotation, angular velocity, and boost against hold or linear baselines. It evaluates the active primary car when multiple actors share a player. Pass `--no-inferred-boost` for the boost-input ablation or `--octane-hitbox` to use the original all-Octane setup. `convert_replay` accepts the same flags. `calibrate_boost replays/train` reports the evidence for interpreting the boost activation counter. `audit_packet_timing` reports raw car/ball update gaps separately from motion-derived intervals; see `RESULTS.md` for its train/validation protocol. Keep `replays/test` for the final frozen evaluation.

For a frame-level masked rotation investigation, pass one train replay and `--rotation-trace`:

```powershell
$replay = "replays/train/1v1/00a0da63-492e-4ab7-8a07-16cd5d14dcb4.replay"
cargo run --release --bin evaluate_corpus -- $replay target/one-replay-report.json --rotation-trace target/one-replay-trace.jsonl
```

Each JSONL row describes a masked primary-car frame, including fresh versus held body fields and source frames, up to three fresh angular packets strictly before the mask window, observed controls, simulated states and interval events, and per-frame errors. Rotation errors are null when the frame has no eligible fresh target packet. The trace uses the evaluator's own mask schedule and can also be produced for a whole split. An experimental `--gated-low-air-angular` option carries a prior near-ceiling angular packet through guarded low-air intervals; it is off by default and is supported by both `evaluate_corpus` and `convert_replay`. See `RESULTS.md` for inspected windows, paired train/validation results, and limitations.

State position and linear velocity use Rocket League unreal units (UU and UU/s); state angular velocity uses radians/s. Rotation is three RocketSim basis columns. The timeline is rounded to 120 Hz from the replay's first timestamp. RocketSim's arena tick advances only during continuous `Active` intervals, so it can differ from the timeline tick. Simulated events and unobserved state fields are estimates, not replay truth.

```python
import sys
sys.path.insert(0, "python")
from replay_to_rocketsim import read_header, iter_frames, load_numpy

header = read_header("target/example.jsonl")
for frame in iter_frames("target/example.jsonl"):
    print(frame["replay_time"], frame["state"]["ball"]["physics"]["position"])
    break
arrays = load_numpy("target/example.jsonl")  # requires NumPy
print(arrays["car_position"].shape)  # frames × car slots × XYZ
```

`iter_frames` streams rich records with only Python's standard library and also accepts `.jsonl.gz`. `load_numpy` makes two passes to allocate dense arrays; it returns time/ticks, ball and car position, rotation and velocity, car boost, demo state, controls, boost pads, team scores, and match clock. It uses NaN for absent numeric observations and a mask for car slots missing from a RocketSim snapshot. Array control channels are in `control_axes_order` and `control_buttons_order`; they include inferred boost. Use the streaming records for original action counters and field provenance, complete state, statistics, and events.

The Rust CLI can write Parquet without an intermediate JSONL file:

```powershell
cargo run --release --bin convert_replay -- $replay target/example.parquet
```

Install the pinned Python extra to read Parquet. The earlier Python second-stage writer remains available for Arrow IPC or comparison:

```powershell
python -m pip install -r python/requirements-columnar.txt
python python/replay_columnar.py target/example.jsonl target/example.parquet
```

```python
import sys
sys.path.insert(0, "python")
from replay_columnar import read_columnar_header, iter_columnar_frames, load_columnar_numpy

header = read_columnar_header("target/example.parquet")
arrays = load_columnar_numpy("target/example.parquet")
print(arrays["car_position"].shape)
first_rich_frame = next(iter_columnar_frames("target/example.parquet"))
```

The columnar format keeps complete observations, simulated events, residuals, and hidden RocketSim fields in `frame_json`; typed columns cover the dense state, controls, pads, score, and clock. Parquet is recommended for repeated Python ML reads because the loader can skip the rich payload. JSONL remains faster to export and easier to inspect. The direct Rust Parquet path simulates twice to determine final car-slot widths before writing 512-frame row groups; it avoids retaining all simulated snapshots, but replay bytes and extracted observations remain in memory for offline lookahead. JSONL still retains all snapshots before writing. [RESULTS.md](RESULTS.md) has format, memory, and runtime measurements.

The Rust CLI also writes typed per-record tables next to the main file, so the scoreboard, touches, contacts, pickups, fitted inputs, and events need no per-frame JSON parsing (pass `--no-event-tables` to skip them). The main file gets four more columns appended after `frame_json` (existing columns and positions are unchanged): `scoreboard_period` and `scoreboard_clock_state` (dictionary-encoded strings, read by PyArrow as categoricals) and nullable Float32 `scoreboard_seconds_remaining` and `scoreboard_overtime_seconds`. For `game.parquet` the tables are `game.touches.parquet`, `game.ball_contacts.parquet`, `game.boost_pickups.parquet`, `game.fitted_inputs.parquet`, `game.packet_lags.parquet`, `game.events.parquet` (goals and demolitions) and `game.pad_pickups.parquet`, one row per record with the `frame` it belongs to; they are always written, empty when a replay has none. A null is an unknown or inapplicable value, never zero. [RESULTS.md](RESULTS.md) ("Parquet columns and record tables") lists every column.

```python
import pyarrow.parquet as pq

frames = pq.read_table("target/example.parquet", columns=["frame", "scoreboard_clock_state", "scoreboard_seconds_remaining"])
touches = pq.read_table("target/example.touches.parquet").to_pandas()    # frame, car_slot, tick, contact_point
events = pq.read_table("target/example.events.parquet").to_pandas()      # kind: goal_scored_on | demolish | dodge_refreshed (car, refreshed_count)
demolitions = events[(events.kind == "demolish") & (events.repeat == False)]
```

`load_columnar_numpy` (and `load_numpy` for JSONL) also return the scoreboard as `scoreboard_period` and `scoreboard_clock_state` (object arrays, `None` where unknown) and `scoreboard_seconds_remaining` and `scoreboard_overtime_seconds` (float32, NaN where unknown); `read_record_tables("target/example.parquet")` returns the side tables that exist as `pyarrow.Table` values keyed by name. The Python `write_columnar` writer writes the scoreboard columns but not the record tables (use `convert_replay` for those). The Rust writer converts in a single pass and stores the replay header (`replay_header_json`, with the car slots and the conversion's diagnostics) in the Parquet file's key-value metadata, written at close: `read_columnar_header` reads it there, while PyArrow's `ParquetFile(...).schema_arrow.metadata` only holds `columnar_version` and `pad_config_json` (use `ParquetFile(...).metadata.metadata` for the rest).

Rust callers can parse a schema-v1 rich frame and rebuild a detached native soccar `ArenaState` with `restoration::state_from_frame_json` and `restoration::restore_soccar_state`; the car slots come from `restoration::car_slots_from_header_json`. `apply_soccar_state_to_arena` seeds a live Arena and reports its own tick and pad cooldown error. A live Arena cannot adopt the serialized absolute tick, RNG, or private physics caches, so continuing simulation from it is not an exact replay continuation. To verify all rich frames in a Parquet file, run `cargo run --release --bin verify_state_restoration -- target/example.parquet`.

## Present accuracy limits

The converter corrects physical fields on fresh replay updates and simulates intermediate active ticks. Known car-body products select RocketSim hitboxes; unknown products use Octane. It passes observed throttle, steer, and handbrake, and estimates boost, jump, and dodge from replicated component evidence with motion gates. A grounded car is driven by the controls of the frame that ends each interval from about 2 + gap/2 ticks before that frame's time (the replicated change happened before it was first seen; offline only, needs the inferred packet lags, never used for a frame withheld by an evaluator; `--no-lookahead-ground-controls` disables). For a flat-ground car it then fits one timing shift of those control switches (-8 to +40 ticks) against the second-next fresh packet and drives the interval to the next packet with it, so the residual at that packet is held out (offline, needs packet lags; `--no-fit-ground-control-timing` disables). A jump from the ground gets the same treatment (one shift of the jump counter's switches, fitted against the second-next packet; `--no-fit-jump-timing` disables). An airborne dodge start and its pitch cancel are fitted the same way (`--no-infer-dodge-start` disables; the first fresh packet after an activation has no chain lag, so its tick is inferred from the path with the fitted start (`--no-infer-dodge-first-packet`) and a dodge that starts after it is planned for the interval after it (`--no-defer-dodge`); the flip's pitch cancel is fitted on the next packet, so rotation and angular velocity in flip windows are in sample: `--flip-cancel-holdout` (with `--flip-cancel-packets n`; `--flip-cancel-source` selects the fit on the next packet, on the previous interval, or the rule of `external/RLCarInputSolver`) leaves the packet it is used for out of the fit as a check); a jump from the ground followed by a dodge is fitted together (jump shift, dodge press and cancel). The ground and jump fits work on any surface (floor, wall, ramp, ceiling). None of these timing fits refuses spans near other cars (their contacts are not modelled, but refusing removed coverage without protecting the fits). RocketSim is pinned to `0b02051` (2026-09-28); earlier revisions dropped the extra ball-car hit impulse, which `--apply-hit-impulse` can re-add for those revisions only. While a car is flipping, the converter fits how much of the flip's pitch torque the player cancelled (an input the replay lacks) by simulating candidates against the next packet at its exact tick (`--no-infer-flip-cancel` disables). Offline aerial controls solve one constant RocketSim pitch/yaw/roll control between the fresh car angular packets that bracket each interval (any bracketing pair in the active phase; optional caps `--air-lookahead-frames`, `--air-lookahead-seconds` and `--air-lookahead-refine` on `evaluate_corpus`); they are model estimates using a later packet, not recovered player inputs, and they are unavailable for causal prediction across masked boundaries. Across withheld or unbridged intervals the converter scales the control implied by the two latest earlier fresh packets by a measured median-persistence table (`calibrate_air_control_persistence`; nothing beyond 0.2 s) and uses observed steer for yaw, or for roll while the airborne handbrake is held; both use only earlier data and can be disabled with `--no-persist-past-air-controls` and `--no-infer-air-roll-from-handbrake` on `evaluate_corpus` (`--legacy-persist-gates` restores the earlier hand-tuned gate). Each ball and car packet was generated somewhere inside its replay frame window, so by default the converter infers its lag (0-3 ticks) from chained packet motion, applies the correction at that packet time and still reports every state at the frame time (`packet_lag_ticks` records the inferred lag and its source; disable with `--no-infer-packet-lag` on `evaluate_corpus`; the `zero_packet_lag` library option instead treats every fresh packet as lag-free, which is right for an offline replay such as the BakkesMod dump's; `dump_reconstruction` thins such a replay's car packets and scores the reconstruction against the dump's true states). This offline inference uses later packets and is not available to causal prediction; RocketSim's 120 Hz timeline is otherwise unchanged. The controls on an exported car state are the action applied from that state: observed throttle, steer and handbrake (with the control timing fitted offline), boost, jump and dodge inferred from replicated counters, and, in the air, the per-interval pitch/yaw/roll of the boundary-value solve (the mean of the true inputs over the next interval where the solve covers it, not per-tick inputs). Fitted jump and dodge presses are listed in `fitted_inputs` on the frame where they are fitted, together with the airborne intervals whose pitch/yaw/roll the boundary-value solve chose (`kind` `air`, with the interval length in `span_ticks`; their pitch, yaw and cancel fields are 0 in JSON and null in Parquet). The recording client's own car leads the server by a median 16 ticks in the observed controls, which the ground timing fit allows for (shifts -8..+40). `ball_contacts` are contacts found from the replay's ball packets (a ball-only rollout between consecutive fresh ball packets: a velocity it cannot reach is a touch; estimated tick, nearest car, whether a simulated touch falls in the interval) and are the evidence to prefer for touches. `boost_pickups` lists the new pad pickups of a frame with the pad, the car the replay names and whether that car's path reaches the pad; skip `demolish` events with `repeat` true. `scoreboard` gives the match clock and its lifecycle per frame (`clock_state`: pregame, countdown, kickoff = clock held until the first touch, running, expired = at 0 waiting for the ball to touch the ground, decided, goal_pause; fractional `seconds_remaining` and, in overtime, `overtime_seconds` counting up), fitted from the replay's integer clock. Each event is reported once: ball touches only exist in the simulation (`touches`: the first tick of each car-ball contact; the raw `simulated_events` keep RocketSim's per-tick hits, so do not add the two), goals and demolitions come from the replay (`events`), and boost pickups from the replay's `pad_pickups` (skip records with `repeat` true, which re-announce an earlier pickup; simulated pickups are removed while the pads are blocked). Observed events: `goal_scored_on` (once per team per post-goal phase) and `demolish` (from `ReplicatedDemolish*`; the converter demolishes the victim for 3 s, `apply_observed_demolitions`). Scores and the clock are exact and prompt; per-player stats can arrive up to about 1.8 s late on a client replay, and in overtime `seconds_remaining` counts up from 0. `scripts/rlbot_events_check.py` scores all of these against an RLBot recording. Ball chains continue across hits (the exact free-flight paths before and after a hit meet at it, which fixes the ticks between the two packets: `--no-ball-hit-chains` disables) and the ball's chain runs are placed against the cars' with the ball-minus-car server tick offset of the replication, estimated from the car-ball contacts of the replay (`--no-estimate-ball-car-offset` disables; needs 20 bridged hits; 3.1 ticks on the LAN remote-client games). Packets are exact 120 Hz server states, but the frame time does not say which tick (a stationary offset of about 0-4 ticks), so a raw packet is not the state at its frame time; `evaluate_corpus --aligned-targets` compares masked predictions with the offline reconstruction at the same frame time to remove that timing noise from the error. Score and clock come from replay packets, never simulated goals. Some transient car actors remain unlinked to players. Corpus-wide train and validation results are in [RESULTS.md](RESULTS.md). The `test` split remains untouched until settings are frozen.

The provided `replays/` and `collision_meshes/` folders, generated `target/` outputs, and IDE files are ignored by Git.
