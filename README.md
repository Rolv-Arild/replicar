# Replay to RocketSim

This Rust project parses Rocket League soccar replays with `boxcars` and reconstructs one RocketSim state per replay network frame. It also retains replay observations that sit outside RocketSim state: score, clock, player statistics, observed actions, and events. See [PLAN.md](PLAN.md) for the field map and open work, [RESULTS.md](RESULTS.md) for measured accuracy, and [AGENTS.md](AGENTS.md) for contributor and agent guidance.

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

Rust callers can parse a schema-v1 rich frame and rebuild a detached native soccar `ArenaState` with `restoration::state_from_frame_json` and `restoration::restore_soccar_state`; the car slots come from `restoration::car_slots_from_header_json`. `apply_soccar_state_to_arena` seeds a live Arena and reports its own tick and pad cooldown error. A live Arena cannot adopt the serialized absolute tick, RNG, or private physics caches, so continuing simulation from it is not an exact replay continuation. To verify all rich frames in a Parquet file, run `cargo run --release --bin verify_state_restoration -- target/example.parquet`.

## Present accuracy limits

The converter corrects physical fields on fresh replay updates and simulates intermediate active ticks. Known car-body products select RocketSim hitboxes; unknown products use Octane. It passes observed throttle, steer, and handbrake, and estimates boost, jump, and dodge from replicated component evidence with motion gates. Offline aerial controls solve one constant RocketSim pitch/yaw/roll control between the fresh car angular packets that bracket each interval (spans up to 4 frames / 0.15 s; flags `--air-lookahead-frames`, `--air-lookahead-seconds`, `--air-lookahead-refine` on `evaluate_corpus`); they are model estimates using a later packet, not recovered player inputs, and they are unavailable for causal prediction across masked boundaries. Across withheld or unbridged intervals the converter keeps the control implied by the two latest earlier fresh packets (up to 0.15 s, large pitch/roll only) and routes steer to roll while the airborne handbrake is held; both use only earlier data and can be disabled with `--no-persist-past-air-controls` and `--no-infer-air-roll-from-handbrake` on `evaluate_corpus`. Car packet timing remains unresolved, so motion-derived intervals are diagnostics and do not alter RocketSim's 120 Hz timeline. Score and clock come from replay packets, never simulated goals. Some transient car actors remain unlinked to players. Corpus-wide train and validation results are in [RESULTS.md](RESULTS.md). The `test` split remains untouched until settings are frozen.

The provided `replays/` and `collision_meshes/` folders, generated `target/` outputs, and IDE files are ignored by Git.
