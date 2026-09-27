# Replay to RocketSim

This Rust project parses Rocket League soccar replays with `boxcars` and reconstructs one RocketSim state per replay network frame. It also retains replay observations that sit outside RocketSim state: score, clock, player statistics, observed actions, and events. See [PLAN.md](PLAN.md) for the field map and open work, and [RESULTS.md](RESULTS.md) for measured accuracy.

## Setup and conversion

Place the supplied RocketSim meshes under `collision_meshes/soccar/`. Run from the repository root:

```powershell
cargo test
cargo run --release --bin convert_replay -- replays/train/1v1/example.replay target/example.jsonl
```

The CLI accepts an optional third argument for the mesh directory. Its output is JSON Lines: one versioned `header` record followed by one `frame` record per network frame. The header records the replay SHA-256, dependency revisions, conversion options, actor-to-car slots, and diagnostics. Each frame contains replay time, timeline tick, a RocketSim state snapshot, the typed observations with source-frame freshness, simulated events, and position prediction residuals. The replay observations distinguish last-seen values from fresh packets through each field's `frame` and `source` keys. Header final scores must never be used as the score at an earlier frame.

To reproduce the development accuracy report, run `cargo run --release --bin evaluate_corpus -- replays/train target/train-conversion-metrics.json` and use `replays/validation` for the held-out development check. The evaluator records one-step residuals and a four-frame masked-physics prediction against hold-position and linear baselines. Pass `--no-inferred-boost` for the boost-input ablation. `calibrate_boost replays/train` reports the evidence for interpreting the boost activation counter. Keep `replays/test` for the final frozen evaluation.

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

`iter_frames` streams rich records with only Python's standard library. `load_numpy` makes two passes to allocate dense arrays; it returns time/ticks, ball and car position, rotation and velocity, car boost, demo state, controls, boost pads, team scores, and match clock. It uses NaN for absent numeric observations and a mask for car slots missing from a RocketSim snapshot. Array control channels are in `control_axes_order` and `control_buttons_order`; they include inferred boost. Use the streaming records for original action counters and field provenance, complete state, statistics, and events.

JSONL is intentionally inspectable and currently large: one 12,292-frame training replay produced 115 MB. A compact columnar format and a truly streaming Rust conversion API are planned. The current Rust converter retains all snapshots in memory before writing.

## Present accuracy limits

The first bridge corrects physical fields only on fresh replay updates and simulates intermediate active ticks. It currently assigns every car an Octane hitbox and passes observed throttle, steer, handbrake, and an inferred boost-active signal to RocketSim. Raw jump, double-jump, dodge, and flip component counters are exported as observations; their timing has not been calibrated as controls. Aerial inputs are not recovered yet. Score and clock come from replay packets; they are never inferred from simulated goals. Some transient car actors remain unlinked to players. Corpus-wide train and validation prediction baselines are in [RESULTS.md](RESULTS.md). The `test` split remains untouched until settings are frozen.

The provided `replays/` and `collision_meshes/` folders, generated `target/` outputs, and IDE files are ignored by Git.
