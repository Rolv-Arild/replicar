# Changelog

## 2.0.0 (unreleased, branch `v2`)

A rewrite with the same reconstruction: every stage, the full default conversion and the evaluator's reports equal
version 1's on the 120 development replays (RESULTS.md, the "v2:" sections).

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
- The one change of results: a flipping car's air controls are solved in segments of about eight ticks instead of four, 15% faster with the same held-out accuracy.

## 1.0.1 (2026-10-04)

- Renamed from `replay-to-rocketsim` to `replicar` (replica car, replicate, and network replication, which is what a replay records). The Rust crate is now `replicar` and the JSON Lines Python loader `python/replicar.py`; the output format is unchanged.

## 1.0.0 (2026-10-04)

First release.

- Converts standard soccar replays into one RocketSim state per network frame: replay packets placed on their inferred server tick, RocketSim (`9910c58`) simulating every 120 Hz tick in between, and unrecorded inputs (control, jump and dodge timing, flip cancels, air pitch/yaw/roll) fitted against later packets.
- Exports JSON Lines or Parquet with seven record tables, observations with their freshness, replay events, the scoreboard with a fitted fractional clock, freshness columns, observed ping, and future-derived training labels. Python loaders for both formats; exact restoration of a RocketSim state from an exported frame.
- Evaluated once on 60 sealed test replays (tag `test-assessment-1`; RESULTS.md, "Test-split assessment"): every replay converted and the errors matched the development splits. The conversion state is unchanged since that run.
- Release clean-up (since tag `pre-cleanup`): the one-off experiment tools and the measured-and-rejected options were removed, `ConvertOptions` went from 63 to 22 fields, `conversion.rs` was split into modules, and the code is formatted and clippy-clean. Default output is byte-identical to before the clean-up on all 120 development replays.
