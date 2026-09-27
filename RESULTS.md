# Reconstruction measurements

Last updated: 2026-09-27. These are development baselines for the current converter, not a final accuracy claim. The ignored machine-readable reports are `target/train-conversion-metrics.json` and `target/validation-conversion-metrics.json`. Each report includes every replay's SHA-256, dependencies, seed, settings, errors, and per-game-size aggregates. No `test` replay has been opened or converted.

## Protocol

`cargo run --release --bin evaluate_corpus -- replays/train target/train-conversion-metrics.json` measures the position immediately before a fresh replay packet corrects it. The evaluator also hides **all ball and car rigid-body fields** at frame offsets 1–4 in each 100-frame block, runs conversion again, and compares the resulting positions with fresh original replay positions. Only active-phase comparisons with a last observed position at most 0.5 seconds old are counted. Hold-last-position and constant-linear-velocity extrapolation share the same last unmasked observation. All values below are pooled absolute position errors in Rocket League unreal units (UU). This test measures short-horizon prediction at network frames; it does not verify every RocketSim field or long unobserved intervals. A regular mask schedule is deterministic but may correlate with periodic replay behavior; a later independent schedule should check it.

## Four-frame masked prediction

Each cell is median / p90 error in UU. The simulated and linear columns use the same observed target positions. Replays converted: 60/60 in each split; failures: zero.

| Split | Game size | Body | RocketSim | Linear | Samples |
| --- | --- | --- | ---: | ---: | ---: |
| train | 1v1 | ball | 12.2 / 41.4 | 14.4 / 51.9 | 1,635 |
| train | 1v1 | car | 18.7 / 50.5 | 30.1 / 69.5 | 1,534 |
| train | 2v2 | ball | 12.5 / 39.1 | 13.2 / 51.3 | 1,532 |
| train | 2v2 | car | 16.7 / 44.2 | 26.7 / 62.9 | 2,852 |
| train | 3v3 | ball | 14.7 / 43.2 | 16.5 / 57.9 | 1,930 |
| train | 3v3 | car | 18.0 / 46.6 | 30.3 / 64.2 | 5,663 |
| validation | 1v1 | ball | 10.6 / 38.4 | 12.1 / 46.4 | 1,720 |
| validation | 1v1 | car | 16.0 / 47.2 | 25.5 / 66.8 | 1,592 |
| validation | 2v2 | ball | 11.8 / 37.3 | 13.1 / 45.6 | 1,733 |
| validation | 2v2 | car | 16.1 / 41.4 | 26.5 / 62.9 | 3,201 |
| validation | 3v3 | ball | 14.2 / 43.9 | 15.3 / 56.0 | 1,945 |
| validation | 3v3 | car | 17.7 / 47.1 | 28.5 / 65.6 | 5,605 |

## One-step pre-correction prediction on train

| Game size | Body | RocketSim p50 / p90 / p99 | Linear p50 / p90 / p99 | Fresh positions |
| --- | --- | ---: | ---: | ---: |
| 1v1 | ball | 9.38 / 34.12 / 64.47 | 9.38 / 35.13 / 67.95 | 164,616 |
| 1v1 | car | 17.89 / 42.96 / 75.93 | 19.06 / 45.17 / 75.63 | 158,062 |
| 2v2 | ball | 11.20 / 33.73 / 63.68 | 10.87 / 34.77 / 69.13 | 151,225 |
| 2v2 | car | 16.12 / 42.07 / 73.23 | 17.36 / 43.79 / 69.75 | 285,355 |
| 3v3 | ball | 11.41 / 36.24 / 66.17 | 11.14 / 37.84 / 74.45 | 189,151 |
| 3v3 | car | 17.44 / 43.73 / 76.01 | 18.69 / 44.67 / 72.29 | 555,575 |

One-step simulated errors at p99 for cars are worse than linear extrapolation in 2v2 and 3v3. Collision, demolition, kickoff, hitbox, and unknown input cases need targeted investigation. The masked four-frame validation results are promising but do not establish accurate jump/flip, boost-pad, event, or scoreboard reconstruction.
