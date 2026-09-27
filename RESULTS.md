# Reconstruction measurements

Last updated: 2026-09-27. These are development baselines for the current converter, not a final accuracy claim. The ignored machine-readable reports are `target/train-conversion-metrics.json` and `target/validation-conversion-metrics.json`. Each report includes every replay's SHA-256, dependencies, seed, settings, errors, and per-game-size aggregates. No `test` replay has been opened or converted.

These tables include the actor-lifetime and inactive-owner fix plus inferred boost input from the boost-component activation counter. The no-boost ablation reports remain locally under `target/*-conversion-metrics-no-boost.json`; reproduce them with `--no-inferred-boost`. The earlier identity baselines remain under `target/*-conversion-metrics-before-identity.json`.

## Protocol

`cargo run --release --bin evaluate_corpus -- replays/train target/train-conversion-metrics.json` measures the position immediately before a fresh replay packet corrects it. The evaluator also hides **all ball and car rigid-body fields** at frame offsets 1–4 in each 100-frame block, runs conversion again, and compares the resulting positions with fresh original replay positions. Only active-phase comparisons with a last observed position at most 0.5 seconds old are counted. Hold-last-position and constant-linear-velocity extrapolation share the same last unmasked observation. All values below are pooled absolute position errors in Rocket League unreal units (UU). This test measures short-horizon prediction at network frames; it does not verify every RocketSim field or long unobserved intervals. A regular mask schedule is deterministic but may correlate with periodic replay behavior; a later independent schedule should check it.

## Four-frame masked prediction

Each cell is median / p90 error in UU. The simulated and linear columns use the same observed target positions. Replays converted: 60/60 in each split; failures: zero.

| Split | Game size | Body | RocketSim | Linear | Samples |
| --- | --- | --- | ---: | ---: | ---: |
| train | 1v1 | ball | 12.2 / 41.1 | 14.4 / 51.9 | 1,635 |
| train | 1v1 | car | 18.6 / 50.4 | 30.1 / 69.6 | 1,535 |
| train | 2v2 | ball | 12.4 / 39.1 | 13.2 / 51.3 | 1,532 |
| train | 2v2 | car | 16.6 / 41.2 | 26.7 / 63.0 | 2,853 |
| train | 3v3 | ball | 14.6 / 41.9 | 16.5 / 57.9 | 1,930 |
| train | 3v3 | car | 17.9 / 44.9 | 30.3 / 64.3 | 5,665 |
| validation | 1v1 | ball | 10.6 / 38.1 | 12.1 / 46.4 | 1,720 |
| validation | 1v1 | car | 15.4 / 45.7 | 25.5 / 66.8 | 1,592 |
| validation | 2v2 | ball | 11.8 / 37.2 | 13.1 / 45.6 | 1,733 |
| validation | 2v2 | car | 16.0 / 40.0 | 26.5 / 63.1 | 3,203 |
| validation | 3v3 | ball | 14.2 / 42.4 | 15.3 / 56.0 | 1,945 |
| validation | 3v3 | car | 17.4 / 46.8 | 28.5 / 65.6 | 5,606 |

## One-step pre-correction prediction on train

| Game size | Body | RocketSim p50 / p90 / p99 | Linear p50 / p90 / p99 | Fresh positions |
| --- | --- | ---: | ---: | ---: |
| 1v1 | ball | 9.38 / 34.12 / 64.43 | 9.38 / 35.13 / 67.95 | 164,616 |
| 1v1 | car | 17.87 / 43.16 / 75.70 | 19.06 / 45.17 / 75.63 | 158,062 |
| 2v2 | ball | 11.20 / 33.72 / 63.64 | 10.87 / 34.77 / 69.13 | 151,225 |
| 2v2 | car | 16.13 / 42.16 / 72.56 | 17.36 / 43.79 / 69.75 | 285,355 |
| 3v3 | ball | 11.41 / 36.24 / 66.14 | 11.14 / 37.84 / 74.45 | 189,151 |
| 3v3 | car | 17.40 / 43.93 / 76.21 | 18.69 / 44.67 / 72.29 | 555,575 |

One-step simulated errors at p99 for cars are worse than linear extrapolation in all three game sizes. Collision, demolition, kickoff, hitbox, and unknown input cases need targeted investigation. The masked four-frame validation results are promising but do not establish accurate jump/flip, boost-pad, event, or scoreboard reconstruction.

## Boost activation check

On `train`, 2,818 of 2,824 short observed boost-amount intervals ending in an odd-to-even boost activation counter transition showed boost depletion. On `validation`, 3,011 of 3,014 did. This supports interpreting odd counter values as active boost input. Enabling that input improved four-frame validation car median/p90 from 16.0/46.8 to 15.4/45.7 UU in 1v1, 16.1/41.3 to 16.0/40.0 in 2v2, and 17.7/47.0 to 17.4/46.8 in 3v3. One-step car p99 increased slightly in all sizes; this remains an outlier investigation target. The counter interpretation is an inference, not a direct action field.
