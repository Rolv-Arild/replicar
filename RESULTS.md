# Reconstruction measurements

Last updated: 2026-09-27. These are development baselines for the current converter, not a final accuracy claim. The ignored machine-readable reports are `target/train-conversion-metrics.json` and `target/validation-conversion-metrics.json`. Each report includes every replay's SHA-256, dependencies, seed, settings, errors, and per-game-size aggregates. No `test` replay has been opened or converted.

These tables include the actor-lifetime and inactive-owner fix, inferred boost input, active-pawn demolition correction, and primary-car selection when an old demolished actor overlaps a replacement. The no-boost ablation reports remain locally under `target/*-conversion-metrics-no-boost.json`; reproduce them with `--no-inferred-boost`. Earlier reports remain under `target/*-conversion-metrics-before-identity.json` and `target/*-conversion-metrics-before-demo-ghost-fix.json` where available.

## Protocol

`cargo run --release --bin evaluate_corpus -- replays/train target/train-conversion-metrics.json` measures the position immediately before a fresh replay packet corrects it. When multiple car actors share a player key, only the selected primary car is evaluated. The evaluator also hides **all ball and car rigid-body fields** at frame offsets 1–4 in each 100-frame block, runs conversion again, and compares the resulting positions with fresh original replay positions. Only active-phase comparisons with a last observed position at most 0.5 seconds old are counted. Hold-last-position and constant-linear-velocity extrapolation share the same last unmasked observation. All values below are pooled absolute position errors in Rocket League unreal units (UU). This test measures short-horizon prediction at network frames; it does not verify every RocketSim field or long unobserved intervals. A regular mask schedule is deterministic but may correlate with periodic replay behavior; a later independent schedule should check it.

## Four-frame masked prediction

Each cell is median / p90 error in UU. The simulated and linear columns use the same observed target positions. Replays converted: 60/60 in each split; failures: zero.

| Split | Game size | Body | RocketSim | Linear | Samples |
| --- | --- | --- | ---: | ---: | ---: |
| train | 1v1 | ball | 12.2 / 41.1 | 14.4 / 51.9 | 1,635 |
| train | 1v1 | car | 18.5 / 49.9 | 30.1 / 69.6 | 1,535 |
| train | 2v2 | ball | 12.4 / 39.1 | 13.2 / 51.3 | 1,532 |
| train | 2v2 | car | 16.4 / 40.7 | 26.7 / 63.0 | 2,853 |
| train | 3v3 | ball | 14.6 / 41.9 | 16.5 / 57.9 | 1,930 |
| train | 3v3 | car | 17.7 / 43.3 | 30.3 / 64.3 | 5,665 |
| validation | 1v1 | ball | 10.6 / 38.1 | 12.1 / 46.4 | 1,720 |
| validation | 1v1 | car | 15.4 / 43.9 | 25.5 / 66.8 | 1,592 |
| validation | 2v2 | ball | 11.8 / 37.2 | 13.1 / 45.6 | 1,733 |
| validation | 2v2 | car | 15.9 / 39.3 | 26.5 / 63.1 | 3,203 |
| validation | 3v3 | ball | 14.2 / 42.4 | 15.3 / 56.0 | 1,945 |
| validation | 3v3 | car | 17.2 / 45.3 | 28.5 / 65.6 | 5,606 |

## One-step pre-correction prediction on train

| Game size | Body | RocketSim p50 / p90 / p99 | Linear p50 / p90 / p99 | Fresh positions |
| --- | --- | ---: | ---: | ---: |
| 1v1 | ball | 9.38 / 34.12 / 64.43 | 9.38 / 35.13 / 67.95 | 164,616 |
| 1v1 | car | 17.83 / 42.29 / 69.38 | 19.06 / 45.16 / 75.59 | 158,055 |
| 2v2 | ball | 11.20 / 33.72 / 63.61 | 10.87 / 34.77 / 69.13 | 151,225 |
| 2v2 | car | 16.09 / 41.42 / 63.49 | 17.36 / 43.78 / 69.68 | 285,343 |
| 3v3 | ball | 11.41 / 36.24 / 66.14 | 11.14 / 37.84 / 74.45 | 189,151 |
| 3v3 | car | 17.31 / 42.65 / 69.77 | 18.69 / 44.66 / 72.23 | 555,544 |

One-step simulated car p99 errors are now below linear extrapolation in all three game sizes. The evaluator's top training car error over linear extrapolation fell from about 10,724 UU before demolition correction to 932 UU after selecting primary cars; the metric also excludes retired duplicate actors. Across train and validation, 32,751 overlapping or otherwise shadowed car-frame records were skipped, and active replay pawn evidence corrected 592 simulated demolition flags. Validation one-step car p99 fell from 76.68/73.38/76.39 UU to 66.69/62.52/72.39 UU across 1v1/2v2/3v3. Collision, kickoff, hitbox, and unknown input cases still need investigation. The masked results do not establish accurate jump/flip, boost-pad, event, or scoreboard reconstruction.

## Boost activation check

On `train`, 2,818 of 2,824 short observed boost-amount intervals ending in an odd-to-even boost activation counter transition showed boost depletion. On `validation`, 3,011 of 3,014 did. This supports interpreting odd counter values as active boost input. At the time of that ablation, enabling the signal improved four-frame validation car median/p90 from 16.0/46.8 to 15.4/45.7 UU in 1v1, 16.1/41.3 to 16.0/40.0 in 2v2, and 17.7/47.0 to 17.4/46.8 in 3v3. The initial small one-step p99 regression was overtaken by the later actor fixes above. The counter interpretation is an inference, not a direct action field.
