# Reconstruction measurements

Last updated: 2026-09-27. These are development baselines for the current converter, not a final accuracy claim. The ignored machine-readable reports are `target/train-conversion-metrics.json` and `target/validation-conversion-metrics.json`. Each report includes every replay's SHA-256, dependencies, seed, settings, errors, and per-game-size aggregates. No `test` replay has been opened or converted.

These tables include the actor-lifetime and inactive-owner fix, inferred boost input, active-pawn demolition correction, primary-car selection when an old demolished actor overlaps a replacement, and loadout-derived hitboxes. The no-boost ablation reports remain locally under `target/*-conversion-metrics-no-boost.json`; reproduce them with `--no-inferred-boost`. Earlier reports remain under `target/*-conversion-metrics-before-identity.json`, `target/*-conversion-metrics-before-demo-ghost-fix.json`, and `target/*-conversion-metrics-before-hitboxes.json` where available.

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
| 1v1 | car | 17.83 / 42.29 / 69.37 | 19.06 / 45.16 / 75.59 | 158,055 |
| 2v2 | ball | 11.20 / 33.72 / 63.61 | 10.87 / 34.77 / 69.13 | 151,225 |
| 2v2 | car | 16.09 / 41.42 / 63.49 | 17.36 / 43.78 / 69.68 | 285,343 |
| 3v3 | ball | 11.41 / 36.24 / 66.14 | 11.14 / 37.84 / 74.45 | 189,151 |
| 3v3 | car | 17.31 / 42.65 / 69.77 | 18.69 / 44.66 / 72.23 | 555,544 |

One-step simulated car p99 errors are now below linear extrapolation in all three game sizes. The evaluator's top training car error over linear extrapolation fell from about 10,724 UU before demolition correction to 932 UU after selecting primary cars; the metric also excludes retired duplicate actors. Across train and validation, 32,751 overlapping or otherwise shadowed car-frame records were skipped, and active replay pawn evidence corrected 592 simulated demolition flags. Validation one-step car p99 fell from 76.68/73.38/76.39 UU to 66.69/62.52/72.39 UU across 1v1/2v2/3v3. Collision, kickoff, hitbox, and unknown input cases still need investigation. The masked results do not establish accurate jump/flip, boost-pad, event, or scoreboard reconstruction.

## Boost activation check

On `train`, 2,818 of 2,824 short observed boost-amount intervals ending in an odd-to-even boost activation counter transition showed boost depletion. On `validation`, 3,011 of 3,014 did. This supports interpreting odd counter values as active boost input. At the time of that ablation, enabling the signal improved four-frame validation car median/p90 from 16.0/46.8 to 15.4/45.7 UU in 1v1, 16.1/41.3 to 16.0/40.0 in 2v2, and 17.7/47.0 to 17.4/46.8 in 3v3. The initial small one-step p99 regression was overtaken by the later actor fixes above. The counter interpretation is an inference, not a direct action field.

## Four-frame masked kinematics

The same mask now measures fresh linear velocity (UU/s), rotation angle (degrees), and angular velocity (radians/s). Each field uses its own last unmasked observation and is counted only when that field is fresh in the original frame and its observation gap is at most 0.5 seconds in active play. The comparison baseline holds that field's last value. Replay angular velocity is scaled by 0.01 before comparison. The primary-car filter applies; sample counts can differ because replay rigid-body velocity fields are optional.

Validation results at mask horizon 4 are below. Each value is median / p90 error. All 60 validation replays converted without failure. The train report also shows lower RocketSim median and p90 than hold for every listed field and game size.

| Size | Body | Field | RocketSim | Hold | Samples |
| --- | --- | --- | ---: | ---: | ---: |
| 1v1 | ball | Velocity (UU/s) | 5.46 / 21.77 | 86.99 / 347.79 | 1,720 |
| 1v1 | ball | Rotation (degrees) | 2.86 / 5.78 | 40.11 / 51.57 | 1,720 |
| 1v1 | ball | Angular velocity (rad/s) | 0.00 / 0.07 | 0.00 / 1.14 | 1,720 |
| 1v1 | car | Velocity (UU/s) | 53.22 / 287.07 | 180.27 / 648.79 | 1,592 |
| 1v1 | car | Rotation (degrees) | 4.51 / 29.06 | 20.32 / 59.16 | 1,592 |
| 1v1 | car | Angular velocity (rad/s) | 0.99 / 3.62 | 1.28 / 4.19 | 1,592 |
| 2v2 | ball | Velocity (UU/s) | 5.44 / 21.65 | 86.79 / 527.24 | 1,733 |
| 2v2 | ball | Rotation (degrees) | 2.86 / 5.73 | 40.11 / 51.57 | 1,733 |
| 2v2 | ball | Angular velocity (rad/s) | 0.00 / 0.07 | 0.00 / 1.65 | 1,733 |
| 2v2 | car | Velocity (UU/s) | 45.78 / 242.76 | 198.85 / 628.28 | 3,201 |
| 2v2 | car | Rotation (degrees) | 3.19 / 25.68 | 17.53 / 53.98 | 3,203 |
| 2v2 | car | Angular velocity (rad/s) | 0.61 / 3.34 | 1.09 / 3.59 | 3,201 |
| 3v3 | ball | Velocity (UU/s) | 5.46 / 25.90 | 88.73 / 987.46 | 1,945 |
| 3v3 | ball | Rotation (degrees) | 2.86 / 6.81 | 43.31 / 51.57 | 1,945 |
| 3v3 | ball | Angular velocity (rad/s) | 0.00 / 0.07 | 0.00 / 2.70 | 1,945 |
| 3v3 | car | Velocity (UU/s) | 41.28 / 219.47 | 209.35 / 641.28 | 5,605 |
| 3v3 | car | Rotation (degrees) | 3.04 / 22.94 | 17.53 / 52.84 | 5,606 |
| 3v3 | car | Angular velocity (rad/s) | 0.54 / 3.22 | 1.09 / 3.52 | 5,605 |

Car angular velocity has the smallest gain, especially near p90. Missing jump and aerial controls and hitbox mismatch are plausible contributors, but the current report does not isolate them. Ball angular velocity median is zero in both systems because many sampled intervals contain no change; its p90 is more informative. This is a short-horizon comparison with replay packets, not a guarantee that all unobserved state is correct.

## Loadout body products and hitboxes

`TAGame.PRI_TA:ClientLoadouts` supplies a body product ID for each team. The extractor preserves both values on each player and attaches the currently selected one to a linked car. IDs were matched to names using the [game-extracted product catalog](https://raw.githubusercontent.com/rocketleagueapi/items/main/src/parsed/products.json), then to hitbox families using [Rocket League's official hitbox list](https://www.epicgames.com/help/c-37599050/c-Trending_0/snadyq-isabh-hitboxes-syarat-rocket-league-a20257614). RocketSim's matching preset is used at car-slot creation.

| Product ID | Body | Hitbox | Playing slots in train |
| ---: | --- | --- | ---: |
| 21 | Backfire | Octane | 1 |
| 22 | Breakout | Breakout | 1 |
| 23 | Octane | Octane | 53 |
| 26 | Gizmo | Octane | 1 |
| 403 | Dominus | Dominus | 5 |
| 4284 | Fennec | Octane | 178 |
| 7012 | Tesla Cybertruck | Hybrid | 1 |
| 7477 | Nomad GXT | Merc | 0 |

Training had 240 playing car slots, all with a mapped body product ID. Product 7477 appeared on a non-playing PRI. One additional raw ID, 7979, appeared on another non-playing PRI and is unresolved in the checked catalog; it remains in observations and falls back to Octane if it is ever used by a playing car. Validation had 240 playing slots: 239 Octane and one Hybrid. Four Octane slots used unresolved products (1919, 10900, 25, 1691); these were left as fallbacks rather than mapped from validation examples.

The mapping changed only seven playing train slots and one validation slot. Train one-step car p90 improved slightly in three of the seven affected replays, was unchanged to two decimal places in three, and worsened by 0.02 UU in one. Aggregate validation four-frame car position median/p90 is unchanged at the precision shown above; validation 2v2 car velocity median moved from 45.40 to 45.78 UU/s. This is primarily a state-fidelity correction; the current sparse non-Octane sample does not establish a broad prediction gain. `--octane-hitbox` reproduces the original setup.
