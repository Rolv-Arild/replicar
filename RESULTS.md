# Reconstruction measurements

Last updated: 2026-09-27. These are development baselines for the current converter, not a final accuracy claim. The ignored machine-readable reports are `target/train-conversion-metrics.json` and `target/validation-conversion-metrics.json`. Each report includes every replay's SHA-256, dependencies, seed, settings, errors, and per-game-size aggregates. No `test` replay has been opened or converted.

These tables include the actor-lifetime and inactive-owner fix, inferred boost input, active-pawn demolition correction, primary-car selection when an old demolished actor overlaps a replacement, loadout-derived hitboxes, and motion-gated jump input. The no-boost ablation reports remain locally under `target/*-conversion-metrics-no-boost.json`; reproduce them with `--no-inferred-boost`. The previous no-jump default reports are `target/*-conversion-metrics-before-gated-jump.json`; reproduce them with `--no-inferred-jump`. Earlier reports remain under `target/*-conversion-metrics-before-identity.json`, `target/*-conversion-metrics-before-demo-ghost-fix.json`, `target/*-conversion-metrics-before-hitboxes.json`, and `target/*-conversion-metrics-before-item-catalog.json` where available.

## Protocol

`cargo run --release --bin evaluate_corpus -- replays/train target/train-conversion-metrics.json` measures the position immediately before a fresh replay packet corrects it. When multiple car actors share a player key, only the selected primary car is evaluated. The evaluator also hides **all ball and car rigid-body fields, as well as fresh car boost amounts,** at frame offsets 1–4 in each 100-frame block, runs conversion again, and compares the resulting positions, kinematics, and boost amounts with fresh original replay positions. Replay boost activation evidence (`boost_active_raw`) remains available to the simulator during masked frames so boost consumption can be simulated. Only active-phase comparisons with a last observed value at most 0.5 seconds old are counted. Hold-last-value baselines share the same last unmasked observation. All position values below are pooled absolute position errors in Rocket League unreal units (UU). This test measures short-horizon prediction at network frames; it does not verify every RocketSim field or long unobserved intervals. An independent replay-specific mask schedule confirmed the direction of the car-position gains below.

## Four-frame masked prediction

Each cell is median / p90 error in UU. The simulated and linear columns use the same observed target positions. Replays converted: 60/60 in each split; failures: zero.

| Split | Game size | Body | RocketSim | Linear | Samples |
| --- | --- | --- | ---: | ---: | ---: |
| train | 1v1 | ball | 12.2 / 41.1 | 14.4 / 51.9 | 1,635 |
| train | 1v1 | car | 18.3 / 46.0 | 30.1 / 69.6 | 1,535 |
| train | 2v2 | ball | 12.4 / 39.1 | 13.2 / 51.3 | 1,532 |
| train | 2v2 | car | 15.9 / 39.8 | 26.7 / 63.0 | 2,853 |
| train | 3v3 | ball | 14.6 / 41.9 | 16.5 / 57.9 | 1,930 |
| train | 3v3 | car | 17.5 / 42.2 | 30.3 / 64.3 | 5,665 |
| validation | 1v1 | ball | 10.6 / 38.1 | 12.1 / 46.4 | 1,720 |
| validation | 1v1 | car | 15.0 / 42.6 | 25.5 / 66.8 | 1,592 |
| validation | 2v2 | ball | 11.8 / 37.2 | 13.1 / 45.6 | 1,733 |
| validation | 2v2 | car | 15.6 / 38.8 | 26.5 / 63.1 | 3,203 |
| validation | 3v3 | ball | 14.2 / 42.4 | 15.3 / 56.0 | 1,945 |
| validation | 3v3 | car | 17.0 / 44.1 | 28.5 / 65.6 | 5,606 |

## One-step pre-correction prediction on train

| Game size | Body | RocketSim p50 / p90 / p99 | Linear p50 / p90 / p99 | Fresh positions |
| --- | --- | ---: | ---: | ---: |
| 1v1 | ball | 9.38 / 34.12 / 64.43 | 9.38 / 35.13 / 67.95 | 164,616 |
| 1v1 | car | 17.86 / 42.33 / 69.43 | 19.06 / 45.16 / 75.59 | 158,055 |
| 2v2 | ball | 11.20 / 33.72 / 63.61 | 10.87 / 34.77 / 69.13 | 151,225 |
| 2v2 | car | 16.05 / 41.39 / 63.49 | 17.36 / 43.78 / 69.68 | 285,343 |
| 3v3 | ball | 11.41 / 36.24 / 66.14 | 11.14 / 37.84 / 74.45 | 189,151 |
| 3v3 | car | 17.25 / 42.61 / 69.74 | 18.69 / 44.66 / 72.23 | 555,544 |

One-step simulated car p99 errors are now below linear extrapolation in all three game sizes. The evaluator's top training car error over linear extrapolation fell from about 10,724 UU before demolition correction to 932 UU after selecting primary cars; the metric also excludes retired duplicate actors. Across train and validation, 32,751 overlapping or otherwise shadowed car-frame records were skipped, and active replay pawn evidence corrected 592 simulated demolition flags. Validation one-step car p99 fell from 76.68/73.38/76.39 UU to 66.69/62.52/72.39 UU across 1v1/2v2/3v3. Collision, kickoff, hitbox, and unknown input cases still need investigation. The masked results do not establish accurate jump/flip, boost-pad, event, or scoreboard reconstruction.

## Boost activation check

On `train`, 2,818 of 2,824 short observed boost-amount intervals ending in an odd-to-even boost activation counter transition showed boost depletion. On `validation`, 3,011 of 3,014 did. This supports interpreting odd counter values as active boost input. At the time of that ablation, enabling the signal improved four-frame validation car median/p90 from 16.0/46.8 to 15.4/45.7 UU in 1v1, 16.1/41.3 to 16.0/40.0 in 2v2, and 17.7/47.0 to 17.4/46.8 in 3v3. The initial small one-step p99 regression was overtaken by the later actor fixes above. The counter interpretation is an inference, not a direct action field.

## Four-frame masked boost prediction

The evaluator withholds fresh car boost amounts during the same four-frame rigid-body mask intervals, while leaving replay boost activation evidence (`boost_active_raw`) available to the simulator. RocketSim simulates continuous boost depletion during active boosting and boost-pad pickups if crossed. Predicted boost amounts (0–100 scale) are compared against fresh original replay boost values and a hold-last-observed-boost baseline. As with kinematics, samples require primary linked cars, active game state, and a last observation gap $\le 0.5$ seconds. All 60 train and 60 validation replays converted without failure.

### Boost error by mask horizon (all game sizes pooled)

Each entry is sample count and absolute boost error quantiles (p50 / p90 / p99) on the 0–100 boost scale.

| Split | Horizon | Samples | RocketSim p50 / p90 / p99 | Hold p50 / p90 / p99 |
| --- | ---: | ---: | ---: | ---: |
| train | 1 | 220 | 0.59 / 12.16 / 88.95 | 7.84 / 14.12 / 95.69 |
| train | 2 | 209 | 0.92 / 12.48 / 100.00 | 8.63 / 16.47 / 100.00 |
| train | 3 | 214 | 0.52 / 12.16 / 96.08 | 7.84 / 17.65 / 100.00 |
| train | 4 | 202 | 0.65 / 12.75 / 97.65 | 8.24 / 22.75 / 100.00 |
| validation | 1 | 251 | 0.72 / 12.65 / 100.00 | 6.67 / 32.55 / 100.00 |
| validation | 2 | 211 | 0.49 / 12.21 / 100.00 | 8.24 / 20.39 / 100.00 |
| validation | 3 | 253 | 0.85 / 12.49 / 100.00 | 9.02 / 28.63 / 100.00 |
| validation | 4 | 197 | 0.47 / 12.21 / 100.00 | 7.45 / 15.69 / 100.00 |

### Boost error by game size at horizon 4

| Split | Game size | Samples | RocketSim p50 / p90 / p99 | Hold p50 / p90 / p99 |
| --- | --- | ---: | ---: | ---: |
| train | 1v1 | 39 | 0.57 / 17.25 / 97.65 | 11.37 / 17.25 / 97.65 |
| train | 2v2 | 47 | 0.64 / 12.46 / 76.86 | 8.24 / 12.16 / 92.55 |
| train | 3v3 | 116 | 0.76 / 12.26 / 100.00 | 7.45 / 44.31 / 100.00 |
| validation | 1v1 | 36 | 1.06 / 13.18 / 100.00 | 9.02 / 74.51 / 100.00 |
| validation | 2v2 | 71 | 0.33 / 12.16 / 100.00 | 6.67 / 12.94 / 100.00 |
| validation | 3v3 | 90 | 0.49 / 12.21 / 85.10 | 7.06 / 16.86 / 100.00 |

### Alternate mask schedule (`--mask-seed 239847`) on validation

An independent check using the deterministic pseudo-random offset schedule confirms the boost metrics:

| Horizon | Samples | RocketSim p50 / p90 / p99 | Hold p50 / p90 / p99 |
| ---: | ---: | ---: | ---: |
| 1 | 191 | 0.46 / 12.16 / 27.83 | 8.63 / 17.25 / 100.00 |
| 2 | 223 | 0.49 / 12.16 / 100.00 | 7.45 / 14.51 / 100.00 |
| 3 | 223 | 0.47 / 12.16 / 100.00 | 7.84 / 17.65 / 100.00 |
| 4 | 189 | 0.64 / 12.16 / 83.92 | 8.63 / 15.29 / 100.00 |

At horizon 4 by game size with `--mask-seed 239847`: 1v1 (43 samples) RocketSim 0.64 / 12.16 / 100.00 vs Hold 8.63 / 33.33 / 100.00; 2v2 (58 samples) RocketSim 0.59 / 12.16 / 44.90 vs Hold 7.06 / 12.55 / 41.57; 3v3 (88 samples) RocketSim 0.64 / 12.16 / 21.45 vs Hold 10.20 / 15.29 / 100.00.

### Boost findings and limitations

1. **Continuous depletion tracking (p50):** RocketSim median boost error is under 1.0 boost unit across all horizons and game sizes (0.33 to 1.06 boost units, representing $\le 1\%$ of total boost capacity). In contrast, the hold-last-observed baseline median error is 6.3 to 11.4 boost units. This confirms that simulating boost depletion from inferred active boost input (`boost_active_raw`) closely matches ground-truth consumption.
2. **Small boost-pad pickups (p90):** Across nearly every horizon and game-size slice, RocketSim p90 error clusters tightly around **12.16 boost units**. In Rocket League, small boost pads provide exactly 12% boost (or 31 raw ticks, $31 \times 100 / 255 \approx 12.156863$ boost units). This reveals that the predominant discrepancy at p90 is small pad pickups occurring during the four-frame mask that the simulator does not reproduce—either because masked car body trajectory deviated from the pickup radius, or because pad cooldown state in RocketSim was not synchronized with the match.
3. **Orb pickups and respawns (p99):** Extreme tail errors reach 80–100 boost units, corresponding to 100-boost orb pickups or kickoff/respawn refills that occurred during the withheld window.
4. **Limitations:** Only frames with fresh original boost packets are evaluated (~200–250 per horizon per split); replay boost updates are replicated at network rates rather than 120 Hz ticks. Simultaneous body masking also couples car position error to boost-pad collision detection.

## Four-frame masked kinematics

The same mask now measures fresh linear velocity (UU/s), rotation angle (degrees), and angular velocity (radians/s). Each field uses its own last unmasked observation and is counted only when that field is fresh in the original frame and its observation gap is at most 0.5 seconds in active play. The comparison baseline holds that field's last value. Replay angular velocity is scaled by 0.01 before comparison. The primary-car filter applies; sample counts can differ because replay rigid-body velocity fields are optional.

Validation results at mask horizon 4 are below. Each value is median / p90 error. All 60 validation replays converted without failure. The train report also shows lower RocketSim median and p90 than hold for every listed field and game size.

| Size | Body | Field | RocketSim | Hold | Samples |
| --- | --- | --- | ---: | ---: | ---: |
| 1v1 | ball | Velocity (UU/s) | 5.46 / 21.77 | 86.99 / 347.79 | 1,720 |
| 1v1 | ball | Rotation (degrees) | 2.86 / 5.78 | 40.11 / 51.57 | 1,720 |
| 1v1 | ball | Angular velocity (rad/s) | 0.00 / 0.07 | 0.00 / 1.14 | 1,720 |
| 1v1 | car | Velocity (UU/s) | 51.85 / 238.31 | 180.27 / 648.79 | 1,592 |
| 1v1 | car | Rotation (degrees) | 4.50 / 29.06 | 20.32 / 59.16 | 1,592 |
| 1v1 | car | Angular velocity (rad/s) | 0.97 / 3.64 | 1.28 / 4.19 | 1,592 |
| 2v2 | ball | Velocity (UU/s) | 5.44 / 21.65 | 86.79 / 527.24 | 1,733 |
| 2v2 | ball | Rotation (degrees) | 2.86 / 5.73 | 40.11 / 51.57 | 1,733 |
| 2v2 | ball | Angular velocity (rad/s) | 0.00 / 0.07 | 0.00 / 1.65 | 1,733 |
| 2v2 | car | Velocity (UU/s) | 43.28 / 203.60 | 198.85 / 628.28 | 3,201 |
| 2v2 | car | Rotation (degrees) | 3.20 / 25.68 | 17.53 / 53.98 | 3,203 |
| 2v2 | car | Angular velocity (rad/s) | 0.61 / 3.34 | 1.09 / 3.59 | 3,201 |
| 3v3 | ball | Velocity (UU/s) | 5.46 / 25.90 | 88.73 / 987.46 | 1,945 |
| 3v3 | ball | Rotation (degrees) | 2.86 / 6.81 | 43.31 / 51.57 | 1,945 |
| 3v3 | ball | Angular velocity (rad/s) | 0.00 / 0.07 | 0.00 / 2.70 | 1,945 |
| 3v3 | car | Velocity (UU/s) | 39.68 / 188.16 | 209.35 / 641.28 | 5,605 |
| 3v3 | car | Rotation (degrees) | 3.04 / 22.94 | 17.53 / 52.84 | 5,606 |
| 3v3 | car | Angular velocity (rad/s) | 0.54 / 3.22 | 1.09 / 3.52 | 5,605 |

Car angular velocity has the smallest gain, especially near p90. Missing jump and aerial controls and hitbox mismatch are plausible contributors, but the current report does not isolate them. Ball angular velocity median is zero in both systems because many sampled intervals contain no change; its p90 is more informative. This is a short-horizon comparison with replay packets, not a guarantee that all unobserved state is correct.

## Rotation calibration and independent mask

The masked metrics in this section are the earlier no-jump baseline; the motion-gated results appear below.

`cargo run --release --bin calibrate_rotation -- replays/train` compared fresh car angular-velocity packets against the quaternion change over short active-play intervals. On 367,746 ground intervals, the world-frame interpretation had median/p90 vector error 0.31/1.14 rad/s, versus 0.33/1.86 when rotated from car-local coordinates and 3.75/6.06 when negated. On 262,200 air intervals, the corresponding errors were 0.62/2.64, 4.20/9.76, and 7.26/12.49 rad/s. The median direction alignment of world-frame angular velocity with quaternion motion was approximately 1.00 in both groups. This confirms the existing 0.01 scale and world-coordinate mapping; the remaining masked error is unlikely to come from a coordinate transform. The comparison uses two packet endpoints, so it also includes real within-interval acceleration.

`evaluate_corpus --mask-seed 239847` uses each replay's SHA-256 and a fixed seed to select one four-frame gap at a different deterministic offset in every 100-frame block. This is an independent check on the original fixed offsets 1–4. Reports are `target/train-conversion-metrics-alt-mask.json` and `target/validation-conversion-metrics-alt-mask.json`. All 60 replays converted in each split with zero failures; the `test` split remains sealed. At horizon 4, car position RocketSim median/p90 versus constant-velocity extrapolation (UU) was:

| Split | Size | RocketSim | Linear |
| --- | --- | ---: | ---: |
| train | 1v1 | 17.3 / 47.4 | 27.5 / 66.8 |
| train | 2v2 | 16.2 / 39.8 | 26.4 / 62.9 |
| train | 3v3 | 18.1 / 47.3 | 30.2 / 65.8 |
| validation | 1v1 | 15.9 / 43.7 | 24.7 / 61.7 |
| validation | 2v2 | 15.9 / 41.2 | 26.3 / 64.2 |
| validation | 3v3 | 17.0 / 46.6 | 28.6 / 64.1 |

The alternate mask also groups fresh car angular-velocity targets by observed height. Values below are median/p90 absolute vector errors in rad/s, pooled across game sizes and mask horizons. `ground` means car center below 50 UU; `air` means above 100 UU; `transition` is between those thresholds. These are height groups, not verified contact states.

| Split | Height | Samples | RocketSim | Hold |
| --- | --- | ---: | ---: | ---: |
| train | ground | 21,914 | 0.20 / 1.17 | 0.53 / 2.21 |
| train | transition | 5,631 | 1.95 / 4.61 | 0.95 / 3.96 |
| train | air | 12,080 | 1.30 / 3.31 | 1.15 / 3.31 |
| validation | ground | 23,638 | 0.20 / 1.17 | 0.52 / 2.23 |
| validation | transition | 5,962 | 1.84 / 4.67 | 0.94 / 4.01 |
| validation | air | 12,599 | 1.25 / 3.25 | 1.13 / 3.36 |

Ground steering accounts for the aggregate angular-velocity gain. Near takeoff the simulator is worse than the held baseline, and airborne median error is also higher. Missing dodge and aerial controls remain concrete hypotheses to test. A read-only train replay audit found that `ReplicatedActive` jump and dodge bytes increment across adjacent frames and dodge updates can coincide with `DodgeTorque`; they are activation evidence, not a demonstrated held-button state. Exact input timing and duration remain unknown; the later motion-gated rule below uses the jump counter only when its impulse is not already observed.

## Action-event timing and earlier raw jump-input ablation

`calibrate_action_events` traces primary linked cars in active play and compares each fresh jump, double-jump, or dodge counter change with the nearest fresh body packets strictly before and after it, each within 0.15 seconds. It preserves `DodgeTorque` as a raw, provenance-tagged vector. The event association does not isolate collisions or prove exact controller timing. The 60 training replays show:

| Transition | Events | Paired body intervals | Median vertical-velocity change (UU/s) | Fresh dodge torque |
| --- | ---: | ---: | ---: | ---: |
| Jump to odd | 17,614 | 17,559 | +302 | 63 |
| Jump to even | 19,525 | 19,461 | -5 | 393 |
| Double-jump to odd | 1,725 | 1,723 | +217 | 6 |
| Double-jump to even | 2,849 | 2,845 | -42 | 10 |
| Dodge to odd | 14,094 | 14,040 | -58 | 13,914 |
| Dodge to even | 15,561 | 15,520 | -31 | 128 |

The same pattern appears on validation: jump-to-odd median vertical-velocity change is +304 UU/s versus -6 for jump-to-even; 14,431 of 14,730 dodge-to-odd transitions carry fresh torque. Nearly all counter changes are a +1 increment. Odd transitions therefore give strong evidence of a jump or dodge activation, and even transitions usually mark its end. The packet timestamp can still lag the physical input, and the replicated duration may differ from the actual button hold.

An opt-in `--inferred-jump` ablation sets RocketSim's jump control while the jump counter is odd. It was evaluated against the earlier no-jump default with the same seed-239847 alternate mask; 60/60 train and 60/60 validation replays converted. Validation car-position median/p90 error (UU) at mask horizon 4 improved modestly, but one-step pre-correction error increased in every game size:

| Size | Four-frame default | Four-frame inferred jump | One-step default | One-step inferred jump |
| --- | ---: | ---: | ---: | ---: |
| 1v1 | 15.86 / 43.70 | 15.87 / 43.44 | 16.36 / 38.85 | 16.54 / 39.10 |
| 2v2 | 15.90 / 41.23 | 15.34 / 40.53 | 15.96 / 41.46 | 16.03 / 41.63 |
| 3v3 | 16.98 / 46.65 | 16.55 / 46.47 | 16.87 / 42.05 | 16.91 / 42.19 |

Training showed the same direction: four-frame medians improved by 0.27–0.47 UU while one-step medians worsened by 0.02–0.15 UU. The raw-counter mode remains opt-in; the motion-gated mode is now the default. That ungated input reaches RocketSim after the replay packet; if the observed body already contains the jump impulse, the simulator may apply it late. The follow-up motion-gated rule is measured below. The alternate-mask reports are `target/*-conversion-metrics-inferred-jump.json`; use `--no-inferred-jump` for the no-jump baseline.

## Motion-gated jump timing

The jump-packet timing audit on `train` found 10,622 odd jump transitions with a fresh rigid-body packet in the same frame. The median vertical-velocity change was +295 UU/s **before** that packet and +20.5 UU/s after it; 6,922 of those same-frame packets already reported upward velocity above 200 UU/s. This supports the inference that applying a new RocketSim jump impulse after every odd counter packet often applies it too late.

The motion-gated rule starts inferred jump input only when the most recently observed car center is below 50 UU and a fresh velocity packet does not already show upward speed above 150 UU/s. That decision stays with the activation until the next counter update. It uses only observations available at that frame, so a hidden rigid-body packet can still leave a jump event available for prediction. The thresholds were selected from `train` timing evidence and then checked unchanged on `validation`. The rule estimates a control; it does not claim the original button timing or duration is known.

All 60 `train` and 60 `validation` replays converted under both the fixed and seed-239847 masks, with zero failures. Validation car-position error (UU) at four-frame horizon and linear-velocity p90 (UU/s) improved in every size. The old columns use `--no-inferred-jump`; the new columns use the motion-gated default.

| Size | Fixed-mask position p50/p90 old → new | Fixed-mask velocity p90 old → new | Alternate-mask position p90 old → new | One-step position p50 old → new |
| --- | ---: | ---: | ---: | ---: |
| 1v1 | 15.36/43.87 → 15.03/42.63 | 287.07 → 238.31 | 43.70 → 42.01 | 16.36 → 16.40 |
| 2v2 | 15.90/39.31 → 15.61/38.75 | 242.76 → 203.60 | 41.23 → 39.77 | 15.96 → 15.92 |
| 3v3 | 17.23/45.24 → 17.03/44.10 | 219.55 → 188.16 | 46.65 → 45.30 | 16.87 → 16.81 |

One-step p90 and p99 are effectively stable; 1v1 one-step median rises by 0.04 UU. Fixed-mask train car-position p90 also improves from 49.89/40.58/43.28 to 45.95/39.77/42.20 UU across 1v1/2v2/3v3. The motion-gated mode is enabled by default. `--no-inferred-jump` reproduces the earlier no-jump baseline, and `--inferred-jump` selects the earlier ungated odd-counter experiment. The new alternate-mask reports are `target/*-conversion-metrics-gated-jump.json`; fixed-mask reports are `target/*-conversion-metrics.json`. The `test` split remains sealed.

## Loadout body products and hitboxes

`TAGame.PRI_TA:ClientLoadouts` supplies a body product ID for each team. The extractor preserves both values on each player and attaches the currently selected one to a linked car. The user's August 2026 `items.csv` supplies product names; [Rocket League's official hitbox list](https://www.epicgames.com/help/c-37599050/c-32343914/a20257614?lang=en-US), additional [Season 22](https://www.rocketleague.com/news/rocket-league-season-22-training-rivalries-and-rewards) and [Season 23](https://www.rocketleague.com/news/hit-the-pitch-for-the-world-cup-in-rocket-league-season-23) announcements, and a localized official listing supply families. RocketSim's dedicated Psyclops preset covers that special body. The checked-in inputs, aliases, source URLs, generator, and unresolved rows are documented in [data/README.md](data/README.md). RocketSim's matching preset is used at car-slot creation.

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
| 7979 | Stampede | Merc | 0 |

The generated map covers 213 of 238 `Body` product rows; 11 of the 25 unresolved rows are generic drop or mystery labels, and 14 are named vehicles without a verified assignment in the checked sources. Unknown IDs remain raw and use an Octane fallback if selected by a playing car. The two rare training PRI products 7477 and 7979 were attached to non-playing actors; 7979 is now identified as Stampede, which the official list puts in Merc. All 240 playing train slots and all 240 playing validation slots have mapped IDs. Validation now has 237 Octane, two Plank, and one Hybrid slot. Its four formerly unresolved playing IDs are 25 Road Hog (Octane), 1691 Mantis (Plank), 1919 Centio (Plank), and 10900 Shokunin (Octane).

Compared with the previous eight-ID map, full-corpus train position metrics are unchanged. Validation gains two Plank slots. Four-frame car position median/p90 is unchanged in 1v1 and 2v2 to three decimals; 3v3 moves from 17.221/45.252 to 17.233/45.235 UU. One-step p90 in the Mantis replay improves by 0.008 UU, while the Centio replay worsens by 0.136 UU; other percentiles move in both directions. This is a state-fidelity correction, and the small sample does not establish a broad prediction gain. `--octane-hitbox` reproduces the original all-Octane setup.
