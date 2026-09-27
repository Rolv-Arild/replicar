# Reconstruction measurements

Last updated: 2026-09-27. These are development baselines for the current converter, not a final accuracy claim. The ignored machine-readable reports are `target/train-conversion-metrics.json` and `target/validation-conversion-metrics.json`. Each report includes every replay's SHA-256, dependencies, seed, settings, errors, and per-game-size aggregates. No `test` replay has been opened or converted.

These tables include the actor-lifetime and inactive-owner fix, inferred boost input, active-pawn demolition correction, primary-car selection when an old demolished actor overlaps a replacement, loadout-derived hitboxes, motion-gated jump input, and motion-gated dodge flip inference. The no-dodge ablation reports remain locally under `target/*-conversion-metrics-gated-jump.json` / `target/*-conversion-metrics-gated-jump-fixed-mask.json`; reproduce them with `--no-inferred-dodge`. Raw ungated dodge reports are `target/*-inferred-dodge-metrics.json`; reproduce them with `--inferred-dodge`. The no-boost ablation reports remain locally under `target/*-conversion-metrics-no-boost.json`; reproduce them with `--no-inferred-boost`. The previous no-jump default reports are `target/*-conversion-metrics-before-gated-jump.json`; reproduce them with `--no-inferred-jump`. Earlier reports remain under `target/*-conversion-metrics-before-identity.json`, `target/*-conversion-metrics-before-demo-ghost-fix.json`, `target/*-conversion-metrics-before-hitboxes.json`, and `target/*-conversion-metrics-before-item-catalog.json` where available.

## Protocol

`cargo run --release --bin evaluate_corpus -- replays/train target/train-conversion-metrics.json` measures the position immediately before a fresh replay packet corrects it. When multiple car actors share a player key, only the selected primary car is evaluated. The evaluator also hides **all ball and car rigid-body fields, as well as fresh car boost amounts,** at frame offsets 1–4 in each 100-frame block, runs conversion again, and compares the resulting positions, kinematics, and boost amounts with fresh original replay positions. Replay boost activation evidence (`boost_active_raw`) remains available to the simulator during masked frames so boost consumption can be simulated. Only active-phase comparisons with a last observed value at most 0.5 seconds old are counted. Hold-last-value baselines share the same last unmasked observation. All position values below are pooled absolute position errors in Rocket League unreal units (UU). This test measures short-horizon prediction at network frames; it does not verify every RocketSim field or long unobserved intervals. An independent replay-specific mask schedule confirmed the direction of the car-position gains below.

## Four-frame masked prediction

Each cell is median / p90 error in UU at mask horizon 4. The simulated and linear columns use the same observed target positions. Replays converted: 60/60 in each split; failures: zero.

| Split | Game size | Body | RocketSim | Linear | Samples |
| --- | --- | --- | ---: | ---: | ---: |
| train | 1v1 | ball | 12.2 / 40.5 | 14.4 / 51.9 | 1,635 |
| train | 1v1 | car | 17.5 / 44.3 | 30.1 / 69.6 | 1,535 |
| train | 2v2 | ball | 12.3 / 38.7 | 13.2 / 51.3 | 1,532 |
| train | 2v2 | car | 15.7 / 38.7 | 26.7 / 63.0 | 2,853 |
| train | 3v3 | ball | 14.6 / 41.7 | 16.5 / 57.9 | 1,930 |
| train | 3v3 | car | 17.1 / 40.4 | 30.3 / 64.3 | 5,665 |
| validation | 1v1 | ball | 10.6 / 38.1 | 12.1 / 46.4 | 1,720 |
| validation | 1v1 | car | 14.8 / 41.6 | 25.5 / 66.8 | 1,592 |
| validation | 2v2 | ball | 11.7 / 37.0 | 13.1 / 45.6 | 1,733 |
| validation | 2v2 | car | 15.3 / 38.3 | 26.5 / 63.1 | 3,203 |
| validation | 3v3 | ball | 14.2 / 41.7 | 15.3 / 56.0 | 1,945 |
| validation | 3v3 | car | 16.5 / 41.2 | 28.5 / 65.6 | 5,606 |

## One-step pre-correction prediction on train

| Game size | Body | RocketSim p50 / p90 / p99 | Linear p50 / p90 / p99 | Fresh positions |
| --- | --- | ---: | ---: | ---: |
| 1v1 | ball | 9.38 / 34.13 / 64.37 | 9.38 / 35.13 / 67.95 | 164,616 |
| 1v1 | car | 17.85 / 42.29 / 69.33 | 19.06 / 45.16 / 75.59 | 158,055 |
| 2v2 | ball | 11.20 / 33.72 / 63.56 | 10.87 / 34.77 / 69.13 | 151,225 |
| 2v2 | car | 15.99 / 41.14 / 62.79 | 17.36 / 43.78 / 69.68 | 285,343 |
| 3v3 | ball | 11.41 / 36.23 / 66.10 | 11.14 / 37.84 / 74.45 | 189,151 |
| 3v3 | car | 17.19 / 42.33 / 69.30 | 18.69 / 44.66 / 72.23 | 555,544 |

One-step simulated car p99 errors are now below linear extrapolation in all three game sizes, and gated dodge inference further reduced one-step car error across all game sizes and quantiles (overall train car p50/p90/p99 improved from 17.01/42.23/68.37 UU to 16.96/41.99/68.01 UU). The evaluator's top training car error over linear extrapolation fell from about 10,724 UU before demolition correction to 932 UU after selecting primary cars; the metric also excludes retired duplicate actors. Across train and validation, 32,751 overlapping or otherwise shadowed car-frame records were skipped, and active replay pawn evidence corrected 592 simulated demolition flags. Validation one-step car p99 fell from 76.68/73.38/76.39 UU to 66.60/61.89/72.19 UU across 1v1/2v2/3v3. Collision, kickoff, hitbox, and unknown input cases still need investigation. The masked results do not establish accurate pad-synchronization, event, or scoreboard reconstruction.

## Boost activation check

On `train`, 2,818 of 2,824 short observed boost-amount intervals ending in an odd-to-even boost activation counter transition showed boost depletion. On `validation`, 3,011 of 3,014 did. This supports interpreting odd counter values as active boost input. At the time of that ablation, enabling the signal improved four-frame validation car median/p90 from 16.0/46.8 to 15.4/45.7 UU in 1v1, 16.1/41.3 to 16.0/40.0 in 2v2, and 17.7/47.0 to 17.4/46.8 in 3v3. The initial small one-step p99 regression was overtaken by the later actor fixes above. The counter interpretation is an inference, not a direct action field.

## Four-frame masked boost prediction

The evaluator withholds fresh car boost amounts during the same four-frame rigid-body mask intervals, while leaving replay boost activation evidence (`boost_active_raw`) available to the simulator. RocketSim simulates continuous boost depletion during active boosting and boost-pad pickups if crossed. Predicted boost amounts (0–100 scale) are compared against fresh original replay boost values and a hold-last-observed-boost baseline. As with kinematics, samples require primary linked cars, active game state, and a last observation gap $\le 0.5$ seconds. All 60 train and 60 validation replays converted without failure.

### Boost error by mask horizon (all game sizes pooled)

Each entry is sample count and absolute boost error quantiles (p50 / p90 / p99) on the 0–100 boost scale.

| Split | Horizon | Samples | RocketSim p50 / p90 / p99 | Hold p50 / p90 / p99 |\n| --- | ---: | ---: | ---: | ---: |\n| train | 1 | 220 | 0.59 / 12.16 / 88.95 | 7.84 / 14.12 / 95.69 |\n| train | 2 | 209 | 0.92 / 12.48 / 100.00 | 8.63 / 16.47 / 100.00 |\n| train | 3 | 214 | 0.52 / 12.16 / 96.08 | 7.84 / 17.65 / 100.00 |\n| train | 4 | 202 | 0.65 / 12.75 / 97.65 | 8.24 / 22.75 / 100.00 |\n| validation | 1 | 251 | 0.72 / 12.65 / 100.00 | 6.67 / 32.55 / 100.00 |\n| validation | 2 | 211 | 0.49 / 12.21 / 100.00 | 8.24 / 20.39 / 100.00 |\n| validation | 3 | 253 | 0.85 / 12.49 / 100.00 | 9.02 / 28.63 / 100.00 |\n| validation | 4 | 197 | 0.47 / 12.21 / 100.00 | 7.45 / 15.69 / 100.00 |

### Boost error by game size at horizon 4

| Split | Game size | Samples | RocketSim p50 / p90 / p99 | Hold p50 / p90 / p99 |\n| --- | --- | ---: | ---: | ---: |\n| train | 1v1 | 39 | 0.57 / 17.25 / 97.65 | 11.37 / 17.25 / 97.65 |\n| train | 2v2 | 47 | 0.64 / 12.46 / 76.86 | 8.24 / 12.16 / 92.55 |\n| train | 3v3 | 116 | 0.76 / 12.26 / 100.00 | 7.45 / 44.31 / 100.00 |\n| validation | 1v1 | 36 | 1.06 / 13.18 / 100.00 | 9.02 / 74.51 / 100.00 |\n| validation | 2v2 | 71 | 0.33 / 12.16 / 100.00 | 6.67 / 12.94 / 100.00 |\n| validation | 3v3 | 90 | 0.49 / 12.21 / 85.10 | 7.06 / 16.86 / 100.00 |

### Alternate mask schedule (`--mask-seed 239847`) on validation

An independent check using the deterministic pseudo-random offset schedule confirms the boost metrics:

| Horizon | Samples | RocketSim p50 / p90 / p99 | Hold p50 / p90 / p99 |\n| ---: | ---: | ---: | ---: |\n| 1 | 191 | 0.46 / 12.16 / 27.83 | 8.63 / 17.25 / 100.00 |\n| 2 | 223 | 0.49 / 12.16 / 100.00 | 7.45 / 14.51 / 100.00 |\n| 3 | 223 | 0.47 / 12.16 / 100.00 | 7.84 / 17.65 / 100.00 |\n| 4 | 189 | 0.64 / 12.16 / 83.92 | 8.63 / 15.29 / 100.00 |

At horizon 4 by game size with `--mask-seed 239847`: 1v1 (43 samples) RocketSim 0.64 / 12.16 / 100.00 vs Hold 8.63 / 33.33 / 100.00; 2v2 (58 samples) RocketSim 0.59 / 12.16 / 44.90 vs Hold 7.06 / 12.55 / 41.57; 3v3 (88 samples) RocketSim 0.64 / 12.16 / 21.45 vs Hold 10.20 / 15.29 / 100.00.

### Boost findings and limitations

1. **Continuous depletion tracking (p50):** RocketSim median boost error is under 1.0 boost unit across all horizons and game sizes (0.33 to 1.06 boost units, representing $\le 1\%$ of total boost capacity). In contrast, the hold-last-observed baseline median error is 6.3 to 11.4 boost units. This confirms that simulating boost depletion from inferred active boost input (`boost_active_raw`) closely matches ground-truth consumption.
2. **Small boost-pad pickups (p90):** Across nearly every horizon and game-size slice, RocketSim p90 error clusters tightly around **12.16 boost units**. In Rocket League, small boost pads provide exactly 12% boost (or 31 raw ticks, $31 \times 100 / 255 \approx 12.156863$ boost units). This reveals that the predominant discrepancy at p90 is small pad pickups occurring during the four-frame mask that the simulator does not reproduce—either because masked car body trajectory deviated from the pickup radius, or because pad cooldown state in RocketSim was not synchronized with the match.
3. **Orb pickups and respawns (p99):** Extreme tail errors reach 80–100 boost units, corresponding to 100-boost orb pickups or kickoff/respawn refills that occurred during the withheld window.
4. **Limitations:** Only frames with fresh original boost packets are evaluated (~200–250 per horizon per split); replay boost updates are replicated at network rates rather than 120 Hz ticks. Simultaneous body masking also couples car position error to boost-pad collision detection.

## Four-frame masked kinematics

The same mask measures fresh linear velocity (UU/s), rotation angle (degrees), and angular velocity (radians/s). Each field uses its own last unmasked observation and is counted only when that field is fresh in the original frame and its observation gap is at most 0.5 seconds in active play. The comparison baseline holds that field's last value. Replay angular velocity is scaled by 0.01 before comparison. The primary-car filter applies; sample counts can differ because replay rigid-body velocity fields are optional.

Validation results at mask horizon 4 with default motion-gated dodge flip inference are below. Each value is median / p90 error. All 60 validation replays converted without failure.

| Size | Body | Field | RocketSim | Hold | Samples |
| --- | --- | --- | ---: | ---: | ---: |
| 1v1 | ball | Velocity (UU/s) | 5.46 / 21.84 | 86.99 / 347.79 | 1,720 |
| 1v1 | ball | Rotation (degrees) | 2.86 / 5.77 | 40.11 / 51.57 | 1,720 |
| 1v1 | ball | Angular velocity (rad/s) | 0.00 / 0.07 | 0.00 / 1.14 | 1,720 |
| 1v1 | car | Velocity (UU/s) | 36.82 / 210.52 | 180.27 / 648.79 | 1,592 |
| 1v1 | car | Rotation (degrees) | 4.37 / 24.95 | 20.32 / 59.16 | 1,592 |
| 1v1 | car | Angular velocity (rad/s) | 0.97 / 4.01 | 1.28 / 4.19 | 1,592 |
| 2v2 | ball | Velocity (UU/s) | 5.44 / 21.65 | 86.79 / 527.24 | 1,733 |
| 2v2 | ball | Rotation (degrees) | 2.86 / 5.73 | 40.11 / 51.57 | 1,733 |
| 2v2 | ball | Angular velocity (rad/s) | 0.00 / 0.06 | 0.00 / 1.65 | 1,733 |
| 2v2 | car | Velocity (UU/s) | 32.25 / 171.74 | 198.85 / 628.28 | 3,201 |
| 2v2 | car | Rotation (degrees) | 3.16 / 20.55 | 17.53 / 53.98 | 3,203 |
| 2v2 | car | Angular velocity (rad/s) | 0.61 / 3.58 | 1.09 / 3.59 | 3,201 |
| 3v3 | ball | Velocity (UU/s) | 5.46 / 27.26 | 88.73 / 987.46 | 1,945 |
| 3v3 | ball | Rotation (degrees) | 2.86 / 6.63 | 43.31 / 51.57 | 1,945 |
| 3v3 | ball | Angular velocity (rad/s) | 0.00 / 0.07 | 0.00 / 2.70 | 1,945 |
| 3v3 | car | Velocity (UU/s) | 31.07 / 154.61 | 209.35 / 641.28 | 5,605 |
| 3v3 | car | Rotation (degrees) | 2.96 / 18.78 | 17.53 / 52.84 | 5,606 |
| 3v3 | car | Angular velocity (rad/s) | 0.54 / 3.34 | 1.09 / 3.52 | 5,605 |

With gated dodge inference, car linear velocity median dropped from 51.9 / 43.3 / 39.7 UU/s to 36.8 / 32.3 / 31.1 UU/s across 1v1/2v2/3v3 (a 22–29% median error reduction), and p90 dropped by 28–34 UU/s. Car rotation p90 dropped by 4.1–5.1 degrees (e.g. from 29.06 to 24.95 deg in 1v1, 25.68 to 20.55 deg in 2v2, and 22.94 to 18.78 deg in 3v3).

## Rotation calibration and independent mask

`cargo run --release --bin calibrate_rotation -- replays/train` compared fresh car angular-velocity packets against the quaternion change over short active-play intervals. On 367,746 ground intervals, the world-frame interpretation had median/p90 vector error 0.31/1.14 rad/s, versus 0.33/1.86 when rotated from car-local coordinates and 3.75/6.06 when negated. On 262,200 air intervals, the corresponding errors were 0.62/2.64, 4.20/9.76, and 7.26/12.49 rad/s. The median direction alignment of world-frame angular velocity with quaternion motion was approximately 1.00 in both groups. This confirms the existing 0.01 scale and world-coordinate mapping; the remaining masked error is unlikely to come from a coordinate transform.

`evaluate_corpus --mask-seed 239847` uses each replay's SHA-256 and a fixed seed to select one four-frame gap at a different deterministic offset in every 100-frame block. Reports are `target/train-conversion-metrics-alt-mask.json` and `target/validation-conversion-metrics.json`. All 60 replays converted in each split with zero failures; the `test` split remains sealed. At horizon 4, car position RocketSim median/p90 versus constant-velocity extrapolation (UU) with gated dodge is:

| Split | Size | RocketSim | Linear |
| --- | --- | ---: | ---: |\n| train | 1v1 | 17.3 / 47.4 | 27.5 / 66.8 |\n| train | 2v2 | 16.2 / 39.8 | 26.4 / 62.9 |\n| train | 3v3 | 18.1 / 47.3 | 30.2 / 65.8 |\n| validation | 1v1 | 15.8 / 41.8 | 24.7 / 61.7 |\n| validation | 2v2 | 15.1 / 38.7 | 26.3 / 64.2 |\n| validation | 3v3 | 16.1 / 42.2 | 28.6 / 64.1 |

Under this independent schedule, validation horizon-4 car position p90 improved from 42.01 to 41.8 UU in 1v1, from 39.77 to 38.7 UU in 2v2, and from 45.30 to 42.2 UU in 3v3 (-3.1 UU!).

## Dodge torque calibration and motion-gated flip inference

### Replay torque calibration (`src/bin/calibrate_dodge.rs`)

Inspection across all 60 training replays identified 14,090 dodge counter activations on primary linked cars in active play. Of these, 13,914 (98.8%) carry a fresh `TAGame.CarComponent_Dodge_TA:DodgeTorque` vector in the same frame.

1. **Torque vector coordinate system:**
   - $T_x \in [-2.60, +2.60]$ (median absolute value 2.597): roll/yaw torque component with constant scale factor 2.60.
   - $T_y \in [-2.24, +2.24]$ (median absolute value 2.236): pitch torque component with constant scale factor 2.24.
   - $T_z = 0.000$ strictly in 100.0% of samples (no vertical torque).
   - Inverted stick mapping: $j_x = -T_x / 2.60$ and $j_y = -T_y / 2.24$.
   - The normalized stick magnitude $\sqrt{j_x^2 + j_y^2}$ has median exactly 1.000, with 100.0% of samples falling in $[0.90, 1.05]$.
   - In RocketSim car controls, `pitch = -T_y / 2.24` and `yaw = -T_x / 2.60`.

2. **Dodge impulse timing & duplicate impulse hazard:**
   - When a fresh rigid-body packet arrives at dodge activation frame $F$ (`linear_velocity.frame == F`):
     - The velocity change from frame $F-1$ to $F$ along the dodge direction has median **+461.25 UU/s** (interquartile range +365.4 to +524.8 UU/s).
     - The velocity change from frame $F$ to $F+1$ has median **+3.61 UU/s**.
   - This proves that when velocity is freshly reported at the activation frame, the ~500 UU/s linear dodge impulse has **already taken effect** in the replay observation.
   - Injecting an active jump control into RocketSim while linear velocity is already present causes an immediate, duplicate 500 UU/s impulse, blowing out velocity and position prediction on subsequent frames.

3. **Motion-gated flip rule:**
   - When a dodge activation edge occurs while airborne:
     - If velocity is **unobserved** (`!car.body.linear_velocity.as_ref().is_some_and(|v| v.frame == frame)`), as happens during masked evaluation intervals or missing network frames, pass `controls.jump = true`, `controls.pitch = pitch`, and `controls.yaw = yaw` to trigger the dodge impulse and flip rotation in RocketSim.
     - If velocity is **already observed**, do not inject a duplicate jump control; instead, directly mark the car's state as flipping (`state.has_flipped = true`, `state.is_flipping = true`, `state.flip_rel_torque = Vec3A::new(tx / 2.60, ty / 2.24, 0.0)`, `state.flip_time = 0.0`). RocketSim simulates rotational flip torque without an extra linear impulse.

### Ablation results on train

| Metric | Baseline (`--no-inferred-dodge`) | Ungated (`--inferred-dodge`) | Gated default (`--gated-dodge`) |
| --- | ---: | ---: | ---: |
| One-step 1v1 car p50/p90/p99 (UU) | 17.86 / 42.33 / 69.43 | 17.98 / 43.54 / 72.46 | 17.85 / 42.29 / 69.33 |
| One-step 2v2 car p50/p90/p99 (UU) | 16.05 / 41.39 / 63.49 | 16.07 / 42.01 / 67.10 | 15.99 / 41.14 / 62.79 |
| One-step 3v3 car p50/p90/p99 (UU) | 17.25 / 42.61 / 69.74 | 17.26 / 43.03 / 70.93 | 17.19 / 42.33 / 69.30 |
| One-step all car p50/p90/p99 (UU) | 17.01 / 42.23 / 68.37 | 17.04 / 42.83 / 70.36 | 16.96 / 41.99 / 68.01 |
| Masked car pos h=4 p50/p90 (UU) | 17.13 / 42.41 | 16.83 / 41.12 | 16.77 / 40.25 |
| Masked car vel h=4 p50/p90 (UU/s) | 45.16 / 216.77 | 35.25 / 186.07 | 34.99 / 180.46 |
| Masked car rot h=4 p50/p90 (deg) | 3.45 / 26.05 | 3.39 / 20.96 | 3.39 / 20.96 |

Ungated dodge worsens one-step prediction significantly (p99 rises by +2.0 UU). In contrast, motion-gated dodge improves one-step prediction across every game size and quantile, while achieving the largest reduction in masked position, velocity (-36.3 UU/s at p90), and rotation error (-5.1 deg at p90).

### Validation generalization

On the 60 validation replays, the gains hold across both fixed-mask and pseudo-random (`--mask-seed 239847`) schedules:
- Validation one-step all car p50/p90/p99 improved from 16.45/41.33/69.87 to 16.39/41.09/69.61 UU.
- Validation fixed-mask car position h=4 p90 dropped from 41.63 to 39.55 UU (-2.08 UU).
- Validation fixed-mask car velocity h=4 p90 dropped from 202.04 to 168.55 UU/s (-33.5 UU/s, -16.6%).
- Validation fixed-mask car rotation h=4 p90 dropped from 24.77 to 20.47 degrees (-4.30 deg).
- Validation pseudo-random mask seed 239847 car position h=4 p90 dropped from 43.44 to 40.66 UU (-2.78 UU).

Gated dodge inference is enabled by default (`infer_dodge_from_active = true`, `gate_dodge_on_observed_impulse = true`). Baseline and ungated modes are accessible via `--no-inferred-dodge` and `--inferred-dodge`.

## Loadout body products and hitboxes

`TAGame.PRI_TA:ClientLoadouts` supplies a body product ID for each team. The extractor preserves both values on each player and attaches the currently selected one to a linked car. The user's August 2026 `items.csv` supplies product names; [Rocket League's official hitbox list](https://www.epicgames.com/help/c-37599050/c-32343914/a20257614?lang=en-US), additional [Season 22](https://www.rocketleague.com/news/rocket-league-season-22-training-rivalries-and-rewards) and [Season 23](https://www.rocketleague.com/news/hit-the-pitch-for-the-world-cup-in-rocket-league-season-23) announcements, and a localized official listing supply families. RocketSim's dedicated Psyclops preset covers that special body. The checked-in inputs, aliases, source URLs, generator, and unresolved rows are documented in [data/README.md](data/README.md). RocketSim's matching preset is used at car-slot creation.

| Product ID | Body | Hitbox | Playing slots in train |\n| ---: | --- | --- | ---: |\n| 21 | Backfire | Octane | 1 |\n| 22 | Breakout | Breakout | 1 |\n| 23 | Octane | Octane | 53 |\n| 26 | Gizmo | Octane | 1 |\n| 403 | Dominus | Dominus | 5 |\n| 4284 | Fennec | Octane | 178 |\n| 7012 | Tesla Cybertruck | Hybrid | 1 |\n| 7477 | Nomad GXT | Merc | 0 |\n| 7979 | Stampede | Merc | 0 |

The generated map covers 213 of 238 `Body` product rows; 11 of the 25 unresolved rows are generic drop or mystery labels, and 14 are named vehicles without a verified assignment in the checked sources. Unknown IDs remain raw and use an Octane fallback if selected by a playing car. The two rare training PRI products 7477 and 7979 were attached to non-playing actors; 7979 is now identified as Stampede, which the official list puts in Merc. All 240 playing train slots and all 240 playing validation slots have mapped IDs. Validation now has 237 Octane, two Plank, and one Hybrid slot. Its four formerly unresolved playing IDs are 25 Road Hog (Octane), 1691 Mantis (Plank), 1919 Centio (Plank), and 10900 Shokunin (Octane).

Compared with the previous eight-ID map, full-corpus train position metrics are unchanged. Validation gains two Plank slots. Four-frame car position median/p90 is unchanged in 1v1 and 2v2 to three decimals; 3v3 moves from 17.221/45.252 to 17.233/45.235 UU. One-step p90 in the Mantis replay improves by 0.008 UU, while the Centio replay worsens by 0.136 UU; other percentiles move in both directions. This is a state-fidelity correction, and the small sample does not establish a broad prediction gain. `--octane-hitbox` reproduces the original all-Octane setup.


## Boost pad pickup reconciliation and cooldown tracking

### Replay pickup extraction and spatial matching

Rocket League replays replicate vehicle pickups via TAGame.VehiclePickup_TA:NewReplicatedPickupData, which contains:
- pad_actor_id: The transient actor ID of the pickup entity.
- pad_actor_name: Name from the object table (e.g. cs_p.TheWorld:PersistentLevel.VehiclePickup_Boost_TA_0).
- instigator_car_id: The actor ID of the car that picked up the pad.
- picked_up: Counter byte (odd on pickup, 255 on respawn).

In observations.rs, these are captured per frame as pad_pickups: Vec<PadPickup>.

In conversion.rs, when options.sync_boost_pad_pickups is enabled (default 	rue):
1. **Pad index mapping:** When a pickup counter changes, the instigator car's xy position is matched to the nearest RocketSim boost pad configuration. Because car-to-pad contact distances in replays range from ~120 to ~280 UU, a spatial threshold of 350 UU uniquely resolves the pad. The actor ID mapping is cached for the actor's lifetime.
2. **Cooldown synchronization:**
   - On pickup (picked_up % 2 == 1): Sets RocketSim rena.set_boost_pad_state(idx, BoostPadState { cooldown }), with 10.0s for big pads and 4.0s for small pads.
   - On respawn (picked_up == 255): Resets cooldown = 0.0.
3. **No future data leakage:** In evaluate_corpus.rs, rame.pad_pickups is explicitly cleared during masked evaluation windows, so pad events are only known when observed before the withheld gap.

### Evaluation metrics and ablation results

All 60 train and 60 validation replays converted with zero failures.

#### Masked boost error on validation: sync pads vs no sync pads

| Horizon | Samples | Pad sync sim p50 / p90 / p99 | No pad sync sim p50 / p90 / p99 | Hold baseline p50 / p90 / p99 |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 251 | **0.621** / 12.892 / 100.000 | 0.719 / 12.654 / 100.000 | 6.667 / 32.549 / 100.000 |
| 2 | 211 | **0.474** / 12.239 / 100.000 | 0.490 / 12.212 / 100.000 | 8.235 / 20.392 / 100.000 |
| 3 | 253 | **0.621** / 12.680 / 100.000 | 0.850 / 12.490 / 100.000 | 9.020 / 28.627 / 100.000 |
| 4 | 197 | **0.392** / 12.157 / 100.000 | 0.458 / 12.212 / 100.000 | 7.451 / 15.686 / 100.000 |

Across all horizons, pad synchronization improves median simulated boost error on validation:
- Horizon 1: 0.719 -> 0.621 boost units (vs hold 6.667, **>10x improvement over hold**).
- Horizon 2: 0.490 -> 0.474 boost units (vs hold 8.235, **>17x improvement over hold**).
- Horizon 3: 0.850 -> 0.621 boost units (vs hold 9.020, **>14x improvement over hold**).
- Horizon 4: 0.458 -> 0.392 boost units (vs hold 7.451, **>19x improvement over hold**).

Car position errors generalize consistently without regression:
- Train all car position p50 / p90 / p99: 16.96 / 41.99 / 68.01 UU.
- Validation all car position p50 / p90 / p99: 16.39 / 41.09 / 69.61 UU.

Enabled by default with --sync-pads and ablatable with --no-sync-pads.


## Aerial steer control routing (`infer_air_steer_controls`)

### Physics analysis and calibration (`src/bin/calibrate_air_steer.rs`)

In RocketSim (update_air_torque), when a car is airborne (`!state.is_on_ground`), `CarControls.steer` only steers wheels on ground surfaces. In the air, RocketSim ignores steer and solely evaluates `CarControls.yaw`, `CarControls.roll`, and `CarControls.pitch`. When these controls are left at zero, RocketSim applies heavy aerodynamic angular damping (`air_control::DAMPING`), bringing rotational velocity to a halt.

In Rocket League replays, players replicate `ReplicatedSteer` (horizontal stick X) continuously. Pitch stick movements (stick Y) are not replicated in standard soccar network frames. Calibration on all 60 
`replays/train` (`src/bin/calibrate_air_steer.rs`) revealed:
- 709,925 total airborne frames across the training corpus.
- 68.1% of air frames (483,261) have non-neutral steer (|steer| > 0.1).
- Steer and local car yaw have the same sign in 283,213 frames vs opposite sign in 94,195 frames (a 3:1 majority), directly matching RocketSim's internal air torque coordinate convention (`dir_yaw = up_dir`).
- Routing steer to `controls.yaw` when airborne (`!state.is_on_ground`) allows RocketSim to accurately simulate player-guided aerial yaw rotation and cancel unwanted aerodynamic damping during active steering.

### Evaluation metrics and ablation results

Evaluated on all 60 `train` and 60 `validation` replays with `--no-infer-air-steer` ablation. Zero failures.

#### Masked car rotation and angular velocity p50 by horizon (Train)

| Metric | Horizon | No air steer (Baseline) | Air steer (`infer_air_steer_controls`) | Hold baseline |
| --- | ---: | ---: | ---: | ---: |
| Rotation angle (deg) | 1 | 1.87 | **1.84** | 8.10 |
| Rotation angle (deg) | 2 | 1.90 | **1.87** | 9.54 |
| Rotation angle (deg) | 3 | 2.74 | **2.66** | 13.36 |
| Rotation angle (deg) | 4 | 3.39 | **3.25** | 18.16 |
| Angular velocity (rad/s) | 1 | 0.442 | **0.414** | 0.511 |
| Angular velocity (rad/s) | 2 | 0.485 | **0.456** | 0.668 |
| Angular velocity (rad/s) | 3 | 0.599 | **0.565** | 0.937 |
| Angular velocity (rad/s) | 4 | 0.618 | **0.579** | 1.186 |

- Masked airborne angular velocity error p50 on train dropped from 1.343 rad/s to **1.255 rad/s**.
- One-step car position p50 / p90 / p99: 16.956 / 41.991 / 68.012 -> **16.955 / 41.991 / 68.033 UU**.

#### Masked car rotation and angular velocity p50 by horizon (Validation Generalization)

| Metric | Horizon | No air steer (Baseline) | Air steer (`infer_air_steer_controls`) | Hold baseline |
| --- | ---: | ---: | ---: | ---: |
| Rotation angle (deg) | 1 | 1.89 | **1.86** | 8.15 |
| Rotation angle (deg) | 2 | 1.89 | **1.85** | 9.71 |
| Rotation angle (deg) | 3 | 2.53 | **2.47** | 12.99 |
| Rotation angle (deg) | 4 | 3.18 | **3.08** | 17.89 |
| Angular velocity (rad/s) | 1 | 0.417 | **0.391** | 0.484 |
| Angular velocity (rad/s) | 2 | 0.473 | **0.436** | 0.650 |
| Angular velocity (rad/s) | 3 | 0.545 | **0.516** | 0.898 |
| Angular velocity (rad/s) | 4 | 0.612 | **0.578** | 1.125 |

- Masked airborne angular velocity error p50 on validation dropped from 1.300 rad/s to **1.218 rad/s**.
- One-step car position p50 / p90 / p99: 16.391 / 41.092 / 69.611 -> **16.391 / 41.092 / 69.613 UU**.

Enabled by default (`infer_air_steer_controls = true`) with `--infer-air-steer` and ablatable via `--no-infer-air-steer`.
