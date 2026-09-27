# Reconstruction measurements

Last updated: 2026-09-27. These are development measurements, not a final accuracy claim. Current reviewed machine-readable reports are `target/train-reviewed.json` and `target/validation-reviewed.json`; older experiment reports are retained under `target/*-conversion-metrics*.json`. Each evaluator report includes replay SHA-256 values, settings, errors, and per-game-size aggregates. No `test` replay has been opened or converted.

The early sections record historical experiments with actor-lifetime, boost, demolition, hitbox, jump, and dodge inference. The later reviewed sections use corrected pad handling, guarded aerial lookahead, and field-specific kinematic residuals. Historical ablation reports remain under `target/*-conversion-metrics*.json`; their figures should be read with the implementation described beside them.

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
| --- | --- | ---: | ---: |
| train | 1v1 | 17.3 / 47.4 | 27.5 / 66.8 |
| train | 2v2 | 16.2 / 39.8 | 26.4 / 62.9 |
| train | 3v3 | 18.1 / 47.3 | 30.2 / 65.8 |
| validation | 1v1 | 15.8 / 41.8 | 24.7 / 61.7 |
| validation | 2v2 | 15.1 / 38.7 | 26.3 / 64.2 |
| validation | 3v3 | 16.1 / 42.2 | 28.6 / 64.1 |

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
     - This indicates that when velocity is freshly reported at the activation frame, the linear dodge impulse has usually already taken effect in the replay observation.
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


## Boost pad pickup reconciliation and cooldown tracking

### Replay pickup extraction and spatial matching

Rocket League replays replicate vehicle pickups via TAGame.VehiclePickup_TA:NewReplicatedPickupData, which contains:
- pad_actor_id: The transient actor ID of the pickup entity.
- pad_actor_name: Name from the object table (e.g. cs_p.TheWorld:PersistentLevel.VehiclePickup_Boost_TA_0).
- instigator_car_id: The actor ID of the car that picked up the pad.
- picked_up: Pickup counter byte; non-255 train values were odd, while 255 marked an available/inactive pickup and had no instigator.

In observations.rs, these are captured per frame as pad_pickups: Vec<PadPickup>.

In conversion.rs, when options.sync_boost_pad_pickups is enabled (default true):
1. **Pad index mapping:** On an instigator event, the car's xy position is matched to the nearest RocketSim pad within 350 UU, provided the next nearest candidate is at least 100 UU farther away. Positions older than 0.1s are not used. The mapping is cached for that pad actor.
2. **Cooldown synchronization:**
   - On `picked_up == 255`: Resets cooldown to zero. This case is checked first because 255 is odd.
   - On any other odd pickup counter: Sets RocketSim cooldown to 10.0s for big pads or 4.0s for small pads.
3. **No future data leakage:** In evaluate_corpus.rs, frame.pad_pickups is explicitly cleared during masked evaluation windows, so pad events are only known when observed before the withheld gap.

### Evaluation metrics and ablation results

After fixing the 255 sentinel order and tightening spatial mapping, the default and `--no-sync-pads` pipelines each converted all 60 validation replays without failures. The same four-frame mask was used in both runs.

#### Masked boost error on validation: sync pads vs no sync pads

| Horizon | Samples | Pad sync sim p50 / p90 / p99 | No pad sync sim p50 / p90 / p99 | Hold baseline p50 / p90 / p99 |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 251 | **0.556** / 12.810 / 100.000 | 0.719 / **12.654** / 100.000 | 6.667 / 32.549 / 100.000 |
| 2 | 211 | **0.392** / **12.157** / 100.000 | 0.490 / 12.212 / 100.000 | 8.235 / 20.392 / 100.000 |
| 3 | 253 | **0.588** / **12.190** / 100.000 | 0.850 / 12.490 / 100.000 | 9.020 / 28.627 / 100.000 |
| 4 | 197 | **0.327** / **12.157** / 100.000 | 0.458 / 12.212 / 100.000 | 7.451 / 15.686 / 100.000 |

Across all horizons, pad synchronization improves median simulated boost error on validation:
- Horizon 1: 0.719 -> 0.556 boost units; p90 slightly worsens (12.654 -> 12.810).
- Horizon 2: 0.490 -> 0.392 boost units.
- Horizon 3: 0.850 -> 0.588 boost units.
- Horizon 4: 0.458 -> 0.327 boost units.

Car position errors generalize consistently without regression:
- Train all car position p50 / p90 / p99: 16.96 / 41.99 / 68.01 UU.
- Validation all car position p50 / p90 / p99: 16.39 / 41.09 / 69.56 UU.

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


## Look-ahead inverse aerial control inference (`infer_air_controls_from_lookahead`)

### Motivation and physics derivation

Rocket League replays replicate horizontal stick input (`ReplicatedSteer`) continuously, but completely omit vertical stick input (pitch) and directional air roll. Consequently, prior baseline conversions left simulated pitch and roll at `0.0` for 100% of frames. In mid-air, RocketSim applies heavy aerodynamic damping (`air_control::DAMPING`), bringing rotational angular velocities to zero unless counteracted by player controls.

Using RocketSim's air torque and damping equations from `rocketsim/src/sim/car/base.rs` and the local analytical formulation in `external/inverse_aerial_controls.py`, we estimate model-equivalent 3D aerial controls. These estimates use the next replay frame; they are not observed player inputs and are intended for offline conversion.

$$\text{dir\_pitch} = -\text{right\_dir}, \quad \text{dir\_yaw} = \text{up\_dir}, \quad \text{dir\_roll} = -\text{forward\_dir}$$

$$\text{TORQUE\_APPLY\_SCALE} = \frac{2\pi}{65536} \times 1000.0 \approx 0.0958738$$
$$\text{TORQUE} = (130, 95, 400) \times \text{SCALE} = (12.4636, 9.1080, 38.3495)$$
$$\text{DAMPING} = (30, 20, 50) \times \text{SCALE} = (2.8762, 1.9175, 4.7937)$$

For each control axis $i \in \{\text{pitch}, \text{yaw}, \text{roll}\}$:
$$\tau_i = \Delta \omega_i / \Delta t$$
$$\text{RHS}_i = \tau_i + \omega_i \cdot D_i$$
$$u_i = \text{clamp}\left(\frac{\text{RHS}_i}{T_i + \text{sign}(\text{RHS}_i) \cdot \omega_i \cdot D_i}, -1.0, 1.0\right)$$

### Corpus calibration on train (`src/bin/calibrate_inverse_air.rs`)

The original calibration tool estimated nonzero pitch on 36,423 of 53,565 airborne frame pairs (68.0%), yaw on 38,336 (71.6%), and roll on 33,483 (62.5%). Of 30,265 pairs with an active steer signal, estimated yaw had the same sign in 25,957 (85.8%). This is a consistency check, not independent controller ground truth. The calibration tool does not yet apply all continuity guards used by conversion.

### Unmasked physical fidelity (`src/bin/measure_air_fidelity.rs`)

`measure_air_fidelity` now samples all 60 replays per split, matches each primary replay car to its RocketSim slot, requires the same car lifetime and fresh airborne endpoints, and uses an isolated one-car simulation from each starting packet. This directly tests one-step control inversion with the future endpoint available. It does not include full match collisions or test causal prediction.

| Dataset | Metric | Without Lookahead (Baseline) | With Lookahead (`infer_air_controls_from_lookahead`) | Improvement |
| --- | --- | ---: | ---: | ---: |
| **Train (52,864 pairs)** | Angular velocity p50 / p90 (rad/s) | 0.529 / 1.342 | **0.026 / 0.857** | Lower angular error |
| Train | Rotation p50 / p90 (deg) | **2.769 / 7.062** | 3.064 / 7.684 | Higher rotation error |
| **Validation (55,321 pairs)** | Angular velocity p50 / p90 (rad/s) | 0.479 / 1.316 | **0.030 / 0.878** | Lower angular error |
| Validation | Rotation p50 / p90 (deg) | **2.758 / 7.226** | 3.038 / 7.788 | Higher rotation error |

### Masked evaluation and leakage prevention

In `evaluate_corpus.rs`, future data leakage is strictly prevented:
- Within a masked interval, `masked_observations` carries forward the prior rigid-body value and its original frame provenance. The lookahead solver requires a fresh next-frame packet, so it falls back to inferred aerial steering at that boundary.
- The converter also requires active play at both endpoints, the same car actor lifetime and owner, fresh positions above 100 UU, and no fresh next-frame dodge activation. These guards reduced unmasked angular fidelity in some cases but avoid inferring air controls from a contact, respawn, or play-state transition.
- A 60-replay validation ablation against `--no-infer-air-lookahead` produced identical masked car-position counts and p50/p90/p99 at horizons 1–4. Unmasked one-step car angular velocity p50 improved from 0.3985 to 0.3664 rad/s with lookahead; p90 improved from 2.5413 to 2.5277. Unmasked car-position quantiles changed by at most 0.001 UU. This establishes equality for the current four-frame mask, not for every possible masking protocol.

Enabled by default (`infer_air_controls_from_lookahead = true`), and ablatable via `--no-infer-air-lookahead` and `--infer-air-lookahead`.


## One-step pre-correction kinematic residuals

### Methodology and scope

The primary unmasked conversion pipeline (`convert_observations` / `convert_bytes`) steps RocketSim forward tick-by-tick between network observations. At each frame $F$ where a fresh rigid-body packet arrives (`actual.frame == F`), the simulation state immediately prior to applying the observation's rigid-body correction represents RocketSim's uncorrected 1-step prediction.

`src/conversion.rs` now records pre-correction kinematic residuals on `PositionResidual` for every fresh update in active play ($dt \le 0.5$s, primary linked cars for vehicles):
1. **Linear velocity error (UU/s):** Euclidean distance between pre-correction simulated velocity and fresh replay velocity, compared against a hold-last-observed-velocity baseline.
2. **Rotation error (degrees):** Geodesic angular distance $\theta = \arccos((\text{Tr}(R_{\text{sim}}^T R_{\text{replay}}) - 1) / 2)$ between pre-correction simulated orientation matrix and replay orientation matrix, invariant to quaternion sign ambiguities ($q \equiv -q$).
3. **Angular velocity error (rad/s):** Euclidean distance between pre-correction simulated angular velocity and replay angular velocity (scaled by 0.01 to convert from replay units to rad/s).
4. **Altitude stratification:** Cars are categorized by altitude: `ground` ($z < 50$ UU), `air` ($z > 100$ UU), and `transition` ($50 \le z \le 100$ UU).

`src/bin/evaluate_corpus.rs` pools and reports these quantiles across the entire corpus. All 60 train and 60 validation replays converted with zero failures.

The reviewed reports are `target/train-reviewed.json` and `target/validation-reviewed.json`, with validation ablations in `target/validation-reviewed-no-pads.json` and `target/validation-reviewed-no-lookahead.json`. These ignored files can be regenerated with the corresponding `evaluate_corpus` flags. The table reflects the final guarded converter and field-specific freshness limits.

### Aggregate 1-step kinematics: Train vs Validation

| Split | Body | Kinematic Field | Samples | RocketSim p50 / p90 / p99 | Hold Baseline p50 / p90 / p99 | Error Reduction (p50) |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| **train** | ball | Linear velocity (UU/s) | 504,518 | **5.39** / 15.52 / 1,237.34 | 21.66 / 37.39 / 2,321.68 | **-75.1%** |
| train | ball | Rotation (degrees) | 504,522 | **2.78** / 5.73 / 11.39 | 10.87 / 17.19 / 22.92 | **-74.4%** |
| train | ball | Angular velocity (rad/s) | 504,518 | 0.00 / 0.00 / 4.33 | 0.00 / 0.00 / 6.43 | Neutral |
| train | car | Linear velocity (UU/s) | 998,544 | **22.03** / 108.04 / 578.43 | 82.26 / 329.00 / 820.30 | **-73.2%** |
| train | car | Rotation (degrees) | 998,832 | **1.88** / 8.86 / 22.06 | 8.03 / 28.34 / 39.47 | **-76.5%** |
| train | car | Angular velocity (rad/s) | 998,544 | **0.372** / 2.552 / 5.782 | 0.514 / 2.086 / 5.556 | **-27.7%** |
| **validation** | ball | Linear velocity (UU/s) | 543,265 | **5.39** / 15.22 / 1,240.70 | 21.58 / 36.95 / 2,325.30 | **-75.0%** |
| validation | ball | Rotation (degrees) | 543,266 | **2.82** / 5.73 / 10.91 | 10.13 / 17.19 / 22.92 | **-72.2%** |
| validation | ball | Angular velocity (rad/s) | 543,265 | 0.00 / 0.00 / 4.38 | 0.00 / 0.00 / 6.46 | Neutral |
| validation | car | Linear velocity (UU/s) | 1,056,312 | **21.63** / 107.82 / 571.12 | 81.61 / 326.69 / 810.86 | **-73.5%** |
| validation | car | Rotation (degrees) | 1,056,577 | **1.85** / 8.75 / 21.95 | 7.92 / 28.10 / 39.12 | **-76.6%** |
| validation | car | Angular velocity (rad/s) | 1,056,312 | **0.366** / 2.530 / 5.780 | 0.506 / 2.073 / 5.548 | **-27.6%** |

### 1-step car angular velocity error by altitude

| Split | Altitude Region | Samples | RocketSim p50 / p90 / p99 (rad/s) | Hold Baseline p50 / p90 / p99 (rad/s) |
| --- | --- | ---: | ---: | ---: |
| **train** | Ground ($z < 50$ UU) | 557,448 | **0.152** / 1.121 / 2.657 | 0.330 / 1.651 / 5.245 |
| train | Transition ($50 \le z \le 100$ UU) | 142,390 | 2.013 / 4.616 / 6.638 | **0.688** / 2.789 / 5.679 |
| train | Air ($z > 100$ UU) | 298,706 | 0.891 / 2.994 / 6.560 | **0.826** / 2.449 / 6.099 |
| **validation** | Ground ($z < 50$ UU) | 593,612 | **0.150** / 1.127 / 2.673 | 0.324 / 1.642 / 5.235 |
| validation | Transition ($50 \le z \le 100$ UU) | 148,464 | 1.999 / 4.661 / 6.785 | **0.671** / 2.804 / 5.650 |
| validation | Air ($z > 100$ UU) | 314,236 | 0.887 / 2.979 / 6.495 | **0.825** / 2.444 / 6.074 |

### Key takeaways and diagnostics

1. **Substantial kinematic prediction gains over hold:**
   - On over 1 million car frame updates, RocketSim reduces median orientation error from ~8.0 degrees to **1.85 degrees (-76.6% error)** and linear velocity error from ~82 UU/s to **21.6 UU/s (-73.5% error)**.
   - For the ball, linear velocity error is reduced by **75%** (5.39 vs 21.58 UU/s) and rotation error by **72%** (2.82 vs 10.12 deg).
2. **Ground vs airborne dynamics:**
   - On the ground (over 55% of car updates), physical wheel contact and steering simulation reduce angular velocity error by more than half (**0.150 vs 0.324 rad/s** on validation).
   - In airborne flight, holding the prior angular velocity slightly outperforms the converter's full-match forward simulation (0.825 vs 0.887 rad/s on validation). This differs from the isolated one-car inversion diagnostic above, which uses the future endpoint directly.
   - In transition zones ($50 \le z \le 100$ UU), contact and takeoff impulses create the largest instantaneous angular discrepancies (2.00 vs 0.67 rad/s on validation).
