# Reconstruction measurements

Last updated: 2026-09-28. These are development measurements, not a final accuracy claim. Current reviewed baseline machine-readable reports are `target/train-reviewed.json` and `target/validation-reviewed.json`; the latest optional low-air gate reports are `target/train-low-air-cap-gated.json` and `target/validation-low-air-cap-gated.json`. Packet timing reports are `target/train-packet-timing.json` and `target/validation-packet-timing.json`. Older experiment reports are retained under `target/*-conversion-metrics*.json`. Each evaluator report includes replay SHA-256 values, settings, errors, and per-game-size aggregates. No `test` replay has been opened or converted.

## Prior-speed low-air hold gate (2026-09-28)

The frame-level train traces below identified two regimes within 50–100 UU: low-air angular speed below the replay's apparent 5.5 rad/s ceiling often decays or responds to known steering, while near-ceiling speed often persists across masked packets. A paired train analysis joined each horizon-four fresh-rotation target to its horizon-one pre-mask row using replay SHA-256, actor ID, actor lifetime, and mask-window start. It matched 10,052 scored windows; one of the 10,053 aggregate rotation samples had no matching horizon-one car row and was excluded from the feature analysis. Among matched low-air windows with prior observed angular speed in [4, 5) rad/s, angular hold made rotation worse by over 1° in 46 and better by over 1° in 11. At [5.48, 5.51) rad/s, it made rotation better by over 1° in 660 and worse by over 1° in 45. These are retrospective **train** labels; the fresh target rotation was used only to score outcomes, never as a gate input.

`--gated-low-air-angular` now tests the old low-air hold only when the most recent angular packet available before the simulated interval has magnitude at least **5.48 rad/s** (replay raw angular units ×0.01). The old altitude, freshness, simulated-contact, and jump/dodge gates still apply. This is an off-by-default ablation. The threshold came from inspecting train packets near the apparent 5.5 rad/s ceiling; it is an empirical regime marker, not proof of player input or engine physics. The hold still overwrites angular velocity after RocketSim integrates the preceding interval's orientation.

An initial 5.0 rad/s gate was checked on validation, then revised using train packet sequences to 5.48. Validation was therefore used twice during this development step; the final test split remains sealed. At train replay `00a0da63`, the angular speed at the last pre-mask packet was 5.075 rad/s for actor 8 at frame 204 and 5.284 for actor 30 at frame 1704; both had fallen from 5.5 in earlier packets. The gate now leaves their default errors at 1.12° and 2.02° instead of the ungated hold's 8.05° and 18.53°. Actor 149 at frame 11404 remained at 5.5 rad/s and retains the hold's improvement from 28.19° to 11.88°. Counterexample: `00b3d382` actor 137 at frame 6704 also had three prior packets at 5.5 rad/s, but zero recorded steer and subsequently falling angular velocity; the gate still applies the harmful hold (default 0.56°, hold 12.83°). Prior packet saturation does not reveal whether an unobserved pitch/roll input was released at the boundary.

Four-frame masked car errors on matched samples (p50 / p90 / p99):

| Split | Setting | Rotation ° | Angular rad/s | Position UU |
| --- | --- | --- | --- | --- |
| train | default | 3.251 / 20.859 / 46.592 | 0.579 / 3.468 / 6.382 | 16.776 / 40.259 / 75.562 |
| train | ungated hold | 3.306 / 18.913 / 41.514 | 0.580 / 3.196 / 6.207 | 16.776 / 40.269 / 75.654 |
| train | speed gated | 3.217 / 18.868 / 41.639 | 0.558 / 3.196 / 6.207 | 16.776 / 40.259 / 75.562 |
| validation | default | 3.078 / 20.512 / 47.163 | 0.579 / 3.521 / 6.399 | 15.967 / 39.609 / 76.639 |
| validation | ungated hold | 3.129 / 18.741 / 42.880 | 0.568 / 3.233 / 6.188 | 15.973 / 39.594 / 76.668 |
| validation | speed gated | 3.056 / 18.760 / 42.923 | 0.548 / 3.245 / 6.190 | 15.973 / 39.607 / 76.668 |

All three settings converted 60/60 train and 60/60 validation replays with zero failures and identical replay SHA-256 sets and sample counts. Horizon four contains 10,053 train and 10,401 validation rotation/position samples, and 10,049/10,398 angular samples. The gated option improved validation rotation p50 and p90 in each game size: 1v1 4.142/25.239→4.101/22.552°, 2v2 3.028/20.812→3.007/19.051°, and 3v3 2.899/18.923→2.866/17.305°. Its horizon-four replay rotation p50 improved/worsened/tied in 27/7/26 train and 25/6/29 validation replays; p90 improved in 56/60 train and 54/60 validation with no regressions. The largest replay-median rotation regression was 0.641° on train and 0.287° on validation. Validation angular p50 improved/worsened/tied in 41/7/12 replays; angular p90 improved/worsened/tied in 52/1/7. The option also improved pooled one-step validation rotation p50/p90/p99 from 1.855/8.750/21.954° to 1.845/8.282/20.171°.

The gate is promising but remains off by default because it still fails a known train decay window, regresses some replay medians, and leaves a post-step orientation/angular-velocity mismatch. Next work should infer whether an orientation-consistent control or prior-trend model can capture sustained spin without that mismatch. Reproduce the reports with `cargo run --release --bin evaluate_corpus -- replays/train target/train-low-air-cap-gated.json --gated-low-air-angular` and the same command with `replays/validation`; omit the flag for baseline and use `--hold-low-air-angular` for ungated hold. For the paired train feature analysis, run the baseline and ungated hold evaluator commands with distinct `--rotation-trace target/train-base-trace.jsonl` and `--rotation-trace target/train-hold-trace.jsonl` paths, then `python python/analyze_low_air_regimes.py target/train-base-trace.jsonl target/train-hold-trace.jsonl`. Trace rows include up to three fresh angular packets strictly before each mask window. All generated files stay ignored under `target/`.

## Masked rotation frame inspection (train only, 2026-09-28)

The evaluator now accepts one `.replay` or a split directory and optionally writes `--rotation-trace <path.jsonl>`. It emits one row per masked primary-linked car frame from the evaluator's existing four-frame schedule. Rows include both the fresh original body and the body supplied to the converter after masking, each field's source frame, original controls at the target and preceding frame, the previous and current simulated car states, and simulator events for the target interval. `previous_simulated.controls_for_next_interval` drove the **preceding** interval; `predicted.controls_for_next_interval` is the control selected for the **next** interval. Event labels beginning `sim_` are RocketSim estimates, not replay-authored contacts. `hold_rotation_error_degrees` is the evaluator's hold-last-observed-rotation baseline; the low-air angular-hold ablation requires its own run. Rotation error fields are null when the target has no fresh eligible rotation packet.

I inspected three train replays frame by frame under the default converter and the off-by-default `--hold-low-air-angular` ablation; the first was also inspected under `--feedback-low-air-angular`. Trace rotation-error keys and existing metric sample counts matched exactly: replay `00a0da63-492e-4ab7-8a07-16cd5d14dcb4` had 1,080 trace rows and 354 eligible rotation samples, `00b3d382-c4f8-403c-a166-5a7afcd1dd90` had 1,034/342, and `00a02f8f-83f1-4307-9005-3948a9153c0a` had 928/382. A final rerun of the first replay also matched all 353 eligible angular samples. Baseline and hold had identical sample keys in all three; no traced actor-lifetime mismatch was found. These are selected diagnostic windows, not a corpus-level performance estimate.

| Train replay / masked actor frames | Evidence in the packet and simulated interval | Default at last frame: rotation / angular error | Angular hold at last frame: rotation / angular error |
| --- | --- | ---: | ---: |
| `00a0da63` actor 8, frames 201–204 | Prior rotation packet at 200; replay steer near +0.79, yaw control +0.79, no simulated contact; height 68 UU | 1.12° / 0.29 rad/s | 8.05° / 2.70 rad/s |
| `00b3d382` actor 137, frames 6701–6704 | Prior angular packet at 6699; all controls zero, no simulated contact; observed angular Y falls from 5.28 to 2.41 rad/s by frame 6704 | 0.56° / 0.20 rad/s | 12.83° / 2.12 rad/s |
| `00a0da63` actor 149, frames 11401–11404 | Prior packet at 11398; replay steer about −0.83 to −0.93, no simulated contact; observed angular X remains high at 4.50 rad/s at frame 11404 | 28.19° / 3.68 rad/s | 11.88° / 1.59 rad/s |

The first window briefly favors hold at frame 202 (rotation 1.20° versus default 1.82°), then strongly favors default by frame 204. Feedback control also failed there: its inferred pitch −0.918 and clipped roll +1.0 preceded frame 204, which ended at 6.86° rotation and 2.09 rad/s angular error. A second known-steer no-contact interval in the same replay, actor 30 at frame 1704, ended at default 2.02° versus hold 18.53° and feedback 15.32°. The second table row isolates natural angular decay with zero controls; RocketSim tracks its observed decline. The third is a counterexample: default damps an angular component the fresh replay packet shows staying high. Missing pitch/roll or an unobserved impulse is plausible, but neither can be inferred uniquely from these traces. The traces also do not establish real collision absence because their contact labels come from the simulator. These mixed regimes explain why a blanket hold can improve tails while damaging typical masked rotation. Keep both low-air options disabled until a discriminator using only information available before the target packet improves matched validation behavior.

Reproduce a focused trace from the repository root with `cargo run --release --bin evaluate_corpus -- replays/train/1v1/00a0da63-492e-4ab7-8a07-16cd5d14dcb4.replay target/trace-00a0da63-report.json --rotation-trace target/trace-00a0da63.jsonl`. Add `--hold-low-air-angular` or `--feedback-low-air-angular` and use distinct output paths to compare the same masked frames. Both outputs are ignored under `target/`. Inspect `frame`, `actor_id`, `horizon`, field source frames, previous interval controls, and errors together; pooling alone obscures the reversal within a four-frame window.

## Low-air control and packet-time follow-up (2026-09-28)

The previous low-air angular hold changes angular velocity **after** RocketSim integrates orientation for that interval. That creates an inconsistent immediate state and is a plausible contributor to its masked rotation-median regression. On train, its four-frame masked rotation p50/p90/p99 changed 3.251/20.859/46.592→3.306/18.913/41.515 degrees, with p50 worse in 33/60 replays. Angular p50/p90/p99 changed 0.579/3.468/6.382→0.580/3.196/6.207 rad/s. The p50 rotation regression occurred in every game size; train examples `00a0da63` and `00b3d382` had +1.109° and +1.002° replay-median regression despite p90 gains. The original hold remains disabled.

`diagnose_air_rotation --low-air` now applies its fresh adjacent-packet, unchanged actor, active-phase, and no-fresh-dodge filters to cars whose **both** observed heights are 50–100 UU. It processed 60/60 train and validation replays, retaining 20,737/21,216 angular pairs. For nominal four-tick gaps, the replay orientation change projected onto mean angular velocity has p50 scale 0.637/0.610 on train/validation; the corresponding translation scale has p50 0.500/0.500. On the subset with a usable target position and velocity at both ends, integrating mean angular velocity over the full replay-frame delta yields rotation p50/p90 3.721°/10.743° on 19,424 train pairs and 3.826°/10.999° on 19,455 validation pairs. Replacing that duration with the **target-position-derived** interval yields 0.826°/4.040° and 0.727°/4.018° respectively. This is a cross-field offline consistency check: the target position and velocity choose the interval, so it is not a masked prediction or an observed packet timestamp. The low-air scales vary widely (four-tick orientation-scale p90 2.072/2.102), making a uniform half-time correction unsound.

To test a consistent simulator update, `--feedback-low-air-angular` is an off-by-default control ablation. It never overwrites angular velocity after a step. At an active low-air frame with a same-lifetime previous car, a position and angular observation no older than 0.15 s, airborne contact-free RocketSim state, no recent odd jump/dodge component packet, and no simulated car collision in the interval, it compares simulated pitch/roll angular velocity with the last available replay angular packet. Only when the pitch/roll difference exceeds 0.75 rad/s does it solve bounded RocketSim pitch/roll controls to approach that packet over a fixed four-tick response interval; it preserves the existing steer-to-yaw rule. Current packets are used only to set controls for *subsequent* ticks. The 0.75 threshold and four-tick response were fixed before validation. Other default controls and the 120 Hz timeline are unchanged.

| Split | Horizon-four field | Matched samples | Default p50 / p90 / p99 | Feedback p50 / p90 / p99 | Replay p50 better / worse / tied |
| --- | --- | ---: | ---: | ---: | ---: |
| train | Angular velocity (rad/s) | 10,049 | 0.579 / 3.468 / 6.382 | 0.601 / 3.393 / 6.394 | 10 / 36 / 14 |
| train | Rotation (degrees) | 10,053 | 3.251 / 20.859 / 46.592 | 3.293 / 20.237 / 46.428 | 17 / 29 / 14 |
| validation | Angular velocity (rad/s) | 10,398 | 0.579 / 3.521 / 6.399 | 0.598 / 3.437 / 6.384 | 10 / 34 / 16 |
| validation | Rotation (degrees) | 10,401 | 3.078 / 20.512 / 47.163 | 3.110 / 19.985 / 46.843 | 14 / 24 / 22 |

Validation horizon-four angular p50 rose in 1v1/2v2/3v3 from 0.893/0.577/0.519 to 0.916/0.598/0.532 rad/s; rotation p50 rose from 4.142/3.028/2.899 to 4.250/3.065/2.916 degrees. Rotation p90 improved in 37/60 validation replays, with none worse and 23 ties, but the median regressions fail the typical-plus-tail gate. Four-frame car-position p50/p90 stayed 15.967/39.609 UU; p99 moved 76.639→76.668 UU. All 60 replays converted successfully in each setting and split, with matched replay SHA-256 values and metric sample counts. The feedback option remains disabled. A physical control can improve an angular tail without matching the replay's apparent car motion interval; the packet-time interpretation remains a hypothesis.

Reproduce ignored reports with `cargo run --release --bin diagnose_air_rotation -- replays/train --low-air > target/train-low-air-motion.txt` and the analogous validation command; compare `cargo run --release --bin evaluate_corpus -- replays/train target/train-low-air-hold-control.json` with `cargo run --release --bin evaluate_corpus -- replays/train target/train-low-air-feedback.json --feedback-low-air-angular`, then substitute `validation` for the fixed check. Do not use the target-position-derived interval as a causal prediction score. `replays/test` remains sealed.

## Low-air contact, cadence, and angular-hold follow-up (2026-09-28)

The replay pad-pickup label requires an odd, non-255 `picked_up` counter and the matching instigator actor; `255` availability updates are excluded.

`diagnose_low_air` groups the existing fresh, one-step, pre-correction car angular-velocity pairs when the **target** altitude is 50–100 UU. It compares RocketSim to holding the previous fresh angular packet, with replay angular velocity scaled by 0.01. Every context is a marginal label over the same samples, so groups overlap. Packet gaps are raw replay frame and time gaps between fresh angular fields, **not** inferred simulator ticks. Prior geometry uses the previous synchronized RocketSim snapshot; `recent_sim_*` covers the preceding 0.15 s including the target interval and is an offline diagnostic. Replay pad pickups and jump/dodge component packets are labeled separately. No replay-authored contact field exists. The report includes per-replay quantiles, hashes, and the 100 worst angular regrets. `persistent_low_air` means prior carried position and current target are in the band, the predicted car is airborne, and no odd jump/dodge component packet occurred in the preceding 0.15 s; this is descriptive rather than contact proof.

Train and validation each converted 60/60 with zero failures. The same 142,390 train and 148,464 validation angular pairs as the existing altitude diagnostic were retained. Train RocketSim p50/p90/p99 was 2.013/4.616/6.638 rad/s versus hold 0.688/2.789/5.679; validation was 1.999/4.661/6.785 versus hold 0.671/2.804/5.650. RocketSim median exceeded hold in **60/60 replays in each split**. Persistent low air accounted for 97,873 train and 101,851 validation pairs. In validation that group had RocketSim p50/p90 2.063/4.641 versus hold 0.453/1.656. Thus the typical discrepancy is broad and persists away from a fresh takeoff signal. The diagnostic does not establish whether missing pitch/roll input, aerodynamics, packet timing, or an unreported contact dominates.

Contact and proximity enrich the tail but do not explain the majority of samples. On validation, previous RocketSim world contact appeared in 2,502/148,464 pairs (sim p90/p99 6.026/8.507 versus 4.641/6.670 without), and a recent simulated car-world hit appeared in 12,473 pairs (5.436/8.224 versus 4.581/6.506). A previous simulated ball distance below 350 UU appeared in 10,524 pairs (sim p99 12.401 versus 6.487 farther away). Previous simulated pad proximity below 250 UU appeared in 19,968 pairs, but its p99 was 6.484 versus 6.833 farther away. These are correlated simulation labels, not attributed replay collisions or causal effects.

An experimental `--hold-low-air-angular` converter option carries the last fresh angular packet across a low-air, airborne interval only when the prior observed and current predicted heights are both 50–100 UU, both prior position and angular fields are at most 0.15 s old, the predicted car has no wheel/world contact, no car contact event occurs in the simulated interval, and no fresh odd jump/dodge packet occurs in the prior 0.15 s. It reads no target rigid-body field; it changes the post-step angular state while leaving that interval's orientation as simulated. It is **off by default**. The intended test was whether a causal hold could improve both typical and tail masked rotation/angular errors without material per-replay regressions.

On validation, one-step transition angular p50/p90/p99 changed from 1.999/4.661/6.785 to 0.673/3.132/6.093 rad/s. Four-frame masked angular p50/p90/p99 changed from 0.579/3.521/6.399 to 0.568/3.233/6.188; rotation changed from 3.078/20.512/47.163 to **3.129/18.741/42.880** degrees. Masked rotation p50 worsened at horizons 2–4 and in every game size at horizon four: 1v1 4.142→4.231, 2v2 3.028→3.077, 3v3 2.899→2.920 degrees. At horizon four, replay rotation p50 improved/worsened/tied in 13/33/14 replays; p90 improved/worsened/tied in 54/0/6. Angular p50 improved/worsened/tied in 27/25/8; p90 in 53/2/5. One-step rotation p50 worsened in 40/60 replays. Four-frame masked car position p50 shifted 15.967→15.973 UU and p99 76.639→76.668 UU. The 60 validation replay SHA-256 values and sample counts matched between baseline and ablation; both converted 60/60 with no failures. On train, four-frame masked angular p50 changed 0.5793→0.5796 and rotation p50 3.251→3.306 degrees, despite better p90 tails. The candidate fails the typical-plus-tail gate and remains disabled. The post-step hold also creates a temporary mismatch between angular velocity and the orientation just integrated, which is a plausible source of rotation regression, not a proven cause.

Reproduce ignored reports with `cargo run --release --bin diagnose_low_air -- replays/train target/train-low-air-context.json` (replace `train` with `validation` for the frozen check), and `cargo run --release --bin evaluate_corpus -- replays/validation target/validation-low-air-hold-control.json` versus the same command with `target/validation-low-air-hold-ablation.json --hold-low-air-angular`. The evaluator now includes masked kinematics by horizon in each per-replay record. Train ablation: `cargo run --release --bin evaluate_corpus -- replays/train target/train-low-air-hold-ablation.json --hold-low-air-angular`. All reports remain ignored under `target/`. The `test` split remains sealed.

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
2. **Small boost-pad pickups (p90):** Across nearly every horizon and game-size slice, RocketSim p90 error clusters tightly around **12.16 boost units**. A small boost pad provides 31 raw boost ticks ($31 \times 100 / 255 \approx 12.156863$ boost units), so missed or spurious pickups are a plausible contributor. The quantile alone cannot attribute individual errors; trajectory divergence, pad cooldown, and other events require event-level checks.
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

`measure_air_fidelity` samples all 60 replays per split, matches each primary replay car to its RocketSim slot, requires the same car lifetime and fresh airborne endpoints without a dodge activation, and uses an isolated one-car simulation from each starting packet. This directly tests one-step control inversion with the future endpoint available. It does not include full match collisions or test causal prediction. These figures share the matched sample in the three-way comparison below.

| Dataset | Metric | Without Lookahead (Baseline) | With Lookahead (`infer_air_controls_from_lookahead`) | Improvement |
| --- | --- | ---: | ---: | ---: |
| **Train (51,045 pairs)** | Angular velocity p50 / p90 (rad/s) | 0.507 / 1.150 | **0.024 / 0.670** | Lower angular error |
| Train | Rotation p50 / p90 (deg) | **2.739 / 6.999** | 3.041 / 7.582 | Higher rotation error |
| **Validation (53,458 pairs)** | Angular velocity p50 / p90 (rad/s) | 0.449 / 1.123 | **0.028 / 0.672** | Lower angular error |
| Validation | Rotation p50 / p90 (deg) | **2.731 / 7.111** | 3.011 / 7.755 | Higher rotation error |

### Masked evaluation and leakage prevention

For the current four-frame mask in `evaluate_corpus.rs`, the next fresh angular packet is unavailable to the lookahead solver at the immediate masked boundary:
- Within a masked interval, `masked_observations` carries forward the prior rigid-body value and its original frame provenance. The lookahead solver requires a fresh next-frame packet, so it falls back to inferred aerial steering at that boundary.
- The converter also requires active play at both endpoints, the same car actor lifetime and owner, fresh positions above 100 UU, and no fresh next-frame dodge activation. These guards reduced unmasked angular fidelity in some cases but avoid inferring air controls from a contact, respawn, or play-state transition.
- A 60-replay validation ablation against `--no-infer-air-lookahead` produced identical masked car-position counts and p50/p90/p99 at horizons 1–4. Unmasked one-step car angular velocity p50 improved from 0.3985 to 0.3664 rad/s with lookahead; p90 improved from 2.5413 to 2.5277. Unmasked car-position quantiles changed by at most 0.001 UU. This establishes equality for the current four-frame mask, not for every possible masking protocol.

Enabled by default (`infer_air_controls_from_lookahead = true`), and ablatable via `--no-infer-air-lookahead` and `--infer-air-lookahead`.


## RLCarInputSolver comparison (supplied C++ code)

`external/RLCarInputSolver` estimates controls from two states. Its `Framework.h` expects a separate C++ RocketSim checkout, so the full ground solver is not directly executable against this project's native Rust RocketSim build. The replay stream directly supplies throttle, steer, and handbrake; it also supplies boost amount and boost/jump/dodge component counters. The C++ solver infers throttle, steer, handbrake, boost activation, and jump from motion. Those estimates may help when packets are absent, but they are not independent ground truth for recorded controls.

I ported the aerial orientation formula from `AirSolver.cpp` into `measure_air_fidelity` for a controlled comparison. All three columns use the same primary car pairs: active play at both ends, unchanged actor lifetime and player, fresh position/rotation/angular velocity at both ends, altitude above 100 UU, $0 < \Delta t \le 0.05$s, and no fresh dodge activation. Each pair starts a single Octane car from the replay state, applies controls for the elapsed RocketSim ticks, and compares the result with the next replay packet. “Replay-only” uses observed controls plus the project's steer-to-yaw fallback; “current inverse” uses the converter's offline aerial estimate; “RLCarInputSolver” replaces its pitch/yaw/roll with the supplied formula, including its deadzones and angular-speed correction. The full C++ boost, jump, flip, handbrake, and ground-steer heuristics are not tested here.

| Split | Pairs | Aerial controls | Angular velocity p50 / p90 (rad/s) | Rotation p50 / p90 (deg) |
| --- | ---: | --- | ---: | ---: |
| Train | 51,045 | Replay-only | 0.507 / 1.150 | **2.739 / 6.999** |
| Train | 51,045 | Current inverse | **0.024 / 0.670** | 3.041 / **7.582** |
| Train | 51,045 | RLCarInputSolver inverse | 0.113 / 0.673 | 3.041 / 7.630 |
| Validation | 53,458 | Replay-only | 0.449 / 1.123 | **2.731 / 7.111** |
| Validation | 53,458 | Current inverse | **0.028 / 0.672** | **3.011 / 7.755** |
| Validation | 53,458 | RLCarInputSolver inverse | 0.111 / 0.676 | 3.012 / 7.816 |

The supplied aerial formula improves angular-velocity fit over replay-only steering, but the current RocketSim-specific inverse is better at p50 and p90 on both splits. Neither inferred method improves isolated rotation error. Because both formulas use the next state, the low angular error is an offline fit, not evidence that they recover the player's actual stick input or improve masked forward prediction. Keep recorded replay controls as observations; use inverse estimates only for missing channels with provenance, and do not replace the current aerial formula on these results.

## Aerial rotation and replay-packet timing diagnosis

The inverse controls greatly reduce isolated next-packet angular-velocity error but increase orientation error. `src/bin/diagnose_air_rotation.rs` tests whether adjacent observed positions, orientations, and velocities describe motion over the elapsed replay-frame time. It uses active, unchanged primary car lifetimes with fresh fields at both ends, airborne positions above 100 UU, no fresh dodge activation, and $0 < \Delta t \le 0.05$s. For each pair, it projects observed displacement onto mean linear velocity and observed quaternion rotation onto mean world angular velocity. A scale of 1 means motion consistent with the full replay timestamp gap; a scale of 0.5 means the observed motion is half that implied by the reported velocity over that gap. The projection is a diagnostic, not a verified packet timestamp.

| Split | Nominal four-tick car translation scale p50 | Car rotation scale p50 | Ball translation scale p50 | Same-frame car/car scale difference p50 |
| --- | ---: | ---: | ---: | ---: |
| Train | 0.500 (47,431 pairs) | 0.511 (44,749 pairs) | 0.999 (448,308 pairs) | 0.001 (95,314 comparisons) |
| Validation | 0.500 (45,507 pairs) | 0.507 (42,801 pairs) | 0.997 (425,566 pairs) | 0.001 (89,268 comparisons) |

The nominal frame gap is usually four RocketSim ticks. Individual replays contain other projected factors, including approximately 1.0 and 1.75; the tool reports per-replay medians. `RecordFPS` is 30 in the inspected examples, and the match clock decrements about once per replay second even where car motion scale is 0.5. Ball packets track the nominal time much more closely. This points to car-specific packet or field timing/velocity semantics, but does not yet distinguish them from replication interpolation or another encoding rule. Applying a uniform half-time correction would hurt some replay intervals.

As a cross-field check, the tool estimates an effective interval from fresh car position and velocity, then integrates the *same pair's* mean angular velocity for that interval without using its target orientation to choose the interval. On pairs with a finite position-derived scale from 0.25 to 2.5:

| Split | Pairs | Full replay-gap rotation error p50 / p90 | Position-scaled rotation error p50 / p90 | Angular versus translation scale difference p50 / p90 |
| --- | ---: | ---: | ---: | ---: |
| Train | 47,227 | 2.904° / 6.726° | **0.105° / 1.549°** | 0.014 / 0.166 (44,456 pairs) |
| Validation | 48,646 | 2.881° / 6.798° | **0.097° / 1.560°** | 0.015 / 0.170 (45,687 pairs) |

This strongly links the orientation regression to a mismatch between the replay-frame gap and car motion implied by packet velocities. The diagnostic uses the next position and velocity, so it is **offline** and cannot be cited as causal masked-prediction improvement. It also does not establish the physical cause or justify changing the converter's 120 Hz replay timeline. The next experiment is to inspect raw actor update cadence, establish a packet-time model, and validate an explicit offline correction across all relevant state fields before enabling it.

The isolated RocketSim test also depends on its initial hidden car state. With `measure_air_fidelity --preserve-car-state`, the validation replay-only/current-inverse rotation p50 changes from 2.930° to 3.077° (the default isolated setup gives 2.731° to 3.011°), while p90 changes from 8.714° to 8.213°. Retaining simulated jump/flip flags changes the absolute errors and narrows the median regression, but does not remove it. This option is diagnostic; the converter was not changed.

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

## Offline car motion interval diagnostic (2026-09-28)

The earlier aerial analysis found that, on adjacent fresh airborne car packets, translation and rotation often imply about half the interval between replay frame timestamps when compared with their reported velocities. The ball usually implies the full interval. See the preceding **Aerial rotation and replay-packet timing diagnosis** for paired train and validation measurements. This does not identify the source of the discrepancy.

Commit `b2965e9` added `estimate_car_packet_interval`. Given two car positions and their linear velocities, it projects displacement onto mean velocity:

`t_projected = (p1 - p0) · mean(v0, v1) / |mean(v0, v1)|²`

The implementation rounds this result to a multiple of 1/120 s and attaches it to each eligible pre-correction car residual as `offline_interval`. It now returns no estimate when mean speed is below 100 UU/s, the fields are invalid, or the starting velocity and position came from different replay frames. The rounded count is a **motion-derived hypothesis**, not an observed server or client physics tick count. The replay frame timestamp and RocketSim's 120 Hz simulation schedule are unchanged.

The `offline_projection_fit` metric measures how closely the starting velocity times that fitted interval reaches the second position. **It is an in-sample fit:** the second position was already used to choose the interval, and the second velocity also enters the estimate. Consequently, this metric must not be compared with causal extrapolation as a prediction gain. The 5× and 225× position improvements claimed in the original branch commit and section do not establish better reconstruction or prediction. The sample sets also differ. We retain the fit only as a diagnostic of how well a quantized scalar interval describes observed translation.

There is independent, but narrower, cross-field evidence: the preceding diagnostic uses the position-derived interval to integrate angular velocity and evaluates against the *unused* target orientation. On its selected airborne pairs, validation median orientation error fell from 2.881° to 0.097°. That supports a shared translation/rotation timing or velocity-semantics effect on those pairs. It still uses future data, is not a causal test, and does not prove that the inferred tick count is the actual packet age.

The prior branch reported modes at 2 and 7 rounded ticks, but its raw actor-cadence analysis and isolated RocketSim experiment were not committed as reproducible source or reports. Those modes should be treated as **inferred motion intervals**, not raw packet timestamps. The following section measures actual per-actor update gaps and compares projected intervals by gap length, replay, and contact state. Any correction to the converter still needs independent validation, with the original replay timeline preserved.

## Raw actor cadence and frozen gap-scale check (2026-09-28)

`audit_packet_timing` counts consecutive fresh car position updates from the same primary linked actor lifetime during continuous `Active` play. It counts ball updates separately. These are **observed replay-frame gaps**, not inferred physics ticks. The ignored reports can be reproduced with:

```powershell
cargo run --release --bin audit_packet_timing -- replays/train target/train-packet-timing.json
cargo run --release --bin audit_packet_timing -- replays/validation target/validation-packet-timing.json target/train-packet-timing.json
```

| Split | Size | Car gap 1 / 2 / 3 / 4+ replay frames | Ball gap 1 frame |
| --- | --- | ---: | ---: |
| Train | 1v1 | 13,236 / 83,463 / 60,601 / 752 | 159,862 |
| Train | 2v2 | 53,861 / 122,466 / 78,094 / 30,908 | 135,325 |
| Train | 3v3 | 109,358 / 274,715 / 144,983 / 26,488 | 174,792 |
| Validation | 1v1 | 10,887 / 73,073 / 49,871 / 23,321 | 143,087 |
| Validation | 2v2 | 48,522 / 145,147 / 92,423 / 36,908 | 154,521 |
| Validation | 3v3 | 126,153 / 282,491 / 116,769 / 51,089 | 177,057 |

Two or three replay frames between car updates are common, so the converter often spans roughly 0.067-0.100 seconds (8-12 RocketSim ticks) before seeing a fresh car body. Ball updates usually appear every replay frame. Long car gaps are unevenly distributed: the largest train 4+ counts are 23,038, 15,432, and 14,351 in three individual replays. The raw-gap counts cover active primary cars, including known owners whose current link is inactive; an independent active-link-only train audit changed counts only slightly.

The same tool groups motion-derived intervals by raw gap. The training median **rounded interval / replay timestamp interval** is 0.50/0.75/0.50 for one-frame gaps in 1v1/2v2/3v3, about 1.125 for two-frame gaps, and 0.88-0.92 for three-frame gaps. Validation shows the same broad pattern, though its one-frame 2v2 median is 0.50. This explains why pooling all gap lengths obscures timing structure. These ratios are inferred from both endpoint positions and velocities; their rounded ticks are still not observed packet timestamps.

As a separate-field check, the position-derived interval is used to integrate angular velocity and compared with the unused target orientation. On validation one-frame airborne pairs, median orientation error falls from 3.406°/2.825°/3.094° using the nominal interval to 0.125°/0.097°/0.097° using the fitted interval for 1v1/2v2/3v3. This is an offline cross-field fit; target position and velocity are already known when the interval is chosen.

To test whether a timing scale transfers without using the *scored* endpoint position, the tool takes the median projected scale for each game size and raw gap of 1-3 frames from the **train** report, freezes those nine values, and scores simple start-velocity position and start-angular-velocity orientation extrapolations on **validation**. Each baseline and model value uses the same eligible pairs. The raw gap becomes known when the next packet arrives, so this is an offline packet-endpoint model, not a 120 Hz forward simulation policy.

| Validation size | Position pairs | Nominal / frozen-gap position p50; p90 (UU) | Air rotation pairs | Nominal / frozen-gap rotation p50; p90 (degrees) |
| --- | ---: | ---: | ---: | ---: |
| 1v1 | 133,831 | 18.99 / **11.94**; 44.61 / **33.73** | 36,047 | 4.04 / **3.50**; 11.09 / **10.58** |
| 2v2 | 286,091 | 18.25 / **13.09**; 45.39 / **37.21** | 79,153 | 3.56 / **3.09**; 10.01 / **9.37** |
| 3v3 | 525,412 | 18.87 / **12.97**; 45.05 / **38.22** | 148,491 | 3.33 / **2.76**; 9.43 / **9.15** |

Despite the pooled gains, **8 of 60 validation replays regress** in median position and median air rotation. The worst per-replay median position regression is 8.58 UU. A naive alternative that carries the previous pair's fitted scale forward is worse across all game sizes (validation pooled-by-size median position error rises from 17.56/17.34/17.85 to 28.46/28.95/31.63 UU). The gap-scale model is therefore a diagnostic candidate, not a converter correction. Replay-specific modes, long-gap cases, contacts, and masked RocketSim behavior need further work; `replays/test` remains sealed.

## Earlier-packet replay calibration (2026-09-28)

The eight validation replays where the frozen train gap scale worsened both median position and air rotation have mostly nominal-looking motion intervals. Their frozen-scale median position regressions range from 2.24 to 8.58 UU. This motivated a replay-specific selector in `audit_packet_timing`: for each raw gap of 1–3 frames, it starts with the frozen train scale and counts whether nominal or frozen start-velocity extrapolation wins on **completed earlier pairs in that replay**. After 64 scored pairs for that gap, it selects nominal time if nominal wins exceed frozen wins by 10%. It scores the current endpoint before adding its result to the history. This is an offline endpoint extrapolation, not a RocketSim or replay-tick correction. The selector was fixed before the validation run.

Reproduce the paired reports with:

```powershell
cargo run --release --bin audit_packet_timing -- replays/train target/train-packet-timing-adaptive.json target/train-packet-timing.json
cargo run --release --bin audit_packet_timing -- replays/validation target/validation-packet-timing-adaptive.json target/train-packet-timing.json
```

All 60 train and 60 validation replays parsed. The following validation numbers compare **the same eligible pairs**; columns are nominal / frozen train scale / earlier-packet selector, with position in UU and airborne orientation in degrees. The orientation endpoint is excluded from scale selection.

| Size | Position pairs | Position p50 / p90: nominal · frozen · selector | Air rotation pairs | Air rotation p50 / p90: nominal · frozen · selector |
| --- | ---: | --- | ---: | --- |
| 1v1 | 133,831 | 18.99 / 44.61 · 11.94 / 33.73 · 11.71 / 34.06 | 36,047 | 4.04 / 11.09 · 3.50 / 10.58 · 3.49 / 10.64 |
| 2v2 | 286,091 | 18.25 / 45.39 · 13.09 / 37.21 · 12.96 / 37.18 | 79,153 | 3.56 / 10.01 · 3.09 / 9.37 · 3.08 / 9.35 |
| 3v3 | 525,412 | 18.87 / 45.05 · 12.97 / 38.22 · 12.43 / 38.26 | 148,491 | 3.33 / 9.43 · 2.76 / 9.15 · 2.70 / 9.19 |

Relative to the frozen model, validation replay median position improves on 11 replays, worsens on 3, and is unchanged on 46; air rotation improves on 10, worsens on 2, and is unchanged on 48. The largest added regression versus frozen is 0.13 UU in replay median position. Relative to nominal time, **8/60 validation replay median positions and 7/60 air rotations still regress**. For the eight original position failures, the selector cuts the worst replay median regression from 8.58 to 1.05 UU, but does not eliminate it. The 1v1/3v3 pooled p90 position and rotation also rise slightly over frozen. Validation position p99 is 73.50/68.54/72.18 UU with nominal time, 89.61/75.71/95.57 UU with frozen scale, and 89.14/75.65/95.54 UU with the selector across 1v1/2v2/3v3. Train results have the same directional pooled median gains but are descriptive because the frozen scale was derived from train. This selector therefore remains diagnostic and is **not enabled in conversion**. A later timing correction needs paired masked RocketSim validation, contact and long-gap analysis, and no material per-replay regression; the 120 Hz replay timeline remains unchanged. `replays/test` remains sealed.

## Low-air transition diagnosis and control ablations (2026-09-28)

`evaluate_corpus` now subdivides field-fresh one-step car angular residuals whose **current observed center height is 50–100 UU**. Origin is the preceding observed position carried in the previous replay frame; `sim_ground`/`sim_air` is RocketSim's pre-correction ground flag. The event labels mean a fresh odd replay jump, double-jump, or dodge component packet within the preceding 0.15 s, searched with the same actor lifetime; they are packet proxies, not verified player inputs. The categories are separate marginal views, so their counts should not be added together. Each metric compares the same fresh angular packets with the pre-correction simulator and held previous observed angular velocity.

| Validation context | Paired samples | RocketSim / hold p50 (rad/s) |
| --- | ---: | ---: |
| Origin below 50 UU | 19,265 | 1.490 / 1.649 |
| Origin already 50–100 UU | 119,381 | 2.116 / 0.523 |
| Origin above 100 UU | 9,818 | 0.692 / 0.947 |
| RocketSim airborne | 139,221 | 2.088 / 0.634 |
| No recent jump or dodge packet | 121,847 | 1.770 / 0.545 |
| Recent jump packet | 6,045 | 0.840 / 0.882 |
| Recent dodge packet | 19,754 | 3.244 / 2.936 |

The corresponding train split has 114,598/142,390 samples already in the band, 133,217/142,390 simulated airborne, and 117,127/142,390 without a recent jump or dodge packet. Thus the large transition-band error is mostly **persistent low-air motion**, rather than a single takeoff impulse. This is an inference from the marginal groups; collision proximity and the exact replay input remain unobserved.

Two opt-in ablations were fixed on train and then checked on all 60 validation replays, with the default converter unchanged:

1. `--infer-transition-air-lookahead` lowers the existing offline inverse-control height guard from 100 to 50 UU while retaining its fresh adjacent endpoint, active-play, actor-lifetime, and dodge guards. Validation transition angular p50/p90/p99 changes from 1.999/4.661/6.785 to 1.986/4.643/6.783 rad/s. Overall one-step angular p50 improves in all 60 replays, but overall rotation p50 worsens in 42/60, and all four masked angular horizons are unchanged. The inverse uses the future angular endpoint; this result measures offline fit, not causal prediction.
2. `--compensate-transition-air-damping` uses current simulated orientation and angular velocity to solve RocketSim aerial controls for zero angular acceleration while airborne at 50–100 UU. It uses no future packet values. Validation transition angular p50/p90/p99 changes from 1.999/4.661/6.785 to **1.657/4.633/6.764** rad/s (train 2.013/4.616/6.638 to 1.668/4.589/6.616). Overall validation one-step angular p90 improves from 2.528 to 2.495 rad/s, and rotation p90 improves from 8.750 to 8.458 degrees; both p90 fields improve in all 60 validation replays. But rotation p50 rises from 1.855 to 1.873 degrees and worsens in 55/60 replay medians.

The causal damping ablation also has a consistent masked tradeoff at horizon four:

| Validation size | Angular p50 / p90: default → damping (rad/s) | Rotation p50 / p90: default → damping (degrees) |
| --- | --- | --- |
| 1v1 | 0.893 / 4.053 → 0.925 / 3.896 | 4.142 / 25.239 → 4.255 / 23.045 |
| 2v2 | 0.577 / 3.581 → 0.582 / 3.474 | 3.028 / 20.812 → 3.116 / 19.675 |
| 3v3 | 0.519 / 3.347 → 0.543 / 3.307 | 2.899 / 18.923 → 2.974 / 18.437 |

Both ablations remain **off by default** because neither improves typical and tail masked errors together. The damping controls are physically motivated simulator compensation, not recovered player inputs. The diagnostic does not establish that RocketSim damping is the only cause; raw car packet timing, wheel contact, and unobserved pitch/roll remain possible contributors. Reproduce ignored reports with `cargo run --release --bin evaluate_corpus -- replays/<split> target/<split>-takeoff-context.json` and the same command with `--infer-transition-air-lookahead` or `--compensate-transition-air-damping` and distinct output names. Train/validation each converted 60/60 under all three settings; `replays/test` remains sealed.

## Compact columnar format prototype (2026-09-28)

`python/replay_columnar.py` converts the Rust schema-v1 JSONL output in bounded batches to Arrow IPC (`.arrow`, zstd) or Parquet (`.parquet`, zstd level 3). Each replay frame has typed time/tick, ball, car, control, pad, score, and clock columns with the same dense shapes, NaN missing values, slot order, and masks as `load_numpy`. The full frame JSON is also retained as a compressed binary column, so typed access does not discard hidden RocketSim fields, replay observations/provenance, events, or residuals. The original header, dependency revisions, source hash, options, car slots, and diagnostics are stored in schema metadata. This is a **Python second stage after Rust JSONL**, not a direct Rust writer or a restored RocketSim arena.

The optional Python dependencies used here were Python 3.11.8, NumPy 2.4.4, and PyArrow 25.0.1 on Windows. PyArrow is pinned in `python/requirements-columnar.txt`. Reproduce one benchmark for each size with the named training replay, for example:

```powershell
python -m pip install --target target/pydeps -r python/requirements-columnar.txt
$env:PYTHONPATH = 'target/pydeps'
cargo run --release --bin convert_replay -- replays/train/1v1/0000a984-75af-4b24-b5a6-cb3663fc4efa.replay target/columnar-sample.jsonl
python python/benchmark_columnar.py target/columnar-sample.jsonl --repeats 3 --report target/columnar-benchmark.json
```

The same commands used `replays/train/2v2/000c0390-fc3c-4a68-8360-3979d9d88aaf.replay` and `replays/train/3v3/00054e5d-90dd-4e96-a922-bf28ae08513a.replay`, with `target/columnar-2v2.jsonl` and `target/columnar-3v3.jsonl`. The ignored reports are `target/columnar-benchmark.json`, `target/columnar-2v2-benchmark.json`, and `target/columnar-3v3-benchmark.json`. Source SHA-256 values are in those reports. The script regenerates both columnar files and gzip JSONL, times three complete dense-array loads for each format, and compares every NumPy array and **every complete rich frame** against the Rust JSONL. All three replays passed exact Python dictionary and NumPy array equality, including missing values. Synthetic tests also cover both formats and gzip JSONL.

| Train size | Frames | Raw JSONL MB / read s | gzip JSONL MB / read s | Arrow IPC MB / read s | Parquet MB / read s |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1v1 | 11,663 | 129.29 / 2.51 | **7.57** / 2.96 | 9.30 / 0.051 | 10.50 / **0.018** |
| 2v2 | 6,786 | 118.67 / 2.26 | **7.46** / 2.62 | 9.16 / 0.048 | 10.26 / **0.011** |
| 3v3 | 10,935 | 262.82 / 5.61 | **17.46** / 6.39 | 21.58 / 0.107 | 24.07 / **0.017** |

Sizes are decimal MB; read times are median seconds over three warm local reads of the same dense outputs, excluding conversion/write time. Parquet's projected loader skips the rich JSON column, whereas the prototype Arrow loader reads the whole IPC table before selecting columns; this difference is implementation-specific. Gzip JSONL is smallest, but still needs JSON parsing for dense arrays. On the 1v1 sample, a single rich-frame pass took 1.16 s JSONL, 1.35 s gzip, 1.13 s Arrow, and 1.22 s Parquet; rich access is therefore much closer across formats. Columnar write time was about 3.3–8.0 s across these samples, excluding Rust replay conversion. These are development machine measurements on one replay per size, with OS-cache effects; they do not establish corpus-wide throughput.

**Format decision for the next implementation step:** target Parquet for Python ML array access while retaining JSONL as the direct, inspectable Rust output until a native streaming writer exists. Arrow IPC remains available when its somewhat smaller files or batch semantics are preferable. The hybrid payload duplicates some typed values; a direct Rust Parquet writer, bounded-memory conversion, and a state-restoration check are still needed before declaring Phase 5 complete. `replays/test` remains sealed.

## Direct Rust Parquet export (2026-09-28)

`convert_replay` now writes `.parquet` directly from Rust using pinned Apache Arrow/Parquet 60.0.0 with zstd level 3. It preserves the Python prototype's columnar version 1 schema: final JSONL-equivalent header in metadata, first-frame pad configuration, typed state/action/score/clock columns, and a complete `frame_json` payload per row. A callback emits each corrected RocketSim snapshot and its residuals without keeping all snapshots in the Parquet path. Final car slots and conversion diagnostics are learned in a first simulation pass; a second pass writes 512-frame row groups. Both passes' complete car-slot metadata and diagnostics must agree. `boxcars` parsing, replay bytes, and the extracted observations remain replay-sized in memory because offline control inference can inspect future observations. This is bounded **snapshot/output** memory, not bounded total conversion memory. No peak-memory or throughput claim has yet been measured.

The reusable `python/verify_direct_parquet.py` check compares the complete header, every dense NumPy array (including NaN missing values), every rich frame dictionary, and the 512-row-group bound against a Rust JSONL export. Run, for example:

```powershell
cargo run --bin convert_replay -- replays/train/1v1/0000a984-75af-4b24-b5a6-cb3663fc4efa.replay target/direct-1v1.parquet
$env:PYTHONPATH = 'target/pydeps'
python python/verify_direct_parquet.py target/columnar-sample.jsonl target/direct-1v1.parquet
```

The checked training replays were the same source files and JSONL references as in the columnar prototype section above. The validation file was `replays/validation/1v1/1a3ac92c-961e-469b-9d1a-29d210a7b5d9.replay`, independently exported to both JSONL and Parquet with the same default options. All four comparisons passed:

| Split and size | Frames | Direct Parquet bytes | Header / arrays / rich frames |
| --- | ---: | ---: | --- |
| Train 1v1 | 11,663 | 10,541,260 | Exact |
| Train 2v2 | 6,786 | 10,324,582 | Exact |
| Train 3v3 | 10,935 | 24,304,160 | Exact |
| Validation 1v1 | 12,724 | 11,882,204 | Exact |

An initial parity failure found that promoting the replay's binary `f32` time directly to `f64` changed nearly every Python time value compared with reading JSONL's shortest decimal representation. The writer now parses that same decimal representation for the typed time column, and the full-array comparisons pass. `cargo test --all-targets` and Python loader tests pass. The format remains opt-in via the `.parquet` output extension. Restoring a serialized snapshot into a RocketSim arena, total peak-memory measurement, and wider validation checks remain open before making Parquet the default ML export. `replays/test` remains sealed.

## Soccar state restoration and output resource use (2026-09-28)

`src/restoration.rs` reads schema-v1 header slots and rich frame payloads, then rebuilds a detached native RocketSim `ArenaState` with the serialized arena tick, ball, car controls and timers, hitbox/team configuration, and boost-pad configuration/cooldowns. It rejects unsupported header revisions/modes, nonsequential or mismatched car slots, unknown hitboxes, and inconsistent pad activity. `verify_state_restoration` checked every field after JSON parse → native state → transfer record on four prior direct Parquet exports (train 1v1/2v2/3v3 and validation 1v1), plus the largest validation 3v3 replay by source-file size: **80,246/80,246 exact detached snapshot round trips**. It also seeded fresh live arenas at every 5,000th frame (19 samples); all public ball/car fields matched immediately, and maximum observed pad cooldown quantization error was 0 seconds. The validation 3v3 file was `replays/validation/3v3/1a0b10f9-c7b7-428a-b745-b5e7607a2d8a.replay` (38,138 frames). Reproduce with `cargo run --release --bin verify_state_restoration -- target/<export>.parquet` after direct conversion.

**Live continuation is not exact.** The pinned RocketSim API cannot set a live arena's absolute tick, RNG state, or private physics/contact/wheel caches. Applying a snapshot leaves the arena's own tick unchanged, and pad cooldown is translated through a rounded 120 Hz pickup tick. Soccar has no tile state; the current transfer schema omits Dropshot tile state and supports only the validated soccar path. A detached `ArenaState` faithfully represents the public snapshot, but stepping a newly seeded live arena is not guaranteed to reproduce the original future trajectory. This limitation is returned in `ArenaApplyReport` as source tick, live arena tick, and maximum pad cooldown error.

`python/benchmark_conversion.py` launched the same prebuilt release converter for paired JSONL and direct Parquet exports, with one warmup then three measured repetitions per format/replay and alternating format order. It recorded child-process wall time, Windows peak working set via psutil, frame count, and file size. The ignored raw reports with source hashes, executable hash, all repetitions, and machine details are `target/conversion-benchmark.json` and `target/conversion-benchmark-large.json`. Reproduce with `cargo build --release --bin convert_replay`, install `python/requirements-benchmark.txt`, then pass the paths listed below to the benchmark script; it rejects `replays/test` and removes generated outputs after measurement unless `--keep-files` is supplied.

```powershell
python python/benchmark_conversion.py replays/train/1v1/0000a984-75af-4b24-b5a6-cb3663fc4efa.replay replays/train/2v2/000c0390-fc3c-4a68-8360-3979d9d88aaf.replay replays/train/3v3/00054e5d-90dd-4e96-a922-bf28ae08513a.replay replays/validation/1v1/1a3ac92c-961e-469b-9d1a-29d210a7b5d9.replay replays/validation/2v2/1a00cbd5-38db-4309-a695-648f1b82792a.replay replays/validation/3v3/1a1c065f-dbfd-4c62-ac87-05f27b3c7d97.replay --report target/conversion-benchmark.json
python python/benchmark_conversion.py replays/train/3v3/001c4769-124f-4954-ba1f-45216cb7f165.replay replays/validation/3v3/1a0b10f9-c7b7-428a-b745-b5e7607a2d8a.replay --report target/conversion-benchmark-large.json
```

| Split / size / replay prefix | Frames | JSONL seconds / peak MB / output MB | Parquet seconds / peak MB / output MB |
| --- | ---: | ---: | ---: |
| Train 1v1 `0000a984` | 11,663 | 0.518 / 95.7 / 129.29 | 0.951 / 82.3 / 10.54 |
| Train 2v2 `000c0390` | 6,786 | 0.436 / 72.4 / 118.67 | 0.867 / 77.8 / 10.32 |
| Train 3v3 `00054e5d` | 10,935 | 0.929 / 137.4 / 262.82 | 1.812 / 119.2 / 24.30 |
| Validation 1v1 `1a3ac92c` | 12,724 | 0.560 / 103.2 / 147.00 | 1.053 / 87.6 / 11.88 |
| Validation 2v2 `1a00cbd5` | 14,137 | 0.889 / 136.8 / 250.05 | 1.770 / 104.8 / 22.05 |
| Validation 3v3 `1a1c065f` | 9,978 | 0.867 / 130.7 / 244.23 | 1.709 / 116.3 / 22.88 |
| Largest train 3v3 `001c4769` | 17,001 | 1.443 / 215.6 / 427.24 | 2.940 / 160.0 / 39.51 |
| Largest validation 3v3 `1a0b10f9` | 38,138 | 2.714 / 423.0 / 907.19 | 5.220 / 273.8 / 73.28 |

All figures are medians of three measured runs, decimal MB, on one Windows development machine. Parquet saves peak memory on seven of eight sampled replays, but **uses 5.4 MB more on the sampled train 2v2**. It takes about twice as long to export because it simulates the replay twice to discover final fixed-width car slots before writing; this is end-to-end export time, not a serializer-only comparison. Peak working set includes parsing, extracted observations, RocketSim, output buffers, and allocator behavior, but excludes filesystem cache and other processes. Cache, disk, and antivirus effects remain possible. The largest sampled Parquet peak is 273.8 MB; observations and input bytes still scale with replay size. A separate streaming parser/observation redesign is worthwhile for very large or highly concurrent workloads, but is not a prerequisite for recommending Parquet for ML reads of the supplied soccar corpus. JSONL remains useful for inspection and faster one-off exports. `replays/test` remains sealed.
