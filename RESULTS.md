# Reconstruction measurements

Last updated: 2026-09-29. These are development measurements, not a final accuracy claim. Current reviewed baseline machine-readable reports are `target/train-reviewed.json` and `target/validation-reviewed.json`; the latest optional low-air gate reports are `target/train-low-air-cap-gated.json` and `target/validation-low-air-cap-gated.json`. Packet timing reports are `target/train-packet-timing.json` and `target/validation-packet-timing.json`. Older experiment reports are retained under `target/*-conversion-metrics*.json`. Each evaluator report includes replay SHA-256 values, settings, errors, and per-game-size aggregates. No `test` replay has been opened or converted.

## Ground driving: the model is unbiased; control timing was the largest error (2026-09-29)

**Question.** Ground driving is the largest single car error partition (train error budget: ground without boost 24.7% of car position and 26.6% of angular-velocity squared error; velocity p50 8.3 UU/s, rotation p90 2.4 deg). Is that RocketSim's ground model, or the unobserved controls?

**Protocol** (`diagnose_ground_driving replays/train` and `replays/validation`; 394,101 and 420,114 flat-ground pairs). Consecutive fresh packets (a, b) of one car with chain-lag ticks (so the elapsed whole ticks k are exact), both flat on the ground (center z < 30 UU, up axis z > 0.97), jump/dodge counters unchanged and even, boost parity unchanged, active play throughout, no ball or other car within 400 UU at either end. RocketSim is started from packet a, with the handbrake ramp value replayed from the observed handbrake history (a packet state does not carry it; the converter's arena keeps it), and stepped k ticks with the observed controls; compared with packet b in the car's frame. Controls come from several hypotheses, some of which use a future observation (labelled): a's frame held (causal), b's frame held, linear interpolation from a to b, and the controls of every frame between them applied with a time shift.

**Findings (train; validation agrees to the third digit).**

1. **The ground model has no bias.** Signed errors in the car frame are centred on zero for every hypothesis and interval length (forward and lateral velocity, all three angular-velocity axes); intervals of 1-2 ticks are exact (velocity p50 0.6 UU/s, angular p50 0.009 rad/s) and pairs with the same controls at both ends have position p50 0.01 UU, velocity p50 0.0 UU/s, rotation p50 0.00 deg.
2. **The error is control timing.** Pairs whose controls changed between a and b (45%) carry most of it (position p50 0.87 vs 0.01 UU, angular p90 1.19 vs 0.35 rad/s for the a-held hypothesis). Applying each frame's controls at the frame's nominal tick (what a same-frame scheme does) is *worse* than holding a's controls (velocity p90 75.6 vs 72.6 UU/s, angular p90 0.921 vs 0.899). Scanning a time shift for the per-frame controls gives an optimum near -4 ticks (velocity p90 67.3, angular p90 0.659, rotation p90 2.20 deg at -4; -2: 70.7/0.785/2.34; +2: 80.7/0.939/2.72): a replicated change is first seen in the frame after it happened, and that frame's state is itself on average 2 ticks (half the 0-4 tick lag range) older than the frame time. The **midpoint rule** derived from that (switch at `T_g - 2 - gap/2`, no tuned constant) gives 67.3 UU/s, 0.662 rad/s, 2.21 deg, i.e. the scanned optimum. It uses the next frame's controls, so it is offline reconstruction, not causal prediction.
3. **The handbrake is a state.** RocketSim ramps a handbrake value (+5/s held, -2/s released). Starting every pair from zero (as a bare `set_car_state` does) inflates errors near handbrake use; replaying the observed history lowers, for the midpoint rule, velocity p90 67.3 to 59.6 UU/s, angular p90 0.662 to 0.610 rad/s and position p90 3.59 to 3.04 UU (unchanged-control pairs: velocity p90 39.3 to 30.4, angular 0.347 to 0.221). The converter already carries the value in its arena between packets. A handbrake-specific time shift (-24 to +16 ticks) has no optimum.

**Converter change.** `lookahead_ground_controls` (default on; `--no-lookahead-ground-controls` on `evaluate_corpus` and `error_budget`): for a car that is on the ground at the start of a frame interval, throttle, steer, handbrake and boost of the frame that ends the interval are applied from `gap/2 - 2` ticks into it (at least from its start) instead of from that frame's packet time. It needs the inferred packet lags (the rule is about physical ticks), is refused for a frame withheld by an evaluator, and only touches cars whose state is on the ground at the interval start, so aerial controls are unchanged. Unit test `lookahead_ground_controls_drive_the_interval_before_a_frame_and_respect_withholding`.

**Effect** (one-step pre-correction, chain-lag packets; per-partition budgets in `error_budget`; train / validation):

| Measure | Before | After |
| --- | --- | --- |
| Ground no boost, velocity p50/p90 UU/s | 8.3/71.8 / 8.3/72.1 | 7.3/64.6 / 7.2/64.1 |
| Ground no boost, angular p90 rad/s | 0.97 / 0.98 | 0.70 / 0.70 |
| Ground no boost, rotation p90 deg | 2.39 / 2.40 | 2.25 / 2.24 |
| All cars, velocity p90 UU/s | 75.9 / 76.1 | 71.4 / 70.8 |
| All cars, angular p90 rad/s | 0.93 / 0.93 | 0.74 / 0.73 |
| `evaluate_corpus` car velocity p50/p90/p99 | 4.421/85.897/560.812 / 4.474/86.233/550.573 | 4.151/81.493/560.650 / 4.151/81.102/550.539 |
| `evaluate_corpus` car rotation p90 deg | 3.291 / 3.268 | 3.238 / 3.198 |
| `evaluate_corpus` car angular p90 rad/s | 0.990 / 1.000 | 0.804 / 0.801 |
| `evaluate_corpus` ground angular p90 rad/s | 1.021 / 1.030 | 0.786 / 0.780 |

Per replay (60 each), p90 improved for angular velocity in 57 train and 58 validation replays, velocity in 43 and 46, rotation in 37 and 36; worse by more than 2%: angular 0 and 1, velocity 7 and 3, rotation 10 and 6 (largest about 5%, mostly rotation; the air partitions are unchanged to 0.1 UU). No partition of the error budget regresses materially (position p90 changes by at most 0.1 UU; ball near a car velocity p90 8.6 to 8.7). Non-aligned masked prediction is unchanged bit for bit (the option is off without packet lags, and never applies to withheld frames). With `--aligned-targets` the target is the offline reconstruction, which improved while the masked predictions did not, so horizon-4 car velocity p90 moves from 164.9 to 168.6 UU/s (train) and 151.7 to 153.9 (validation); this is a target change, not a prediction regression.

**What is left.** Hard steering with unchanged controls keeps a tail: sim minus true yaw rate times the steer direction has p90 exactly 0.000 and p10 -0.39 (-0.57 above 1,600 UU/s), so the real car turns more than the sim in about 17% of those pairs. Those pairs are already sliding (median lateral velocity 130-216 UU/s, yaw rate about 2.3-2.7 rad/s) and sit next to observed handbrake use (handbrake observed within 32 ticks, median 1) while the handbrake reads off inside the interval; forcing the handbrake on reproduces 7% of hard-steer pairs that the observed setting misses, and 10.7% remain unexplained by either. That points at short handbrake taps between frame samples (unobserved input, not a RocketSim error), which could be inferred like the flip cancel; unconfirmed. Recorded in PLAN.md as the next candidate. Reproduce: `diagnose_ground_driving replays/train` (add `--reset-handbrake-value` to start every pair from zero), `error_budget replays/train [--no-lookahead-ground-controls]`, `evaluate_corpus replays/<split> report.json --aligned-targets [--no-lookahead-ground-controls]`.

## Ball touches: the car state is not the limit; car/ball tick alignment is (2026-09-29)

**Question.** After the hit impulse was fixed upstream, 15-25% of real touches are still not reproduced. Is that car state error at the contact tick? Protocol (`diagnose_contact_fit`, `diagnose_contact_model`, train and validation, 1,746 and 1,750 touches): consecutive ball packets (A, B) whose velocity change at B exceeds a ball-only prediction by 50 UU/s, one car within 500 UU of the ball, a fresh chain-lag packet of that car at A's tick or one tick before it. Car and ball are set from their packets and stepped to B in a scratch arena; "matched" means RocketSim produced a `CarHitBall` and the ball velocity at B is within 50 UU/s of the packet. The fits below use the ball packet at B (a future observation, an offline fit, not a prediction).

**Correction to earlier numbers.** The diagnostic started a car packet one tick before A but stepped it only to B - 1, so the car ended a tick short. Fixed by stepping the car alone (ball parked far away) for that tick first. Corrected on train with no workaround (`diagnose_contact_model replays/train 1 --no-apply-hit-impulse`): 84.5% of touches produce a sim hit (was 78.1%, so 15.5% missed, not 22%); 89.4% when the car packet is at A's tick and 73.2% when it is one tick earlier (was 87% and 51%). Real vs sim hit impulse p50 990.3 vs 971.2 UU/s for sim hits, direction cosine 1.00; ball velocity error p50 18.0 UU/s.

**Results (train; validation in parentheses).**

| Measure | Value |
| --- | --- |
| Matched, measured timing | 65.1% (63.9%); 77.0% when the car packet is at A's tick, 37.2% when one tick earlier |
| Matched within a start-position shift grid (+-16 UU along travel, +-8 sideways, +-4 up) | 84.1% (83.5%), smallest shift p50/p90 0.0/5.7 UU (0.0/6.9) |
| Direction of the needed shift, in the car frame (n 323) | right and up centred on 0 (p50 -0.0 and 0.2 UU), forward p50 +1.0 (p10/p90 -4.0/9.7); no per-hitbox offset (octane 1,696 of 1,746 touches) |
| Car position error at the car's own next packet, unshifted vs contact-matched shift | p50 0.88 vs 1.44 UU; the shifted car is closer in only 4.8% |
| Sim NO hit (271): smallest hitbox-to-ball gap over the interval, no contact | p10/p50/p90 -6.6/2.2/99 UU (sim hits: p99 +1.4 UU) |
| Car-alone position drift over an ordinary packet interval | p50 4-5 UU at 9-11 ticks, p90 14-19 UU |

So the shift that makes a touch reproduce is about the size of ordinary drift over a packet interval, but it is not supported by the car's next packet (it worsens it) and has no systematic direction, so the converter does **not** nudge car positions to force contacts. Anchoring the car to its next packet (spreading the miss linearly) made contacts worse (matched 65.6% to 57.0% validation), but that test is confounded, since the car-alone miss at the next packet contains the real collision recoil; it says nothing either way and is kept only as a caveat. About 10% of sim misses show hitbox-ball overlap of more than 6 UU without a hit and a similar share are more than 99 UU apart at the closest approach (probably the touch was by another car or the recorded car is wrong); both remain unexplained. Near misses of a few UU are within one tick of relative motion (a car and ball close at up to 25 UU per tick).

**The one-tick puzzle: relative car/ball timing.** Re-running each touch with the car packet assumed delta ticks later or earlier than the chains say (delta -2..+2), matched touches on train (validation in parentheses):

| Car packet vs ball packet A | n | -2 | -1 | 0 (as measured) | +1 | +2 |
| --- | --- | --- | --- | --- | --- | --- |
| same tick | 1,224 (1,252) | 3.9 | 17.0 | **77.7** | 35.0 | 19.5 |
| one tick earlier | 522 (498) | 4.6 | 20.3 | 37.0 | **44.4** | 18.8 |

Same-tick pairs have a sharp optimum at the measured alignment. For one-tick-earlier pairs the optimum is one tick further (the car packet acts as two ticks early), and it is much weaker, so in roughly half of them the chains misplace the car relative to the ball by a tick. It is not a whole-chain constant: choosing a per-car-chain delta on half of a chain's touches and scoring on the other half lowers matches (71.6% to 69.1% on train, 68.7% to 65.8% validation, 129 and 122 chains), and same-frame packets do not share one tick (the one-tick-earlier group is 378 of 522 touches from the same frame as the ball packet, where "same tick" gets only 21.7%). The mismatch is per touch or per chain segment. A per-touch oracle (best delta for each touch, which uses the ball packet at B) reproduces 78.3% (78.1%) with delta -1..+1, against 65.5% (64.6%) as measured, so relative timing explains at most about 13 points of the missing 35%; the remaining ~22% is the contact model, the hitbox geometry at the tick, or the ball-side.

**Decision.** Nothing enabled. The car state at the contact tick is not what limits touch reproduction, since the unshifted car already reaches its next packet within about 1 UU for touches with short intervals. A converter feature would have to choose the car/ball tick alignment per touch from the ball packet (a discrete five-way fit that uses a future observation, provenance-labelled), and the ceiling is +13 points of matched touches on a ball error that is already small (ball position p90 0.8 UU, and the ball is re-injected at every ball packet), so it ranks below the car error sources (dodge window rotation, ground driving, jump velocity). Logged on the backlog. Reproduce: `diagnose_contact_fit replays/train` and `replays/validation`.

## Dodge start fit on the updated RocketSim: still off (2026-09-29)

**Question.** After the update, does fitting the dodge start tick (`--infer-dodge-start`) help, and does using more than one later packet fix its weakness? Protocol: `error_budget replays/{train,validation} [--infer-dodge-start]`, chain-lag packets, base is the default converter; nothing tuned on validation.

**Impulse model (`diagnose_dodge_fit`, 1,500 train events).** RocketSim's dodge impulse matches the real one: |I| p50 550 sim vs 531 real, ratio p10/p50/p90 0.68/1.00/1.16, direction cosine p10/p50 0.59/0.99 (1,327 forward, 135 backward dodges). So what limits the fit is timing and the unobserved pre-dodge orientation and pitch cancel, not the impulse. The fitted start minus the activation frame's tick has p10/p50/p90 -4/-1/8 ticks. At the next packet a fitted start beats the frame-time start in 1,038 of 1,288 events (position p50/p90 7.4/24.0 vs 19.2/46.8 UU, rotation 5.6/15.6 vs 10.9/23.9 deg), in-sample.

**Two variants in the converter.** (1) Fit against the first fresh packet only; (2) fit against up to three fresh packets in the 12 frames after the activation, position and 0.1 x velocity error summed over packets, angular velocity only counted at packets at or after the start, using exact packet ticks from the lag chains. A test that forced `jump=false` before the scheduled start changed nothing.

First packet after a dodge, position p50/p90 UU and velocity p50 UU/s (rotation p50 deg), base to three-packet fit, train (validation moves the same way):

| Previous z | Packets | Position | Velocity p50 | Rotation p50 |
| --- | --- | --- | --- | --- |
| >= 300 | 1,639 | 14.8/38.3 to 12.8/36.3 | 38.1 to 25.0 | 5.94 to 5.42 |
| 120-300 | 1,359 | 15.7/38.4 to 13.6/37.6 | 29.1 to 18.9 | 6.22 to 4.73 |
| 50-120 | 2,975 | 14.8/39.7 to 11.1/36.5 | 28.2 to 17.9 | 6.37 to 4.89 |
| < 50 | 7,588 | 16.7/48.0 to 14.7/46.7 | 36.6 to 26.8 | 6.21 to 5.12 |

**But the whole dodge window gets worse.** "Dodge counter odd" (90,621 train packets): rotation p90 7.45 to 7.77 deg, rotation squared-error share 0.313 to 0.369, angular velocity p90 0.66 to 0.88 rad/s; validation the same (0.313 to 0.361, 0.66 to 0.88). Position and velocity in that window improve slightly (p99 27.5 to 24.3 UU). The fit trades the first packet against later ones, so the option stays **off by default**. The low-altitude first packets (7,588, previous z < 50, the largest share of first-dodge error) barely move (position p50 16.7 to 14.7 UU) because the jump start, hold and dodge start are not fitted jointly and a ground packet gives no orientation. Not a usable improvement until later-window angular velocity stops regressing; candidate cause is the planned pitch cancel and roll persisting past the fitted packets.

## RocketSim update: 79f4d22 to 0b02051 (2026-09-29)

RocketSim `v3-rust` moved 141 commits (2026-08-26 to 2026-09-28, version 0.2.0 to 0.2.1). The dependency is now pinned to `0b020516c4fc633e0db09dfbfaa2026bcddb058e` (`Cargo.toml`, `Cargo.lock`, `serialization::ROCKETSIM_REVISION`). API changes handled in this repository: wheel contacts are `[Option<RaycastHitInfo>; 4]`, `last_extra_hit_tick` moved from the ball to each car (serialized per car, the ball record keeps a null field), and there is a new `CarLanded` event (serialized). The schema version stays 1, but exports made with the previous revision are rejected by the restoration check because the revision string changed. Unit tests (19), the Parquet/JSONL parity check and exact state restoration (12,485 and 13,290 snapshots) pass on the new revision.

**Re-check of the logged issues (details in `ROCKETSIM_NOTES.md`).** (1) The dropped ball-car hit impulse is **fixed upstream**: `on_hit` applies it after contact solving with `accum = false`. With no workaround, the 2,000 UU/s probe sends the ball out at 2,662 UU/s (1,531 before), the mutator scale 1, 2, 3 gives 2,662, 3,793, 4,925 UU/s, and against 1,746 real touches (`diagnose_contact_model`) the simulated hit impulse is 975.3 UU/s versus 990.3 real (cosine 0.99-1.00, median ball velocity error 11.4 UU/s, 78.1% before the 2026-09-29 diagnostic fix, 84.5% after, of touches produce a sim hit; the workaround on the old revision gave 1,001.5 vs 1,008.5, 13.8 UU/s and 75.9%). The workaround is now off by default (`apply_hit_extra_impulse`, `--apply-hit-impulse`) because it double counts on the new revision. (2) Speed limits applied at the start of the next tick **remain**: a flipping car still reports 7.45 rad/s, so `limit_reported_velocities` stays on. (3) Missed touches: 270 of 1,746 (15.5% after the 2026-09-29 diagnostic fix; 382 or 22% before it, see "Ball touches"). (4) False demolitions the converter corrected: 300 to 162 (train) and 282 to 140 (validation), consistent with upstream demo/bump cone gating. (5) API limits unchanged.

**Corpus effect** (60/60 replays each, zero failures; same defaults; hit-impulse workaround off on the new revision), one-step pre-correction, RocketSim p50 / p90 / p99:

| Split | Field | 79f4d22 | 0b02051 |
| --- | --- | --- | --- |
| train | car position UU | 0.222 / 5.486 / 37.420 | **0.127** / 5.401 / 37.055 |
| train | car velocity UU/s | 5.856 / 87.329 / 567.130 | **4.421** / 85.897 / 560.812 |
| train | car rotation deg | 0.467 / 3.418 / 11.802 | 0.431 / 3.291 / 11.282 |
| train | car angular rad/s | 0.118 / 1.005 / 3.554 | 0.113 / 0.990 / 3.443 |
| train | ball velocity UU/s | 0.010 / 0.014 / 467.473 | 0.010 / 0.014 / 446.713 |
| train | ball rotation deg | 0.000 / 0.028 / 4.184 | 0.000 / **0.000** / 4.097 |
| validation | car position UU | 0.221 / 5.510 / 37.418 | **0.125** / 5.428 / 37.066 |
| validation | car velocity UU/s | 5.825 / 87.714 / 557.815 | **4.474** / 86.233 / 550.573 |
| validation | car rotation deg | 0.459 / 3.400 / 11.754 | 0.423 / 3.268 / 11.118 |
| validation | ball velocity UU/s | 0.010 / 0.015 / 475.673 | 0.010 / 0.014 / 462.940 |

Budget (train, chain-lag packets): ball near a car velocity p90 15.2 to 8.6 UU/s (position p90 1.0 to 0.8); all-car velocity p50 5.3 to 3.3 UU/s, rotation p90/p99 3.14/10.7 to 3.01/10.2 deg. Masked prediction with aligned targets (validation): horizon 4 car position p50/p99 3.622/60.188 to 3.524/58.433 UU, car rotation p50/p90/p99 2.706/13.122/37.237 to 2.619/12.606/35.985 deg, ball position p99 56.4 to 55.5 UU; horizon 1 ball position p99 30.8 to 32.0 UU (slightly worse). The free-flight ball and airborne car tick audits still match to storage precision (ball position p50/p99 0.0051/0.014 UU; 95.1% of pairs below 0.01 UU versus 97.3%, car 89.7% versus 91.5%, unexplained but tiny). The remaining unresolved items (missed hits, flip and jump start timing, ground driving) are unchanged in kind. `replays/test` remains sealed.

## Ball-car contact: the pinned RocketSim drops the hit impulse (2026-09-29)

**Question.** After exact lag chains, ball residuals are essentially exact away from cars (position p90 0.009 UU), but ball near a car held 66% of ball position and 92% of ball velocity squared error. Is that limited by car replication, or by the contact model?

**Car age (`diagnose_contacts replays/train`).** Ball residuals with a car within 300 UU, by physical ticks from the ball packet to the car's nearest chain-lag packet: ball velocity error p90 is 9.8 UU/s at 0-2 ticks (64,438 residuals), 105.1 at 3-5 (41,500), 1,065.2 at 6-8 (4,166) and 1,075.2 at 9-11 (2,053); position p90 0.57, 1.84, 13.78, 14.55 UU. So car extrapolation limits most of the spread, but the tail persists next to a car packet (p99 velocity error 1,454 UU/s at 0-2 ticks). Classifying intervals with a car packet within 2 ticks by a *real* touch (ball velocity differs from a ball-only RocketSim prediction by more than 50 UU/s) and by whether RocketSim produced a `CarHitBall`: no touch either 53,347 (velocity error p99 11 UU/s); real touch with a sim hit 3,607 (error p50/p90 683/1,425 UU/s against a real impulse p50 of 1,279); real touch and sim no hit 856 (error p50 562); phantom sim hit 1,807 (p99 1,160). Even with a fresh car, hits were about half right.

**Contact model with a nearly exact car (`diagnose_contact_model replays/train [scale]`).** 1,773 real touches with a single car within 500 UU and a chain-lag car packet at the earlier ball packet's tick or one before; only that car (with its hitbox and observed throttle, steer, handbrake, boost) and the ball are simulated to the next ball packets. Result: RocketSim's ball impulse is about **half** the real one (median 265 vs 946 UU/s overall, 526 vs 1,007 UU/s for sim hits; 78 vs 367 on the ground) while its direction is right (median cosine 0.97), across ground, airborne, dodging and jumping cars and boost on or off. A head-on probe (`simulate_hit_probe`) shows a 2,000 UU/s car sending a stationary ball out at only 1,531 UU/s, slower than the car.

**Cause.** RocketSim's `on_hit` adds an extra ball-car impulse (0.65 x relative speed at low speed, scaled by `ball_hit_extra_force_scale`) to the ball's per-tick accumulator (`accum_lin_vel`). The `CarHitBall` event reports it (`extra_hit_vel`, 1,133, 2,267 and 3,400 UU/s at scale 1, 2, 3 for the 2,000 UU/s probe) but the ball's speed after the hit is identical in all three runs (1,530.6 UU/s): the impulse never reaches the ball, because the accumulator is cleared at the start of the next tick. All five RocketSim revisions in the local cargo cache (2026-06-22 to 2026-08-26, including the pinned `79f4d22`) have this structure. The intended impulse is part of the game's hit response.

**Fix (`apply_hit_extra_impulse`, default on; `--no-apply-hit-impulse`).** `step_tick_with_hit_impulse` adds the reported `extra_hit_vel` of every `CarHitBall` event to the ball's velocity at the end of the same tick, leaving the dependency pin unchanged. A unit test checks the head-on hit: the reported impulse is present in the ball's speed only with the workaround.

Contact-model test, sim impulse and velocity error against the real touches (impulse off, then on): all touches 261 vs 525 UU/s sim impulse (real 946) and ball velocity error p50/p90 561.5/1,457.7 to **28.5/1,024.6** UU/s; sim hits only, sim impulse 526 vs 1,001.5 (real 1,008.5), error p50/p90 534.7/1,331.7 to **13.8/199.4**; direction cosine p10/p50 0.79/0.97 to 0.99/1.00; at the next ball packet (contact finished) error p50/p90/p99 481.4/1,315.3/1,823.2 to **25.1/354.6/1,048.9**.

Corpus (60/60 replays each, zero failures): one-step ball linear-velocity p99 1,073.4 to **467.5** UU/s (train) and 1,091.0 to **475.7** (validation), ball angular p99 1.457 to 1.218 and 1.509 to 1.273 rad/s, ball rotation p99 4.372 to 4.184 deg; per replay ball velocity p99 improved in 60/60 (largest p90 regression 0.0035 train, 0.475 UU/s validation, on a scale of thousands). In the budget, ball near a car velocity p90 falls 52.4 to **15.2** UU/s and its share of velocity error 0.916 to 0.880; car metrics are unchanged. Masked prediction with aligned targets, ball position p99 at horizons 1-4: train 32.6/52.8/89.2/121.4 to 30.9/44.1/59.5/70.4 UU, validation 34.5/48.0/76.1/111.3 to 30.8/36.4/45.6/56.4 UU; velocity p99 h1/h4 train 960/1,460 to 650/1,163, validation 917/1,435 to 540/1,001 UU/s.

**What is left of ball contact.** With the impulse applied, sim hits reproduce the real impulse (median 1,001.5 vs 1,008.5 UU/s), so the remaining ball-contact error is mostly *missed* or phantom hits: 428 of 1,773 real touches (24%) still produced no sim hit (error p50 727 UU/s), and the hit rate falls from 87% with the car packet at A's tick to 51% when it is one tick stale, i.e. a few UU of car position error decides whether a touch happens. That is the car-replication limit you suspected, now isolated: better car state at the contact tick (between sparse packets, the ground driving model, flips) is what remains, and the ball packets themselves reveal contact timing and car position offline. Reproduce: `diagnose_contacts`, `diagnose_contact_model [scale] [--no-apply-hit-impulse]`, `simulate_hit_probe` (env `EXTRA_SCALE`, `NO_APPLY`), `error_budget [--no-apply-hit-impulse]`, `evaluate_corpus [--aligned-targets] [--no-apply-hit-impulse]`. `replays/test` remains sealed.

## Exact whole-tick lag chains (2026-09-29)

**Problem.** Position p90 stayed near 13 UU in every category of the budget and the ball in free flight still showed p90 7.3 UU, which I attributed to lag inference. `audit_lag_accuracy replays/train` checks that against RocketSim's exact whole-tick identification on 105,360 clean free-flight ball frame pairs: the tick count implied by the inferred chain lags was exactly right for only **91.1%** of pairs (off by one tick for 8.6%, by two for 0.2%). Physical tick differences between packets are integers, so those errors were an artifact of rounding continuous, noisy lag estimates independently.

**Measured basis for the fix.** The motion-based interval estimate is very close to the exact whole tick count: ball |estimate - exact k| p50/p90/p99/p99.9/max = 0.001/0.006/0.016/0.029/1.291 ticks (one outlier), airborne cars (5,536 exact fits) 0.004/0.016/0.045/0.068/0.094. So an estimate more than 0.25 tick from an integer is not reliable, and otherwise it can be snapped.

**Method (`exact_tick_lag_chains`, default on; `--no-exact-tick-lag-chains`).** `chain_packet_lags_exact` snaps every chained interval to an integer (pairs more than 0.25 tick from one end the run), so each packet's physical tick is `S = S0 + K` with integer `K` and lag differences are exact. A packet was generated no later than its frame time and no earlier than the previous frame's time, so on the integer timeline `tl(previous) <= S <= tl(frame)`; a run ends when no integer `S0` satisfies this for every packet (a mistaken interval shows up this way). Within the feasible starts, `S0` is the one that violates the real-time window `0 <= T - S <= window` least (ties to the middle), and the applied lag is the integer `tl(frame) - S`. A unit test with whole-tick synthetic physics recovers every lag exactly, rejects fractional lags, and checks the withheld-frame barrier.

Variants tried on train (ball |k'-k| = 0 share; car position p50/p90 UU): frame-integer chains centered on the feasible interval 99.6%, 0.223/5.566; hard real-time integer feasibility (runs cut whenever no integer fits with 0.15 tick slack) 96.6%, 0.284/10.737, worse because the window model is slightly loose and each cut loses the exact chain; no cuts at all (min-violation start only) 98.8%, 0.243/6.855, because one mistaken interval contaminates the rest of a run; **integer-timeline cuts plus min-violation start (default) 99.6%, 0.222/5.489**. The absolute tick cannot be checked with residual metrics (they are relative); the min-violation rule avoids the systematic one-tick shift that centering an interval about one tick wide can introduce, and the assigned lags come out spread over 0-4 ticks (about 10/33/29/18/10% for the ball on one replay, mean 1.85).

One-step pre-correction error, RocketSim p50 / p90 / p99 (60/60 replays each, zero failures, same samples):

| Split | Field | Before | After (default) |
| --- | --- | --- | --- |
| train | ball position UU | 0.005 / 10.208 / 25.903 | 0.005 / **0.009** / 19.231 |
| train | ball rotation deg | 0.000 / 2.782 / 5.729 | 0.000 / **0.028** / 4.372 |
| train | ball velocity UU/s | 0.010 / 5.443 / 1101.572 | 0.010 / **0.014** / 1073.418 |
| train | car position UU | 0.428 / 15.170 / 38.274 | 0.222 / **5.489** / 37.342 |
| train | car velocity UU/s | 7.400 / 89.100 / 567.531 | 5.875 / 87.378 / 566.508 |
| train | car rotation deg | 0.601 / 3.654 / 11.998 | 0.468 / 3.421 / 11.804 |
| validation | ball position UU | 0.006 / 13.862 / 26.504 | 0.005 / **0.011** / 20.434 |
| validation | car position UU | 0.667 / 16.602 / 38.320 | 0.222 / **5.513** / 37.372 |
| validation | car velocity UU/s | 8.268 / 89.895 / 559.008 | 5.846 / 87.810 / 557.026 |
| validation | car rotation deg | 0.655 / 3.693 / 11.988 | 0.460 / 3.402 / 11.770 |

Per replay (improved/worse/tied): car position p50 and p90 60/0/0 in both splits, p99 43/16/1 (train) and 49/11/0 (validation); ball position p50/p90/p99 60/0/0 in both splits (59/1 on validation p99); car rotation p50 60/0 and p90 58/2 (train), 60/0 and 56/4 (validation); car velocity p50 60/0 in both. The lag inference remains offline reconstruction; masked prediction is unchanged in method but its aligned targets are now more exact: validation masked car position p50/p90 at horizon 1 fall from 2.365/18.993 to 0.734/16.962 UU and at horizon 4 from 7.527/22.694 to 3.640/20.478 UU (train h4 7.361/23.359 to 4.294/20.926), with rotation p50 h1 1.129 to 0.978 deg and no material tail change. So the true four-frame (133 ms) forward position error of the causal model is about 3.6-4.3 UU at the median.

**What is left.** The clean budget (chain-lag packets) now has ball residuals essentially exact except in contact: ball position p90 0.0 overall, and **ball near a car holds 66% of ball position and 92% of ball velocity squared error** (p99 29.6 UU, velocity p90 52 UU/s). For cars, position p50/p90/p99 0.2/4.3/28.7 UU. Next: car-ball contacts, jump activity (18% of velocity error), ground driving and the flip start tick. Reproduce: `audit_lag_accuracy replays/train [--no-exact-tick-lag-chains]`, `audit_tick_integrality`, `audit_car_tick_integrality` (interval accuracy), `evaluate_corpus`, `error_budget`. `replays/test` remains sealed.

## Flip-cancel inference (2026-09-29)

**Problem.** Flips in the replays fade their pitch component at broad times because players cancel them (opposite pitch input); pitch is not replicated, so the converter never supplied the cancel and RocketSim's flip kept its pitch torque for the whole 0.65 s. The aerial inverse skipped every dodge interval. Dodge windows held 74% of car rotation squared error (median 6.3 deg) and 60% of angular-velocity error.

**Method (`infer_flip_cancel`, default on; `--no-infer-flip-cancel`).** RocketSim already models a cancel: an opposite pitch input scales the flip's pitch torque by `1 - |pitch|`. At each fresh car packet while the simulated car is flipping (with a relative pitch torque), `fit_flip_cancel` copies the corrected car state into a scratch arena, simulates the span to the next fresh packet of the same car at its exact physical tick (frame time minus the inferred lag) under cancels of 0, 0.25, 0.5, 0.75 and 1, applies the reported angular-speed limit, and keeps the cancel whose angular velocity is closest to that packet. The chosen cancel drives every interval in the span, and the last fit is held as a persistent input in spans with no later packet. Spans that are longer than 8 frames or 40 ticks, contain an inactive or withheld frame, or change the dodge counter are refused (the barrier keeps masked evaluation causal). The grid step is a resolution, not a tuned gate. This is offline reconstruction for fitted spans; the fitted angular velocity is partly in-sample by construction, so rotation, which is not fitted, is the independent evidence. A unit test recovers a simulated 0.75 cancel exactly and checks the withheld-frame and counter-change refusals.

One-step pre-correction car error, RocketSim p50 / p90 / p99 (60/60 replays each converted, zero failures; sample counts unchanged):

| Split | Field | Without | With flip-cancel (default) |
| --- | --- | --- | --- |
| train | rotation deg | 0.633 / 4.891 / 18.603 | **0.601 / 3.654 / 11.998** |
| train | angular rad/s | 0.139 / 1.322 / 4.363 | 0.124 / 1.017 / 3.619 |
| validation | rotation deg | 0.690 / 4.840 / 18.599 | **0.655 / 3.693 / 11.988** |
| validation | angular rad/s | 0.138 / 1.328 / 4.345 | 0.124 / 1.030 / 3.587 |

Per replay, rotation p50 and p90 improved in 60/0/0 train and 59/1/0 validation replays (improved/worse/tied); angular p50 and p90 in 60/0/0 in both. Position and velocity moved by at most 0.4 UU/s. In the dodge partition of the budget (chain-lag packets): rotation p50/p90/p99 6.31/17.17/27.9 to **2.41/9.42/22.5** deg, angular p50/p90 1.02/3.67 to 0.21/1.25 rad/s; whole-car rotation p90/p99 4.59/18.4 to 3.43/11.0 deg.

Masked prediction with aligned targets (causal: only the held cancel from earlier fitted spans can act inside a withheld window), car rotation p50/p90/p99 deg, before to after. Train: h1 1.167/7.303/22.426 to 1.126/5.768/17.704; h4 3.134/17.665/45.403 to 2.988/13.612/38.513. Validation: h1 1.175/7.305/22.892 to 1.129/5.650/16.496; h2 1.428/8.411/24.086 to 1.377/6.212/18.680; h3 2.131/12.443/38.044 to 2.058/9.379/28.777; h4 2.930/17.600/47.411 to **2.791/13.029/36.378**. Angular p90 h4 2.487 to 2.183 rad/s (validation); position unchanged.

**Remaining.** After this change the budget (chain-lag packets, car residuals) puts rotation squared error in dodge windows (56% share, p50 2.41 deg: the flip start tick inside a span and roll input are still unmodelled), ground no boost (11%, p90 2.47 deg) and air (13%); angular-velocity error in ground driving (24%, p90 0.98 rad/s) and dodges (33%); velocity error in the jump partition (18%, median 123 UU/s) and ground driving (14%). Reproduce: `error_budget replays/train [--no-infer-flip-cancel]`, `evaluate_corpus replays/<split> ... [--aligned-targets] [--no-infer-flip-cancel]`. `replays/test` remains sealed.

## Recovering input timing by fitting (jump probe, 2026-09-29)

`diagnose_jump_timing replays/train 400` tests whether an input's tick can be recovered by iterating candidates against exact packet ticks, with an out-of-sample check. For 400 jump activations (jump counter even to odd) with a chain-lag ground packet before and at least three packets after, RocketSim was started from the earlier packet with the observed throttle, steer, handbrake and boost, every (start tick D, hold length H in 1-24 ticks) was simulated, the first two later packets were used to fit, and the rest were held out (error = position UU + 0.1 x velocity UU/s). Findings: the fitted start is broad and typically *after* the counter's frame time (p10/p25/p50/p75/p90 = -4.2/-3.0/+3.0/+8.0/+11.0 ticks), the fitted hold is short (p50 6 ticks, p25 1), and the start is only weakly determined: the best start two or more ticks away scores within 1 unit of the best in 36% of events (median margin 1.62). The fit works in sample (p50 12.8, p90 55.2) but held-out error stays large (p50 64.8, p90 165.8), because air control, boost and dodge after the jump are unmodelled; pinning D to the activation frame time (fitting only H) gives 72.8 and 211.4, so fitting the start improves held-out error by about 11-21% and is better in 283 of 361 decided events and worse in 78. So the approach works in principle and is informative, but for jumps the start and hold trade off (a later start with a longer hold mimics a smoother force profile), and a start later than the counter's frame time suggests RocketSim's jump profile or the counter's meaning differs from the game's. It is a diagnostic, not a converter change. `replays/test` remains sealed.

## Re-budget after timing, reported-velocity limits and flip physics (2026-09-29)

**Cleaner budget.** `error_budget replays/train` now uses only packets whose lag came from their own motion chain (1,443,438 residuals; 59,916 without a chain lag skipped), so residuals isolate model error from timing error, and assigns each residual to one exclusive behavior partition so squared-error shares add up. Car residuals (948,684): p50/p90/p99 position 0.4/14.4/33.3 UU, velocity p50/p90 6.7/80.3 UU/s, rotation 0.60/4.59/18.4 deg, angular p50/p90 0.13/1.26 rad/s (after the fix below). Largest shares of squared error:

| Car partition (n) | Position | Velocity | Rotation | Angular velocity |
| --- | --- | --- | --- | --- |
| dodge counter odd (105,092) | 0.251 | 0.383 | **0.737** (p50 6.31 deg) | **0.596** |
| jump counter odd (30,661) | 0.048 | **0.183** (p50 123 UU/s) | 0.010 | 0.027 |
| ground, no boost (369,790) | 0.286 | 0.136 | 0.054 | 0.131 |
| air, no boost (142,518) | 0.123 | 0.027 | 0.057 | 0.043 |
| ground boosting (92,613) | 0.082 | 0.026 | 0.013 | 0.032 |
| all other partitions | 0.21 | 0.23 | 0.13 | 0.17 |

Ball residuals (494,754): ball near a car holds 0.892 of velocity error and 0.905 of angular error; ball in high air holds 0.501 of position error (p90 7.3 UU) although its flight is exact, which is residual lag-inference error. Position p90 is about 12-14 UU in nearly every partition (p50 about 0.4), so about one packet in ten still has a lag wrong by a tick; exact whole-tick identification (see the section above) should remove it.

**Reported-velocity limits (fixed, default on; `--no-limit-reported-velocities`).** RocketSim applies its speed limits (car 2300 UU/s and 5.5 rad/s, ball 6000 UU/s and 6 rad/s) at the *start of the next tick*, after which the flip torque adds up to 2.17 rad/s more, so the state it reports after a step can exceed limits that recorded server states never do (a flipping car showed 7.5 rad/s where replays show 5.50). The converter now limits the reported state after each step; trajectories are unchanged. One-step car angular velocity p50/p90/p99 on train 0.150/2.081/5.268 to **0.139/1.322/4.363** rad/s (validation 0.151/2.061/5.236 to 0.138/1.328/4.345), car velocity p50 8.233 to 7.722 UU/s (validation 8.758 to 8.422), rotation and position unchanged. In the dodge partition angular p50/p90 fell 2.39/4.70 to 1.02/3.67. Under `--aligned-targets` on validation the masked angular p90 at horizon 1/4 fell 1.940/2.835 to 1.609/2.487 rad/s; nothing else changed.

**Flip physics.** Local-frame angular velocity in 2,704 train dodges with a fresh torque (aligned to the first packet with |w| >= 2 rad/s; medians): |w| reaches the 5.50 rad/s cap within 6 ticks and the direction converges on a pure roll about the car's forward axis. Roll-dominant torque (angle < 30 deg): roll 1.99, 4.39, 5.09, 5.38, 5.46 at 0, 6, 12, 18, 24 ticks with pitch 0.2 and yaw 0. Diagonal (30-60 deg): roll 2.44, 4.13, 4.71, 5.38 and pitch 1.30, 1.34, 1.10, 0.48, 0.13 at 30 ticks. Pitch-dominant (> 60 deg): pitch 1.46, 2.02, 3.08, 2.79, 2.75 then 0.72 at 30 ticks and 0.17 at 42. RocketSim (`simulate_flip_profile`, upright motionless car): roll-dominant matches (5.50 held), but it holds a diagonal flip's pitch at 3.5 rad/s and a pitch flip at 5.4 rad/s for its whole 0.65 s torque time. The pitch component's drop time (tick at which it falls below half its peak and stays) is broad, not a fixed timescale: p10/p25/p50/p75/p90 = 6.6/14.7/25.7/61.8/90.6 ticks for pitch-dominant and 6.1/12.2/26.7/70.2/86.2 for diagonal, with a second cluster at 60-96 ticks (RocketSim's 78-tick torque end); only 14-22% show no drop within about 26 ticks. This looks like unobserved player flip cancels (opposite pitch input; RocketSim does model it, scaling the flip pitch torque by `1 - |pitch|` when the pitch input has the sign of the flip's relative torque). Pitch is not replicated, so the cancel timing must be inferred from the angular-velocity packets; the offline aerial inverse currently excludes every interval containing a dodge counter, which is where most rotation error remains.

`diagnose_dodge_start` (600 train dodges, RocketSim started from the last exact packet before the activation, dodge triggered at every candidate tick, fit against later packets at their inferred ticks): with the limit applied the RMS angular-velocity fit error at the best start is p50/p90 2.31/4.27 rad/s (3.19/4.99 without it) and grows with time after the start (packet 0: 1.03, packet 5: 2.91 rad/s p50); the best start relative to the activation frame time is broad and bimodal (p10/p50/p90 -7/0/11 ticks), so a plain RocketSim dodge with no cancel input does not fit the trajectory whatever the start. Early single-event traces show RocketSim's ramp matches the replay when started at the right tick (event 0: (2.25, 0.33, -2.05) vs (2.86, -0.16, -2.53) at the second packet).

**Next.** A flip-aware inverse for dodge intervals that infers the cancel input and its start tick by simulating candidates against the exact packet ticks; then the jump-counter partition (18% of velocity error), exact whole-tick lag identification, and ball-car contacts. Reproduce: `error_budget`, `diagnose_dodge_start replays/train [max_events]`, `simulate_flip_profile` (no replay data). `replays/test` remains sealed.

## Replay packets are exact server ticks; aligned masked targets (2026-09-29)

**Question.** Is a replay packet an exact 120 Hz physics state, and what does its frame time mean? This decides how timing is modelled, how the state at a frame time is defined, and how error is measured. The tests below use RocketSim itself as the forward model, so they do not depend on the lag-inference heuristics. All are offline **train** diagnostics (`audit_tick_integrality`, `audit_car_tick_integrality`, `audit_tick_offsets`; each refuses paths containing `test`).

**Ball packets are exact ticks.** For 101,202 consecutive-frame pairs of clean free-flight ball packets (z 200-1750 UU, away from walls and any car), RocketSim started from the first packet and stepped a whole number of ticks k in 0..12 reproduces the second packet with position error p10/p50/p90/p99 = 0.0029/0.0050/0.0069/0.013 UU (no pair above 0.1 UU), velocity error 0.0057/0.0098/0.0131/0.016 UU/s, and rotation error below 0.001 deg for 94.7% of pairs (the rest below 0.05 deg). Signed residuals converted to ticks lie within +/-0.02 tick for 99.9-100% of pairs; a packet sampled between ticks would show up to +/-0.5 tick (about 10 UU at the median 1,763 UU/s). RocketSim's ball flight matches the server to the replay's storage precision. The best whole-tick count k differs from `round(frame dt * 120)` by -2/0/+2 at p10/p50/p90.

**Car packets too, where motion is predictable.** For 5,756 airborne car packet pairs (up to three frames apart, zero throttle, unchanged boost/jump/dodge counters, clear of walls, ceiling, ball and cars; linear motion is ballistic whatever the unknown pitch/roll), 91.5% match the next packet's position to below 0.01 UU with a whole tick count (median 0.0050 UU), 98.4% within 0.02 tick along the velocity, independent of the frame gap. The 6% that miss are probably unmodelled inputs. Whole ticks between car packets cluster at 9, 11, 2, 10, 7, 8 (a roughly 10-tick replication cadence, unrelated to the frame grid).

**What the frame time is.** It only bounds the packet: on 1,007 unbroken runs of exactly identified ball ticks (median 38 packets), the offset of each packet's physical tick behind its frame time has a within-run range of **3.03/3.99/4.34 ticks (p10/p50/p90; max 7.06)**, slope 0.0009 ticks/frame (no drift; p10/p90 -0.019/+0.022), and lag-1 autocorrelation 0.06. Above the run minimum the offsets are spread roughly uniformly over 0-4 ticks (share per half tick: 17.5, 10.8, 27.3, 9.5, 15.2, 4.6, 10.1, 2.4, 2.4, then about 0), and the increments between consecutive frames' ticks range over 1-9 (mode 4). That is a stationary, independent jitter of about one frame period (4 ticks): each packet is an exact server state from a tick inside the preceding frame interval, and the frame timestamp does not identify which. It supports the window model used by `infer_packet_lag`, pins the per-run constant to about +/-0.1-0.2 tick, and makes half the window (2 ticks) the median prior for an unknown lag. Whole-tick lags are what RocketSim can represent; the frame time itself is real valued, so the state at a frame time is defined only to within about one tick.

**Consequences for error measurement.** (1) Pre-correction residuals at packet time are clean physics-fidelity measures (a free-flight ball is reproduced to 0.005 UU). (2) A raw packet is not the state at its frame time: a masked comparison against it carries an unpredictable offset of up to 4 ticks (about 11-16 UU of position at typical speeds, or several degrees of rotation for a spinning car) that no model can remove. (3) `evaluate_corpus --aligned-targets` therefore keeps lag inference on for pre-window frames of the masked conversion (chains never bridge a withheld frame, so it stays causal) and takes each target from the unmasked offline reconstruction at the same frame time (the packet advanced by its inferred lag) in place of the raw packet. `--aligned-targets --no-infer-packet-lag` reproduces the old numbers exactly.

Masked car errors on validation, RocketSim p50 / p90 / p99, raw-packet targets versus aligned targets (same default options, 10,405/10,623/10,479/10,401 samples at horizons 1-4):

| Horizon | Position UU: raw | Position UU: aligned | Rotation deg: raw | Rotation deg: aligned |
| --- | --- | --- | --- | --- |
| 1 | 16.562/41.040/75.398 | **2.378/18.995/56.487** | 1.821/7.890/20.206 | 1.175/7.305/22.892 |
| 2 | 14.959/38.364/62.182 | **3.086/18.952/48.180** | 1.806/8.183/21.953 | 1.428/8.411/24.086 |
| 3 | 16.198/40.064/76.493 | **5.215/20.057/58.624** | 2.382/11.800/34.916 | 2.131/12.443/38.044 |
| 4 | 15.953/39.604/76.639 | **7.546/22.721/61.044** | 3.019/16.438/44.443 | 2.930/17.600/47.411 |

Most of the earlier masked position error was target timing noise: the true horizon-four forward error is about 7.5 UU at the median. The aligned rotation tail is a little worse because the aligned truth itself advances the packet a few ticks with default air controls. The aerial-control decisions were tuned against the noisy target, so they were re-tested under aligned targets (horizon-four rotation p50/p90/p99 deg; validation, then train): no persistence 2.982/21.795/50.361 and 3.182/22.306/49.776; no persistence and no handbrake roll 3.005/22.915/50.599 and 3.214/23.355/50.297; calibrated persistence (default) 2.930/17.600/47.411 and 3.134/17.665/45.403; tuned legacy gate 2.879/17.038/45.411 and 3.105/16.682/42.669. The conclusions stand (persistence and handbrake roll help; the tuned gate keeps a small tail edge); only the noise floor moved. Position is unaffected by aerial controls (7.564 to 7.546 UU p50).

**Limits and follow-ups.** The chain-based lag inference is approximate; an exact identification by whole-tick RocketSim simulation (as in the audits) would be more robust for ball and airborne cars. Ground cars need the unknown inputs. Aligned targets rely on the offline reconstruction, so they are only as good as the inferred lags (tiny where identified); do not use them with `--no-infer-packet-lag` expecting new information. Remaining masked error on aligned targets is physics (contacts, driving, aerial control): position p90 about 22 UU at horizon four.

Reproduce: `audit_tick_integrality`, `audit_car_tick_integrality`, `audit_tick_offsets` on `replays/train`; `evaluate_corpus replays/<split> target/<name>.json --aligned-targets` (add the aerial flags for the re-test). `replays/test` remains sealed.

## Error budget and packet-lag inference (2026-09-29)

**Why this step.** After the aerial-control work the question was whether the largest error sources were being addressed first. `error_budget` (train, default options before this change) splits the converter's own one-step pre-correction residuals by object, packet gap, regime and direction of travel. Car position error was p50/p90/p99 17.0/42.0/67.9 UU (1.0 million samples, 68,891 above 50 UU) and ball position 10.7/34.8/64.8 UU, against car rotation p50 1.73 deg, so aerial rotation was a small part of the total. **97-99% of position error is along the direction of travel (`along` 0.97 car, 0.98 ball)**, and even a free-flight ball with 5 UU/s velocity error missed by 10.7 UU: a timing error, not a physics error. Car share of squared position error: ground 0.498, airborne 0.393, near ball 0.063, wall/ramp 0.047; frame gap 2 carried 0.408 and gap 3 0.274.

**Finding (offline train audits, `audit_ball_ticks`, `audit_shared_timing`, `audit_car_ticks`).** Replay frame times are regular (median 33.3 ms, 4 ticks), but the 120 Hz ticks implied by an object's own motion between consecutive packets are not. For 182,518 free-flight ball frame pairs with nominal 4 ticks the implied count spreads over 1-9 ticks, and the lag-1 autocorrelation of implied minus nominal is -0.42 (about -0.5 for independent per-frame jitter on a regular grid). Consistent with each packet having been generated at an arbitrary time inside its frame window `(previous frame time, frame time]`, so its state is valid up to 0-3 ticks (0-25 ms) *before* the frame time the converter assigned, which is 0-37 UU at 1500 UU/s. Car packets have their own timing: ball and car lags correlate at only r = 0.30 (`audit_shared_timing`: car consecutive-frame packets imply about 2 ticks fewer than nominal), while two cars over the same frame pair correlate at 0.82. The physical spacing between consecutive car packets is about 9-11 ticks regardless of whether the frame gap is 2 or 3, with occasional extra packets about 2 ticks after a regular one, and cumulative implied time equals frame time on average (per-lifetime drift p10/p50/p90 = -2/0/2 ticks).

**Method (`infer_packet_lag`, default on; `--no-infer-packet-lag`).** For each ball and each car actor lifetime, chains of consecutive fresh packets give the implied elapsed ticks from displacement along the mean endpoint velocity (exact for constant acceleration, biased only at second order in the turn angle). The chain fixes lag *differences* between packets; each lag must lie in its frame window, so the unknown constant is confined to an intersection interval and centered in it (`infer_packet_lags`). Pair validity: ball smooth motion (speed above 300 UU/s, velocity cosine >= 0.97, speed change <= 25%, <= 2 frames apart); cars speed above 350 UU/s, cosine >= 0.95, speed change <= 40%, <= 8 frames, no dodge counter, all frames active. Runs end at an invalid pair or infeasible window. The converter then splits each replay interval into groups by lag: it steps to each object's packet time, applies that object's correction there (so pre-correction residuals compare like with like), and steps on to the frame time, so every exported state is still the state at the frame time. Lags are per car; a car without its own chain uses the frame median car lag; without inference the default is half the frame window (the median of the roughly uniform lag distribution). Each frame record carries `packet_lag_ticks` (ticks and source: `chain`, `frame_median`, `default`) as provenance; they are inferred, not observed. On one validation replay 95% of ball and 92% of car packets used their own chain lag.

**This is offline reconstruction.** It uses the next packet of the chain, and the fitted timing makes the position residual partly in-sample. Independent evidence: rotation (never used by the inference) and ball rotation, and the ball's velocity when it is unused. One-step pre-correction error, RocketSim p50 / p90 / p99 (train and validation, 60/60 replays each converted, zero failures; sample counts unchanged):

| Split | Field | Without lag inference | With lag inference (default) |
| --- | --- | --- | --- |
| train | car position UU | 16.958 / 41.995 / 67.928 | **0.435 / 15.168 / 38.274** |
| train | car linear velocity UU/s | 22.044 / 108.365 / 579.417 | 8.233 / 89.320 / 568.022 |
| train | car rotation deg | 1.730 / 7.432 / 19.494 | **0.633 / 4.891 / 18.603** |
| train | ball position UU | 10.669 / 34.850 / 64.811 | **0.005 / 10.208 / 25.892** |
| train | ball rotation deg | 2.776 / 5.730 / 11.416 | **0.000 / 2.784 / 5.729** |
| validation | car position UU | 16.391 / 41.095 / 69.570 | **0.674 / 16.603 / 38.320** |
| validation | car linear velocity UU/s | 21.638 / 108.155 / 572.439 | 8.758 / 90.083 / 559.950 |
| validation | car rotation deg | 1.677 / 7.360 / 19.392 | **0.690 / 4.840 / 18.598** |
| validation | ball position UU | 10.680 / 33.886 / 64.281 | **0.006 / 13.862 / 26.508** |
| validation | ball rotation deg | 2.820 / 5.730 / 10.922 | **0.000 / 2.864 / 5.729** |

Car position errors above 50 UU fall from 68,891 to 5,850 on train. The along-track share drops from 0.97 to 0.86, so the remaining error is mostly physics (contacts, driving, aerial control). After the change the car squared-error share is airborne 0.444, ground 0.386, near ball 0.104, wall/ramp 0.066. Per replay, final model versus no inference: car position p50 and p90 improved in 60/60 replays in both splits, p99 in 60/60 (train) and 58/60 (validation, worst +1.10 UU); ball position p50/p90/p99 improved in 60/60 in both splits; car rotation p50 and p90 improved in 60/60 in both splits.

Development path on train (car position p50/p90/p99 UU; car rotation p50 deg): frame-shared lag, strict thresholds, default 0: 7.709/35.069/73.633, 1.154; default half window: 7.374/33.941/70.413, 1.139; relaxed car thresholds: 4.654/33.330/71.140, 1.045; **per-car lags: 0.435/15.168/38.274, 0.633**. The frame-shared version worsened 13% of packets by more than 10 UU (47% improved), which motivated per-car lags. Relaxing the ball's altitude guard gave ball position p90/p99 11.3/28.1 from 15.3/43.6 with the default half window. Threshold values were set by coverage and these train numbers; validation was run only afterwards.

**Masked benchmark.** A withheld target's own lag is unknowable (about uniform over the window), and a correctly timed state carries that full lag against a packet, whereas the old un-timed baseline benefited from two lags partly cancelling: with lag inference on, masked validation h1 position rose 16.56 to 19.49 UU. That comparison is not meaningful, so `evaluate_corpus` disables packet-lag inference for the masked conversion. Masked numbers (aerial-control benchmark) are unchanged. A causal timing model would need the roughly regular 9-11 tick car cadence; it is not implemented.

**Limits and follow-ups.** The chain needs smooth motion; slow, sharply turning or contact intervals fall back to the frame median or the half-window default. Lags are rounded to whole ticks. Earlier aerial-control fits (`solve_span_air_controls`, persistence calibration) still use frame-time deltas; using the inferred packet-time deltas is the obvious next accuracy step and could change those calibrations. Remaining budget items: car-ball contacts (ball near a car: velocity p90 173 UU/s), ground driving and aerial dynamics. The test `live_car_wins_over_retired_car_with_same_player` now pins `infer_packet_lag: false` because it asserts state equals packet.

Reproduce: `cargo run --release --bin error_budget -- replays/train [--infer-packet-lag]`; `audit_ball_ticks`, `audit_shared_timing`, `audit_car_ticks`, `lag_outliers` take `replays/train`; `evaluate_corpus replays/<split> ...` with and without `--no-infer-packet-lag`. `replays/test` remains sealed.

## Input-process calibration replaces tuned persistence gates (2026-09-29)

**Why.** The persistence gate (`max(|pitch|,|roll|) >= 0.5`, 0.15 s expiry) and the 4-frame span cap were chosen by sweeping masked error, so they were hill-climbed constants. Audit of the aerial controls added on this branch: *derived from mechanics or source* were the future-packet barrier, the forward model (mirrors RocketSim `update_air_torque`) and handbrake-as-air-roll (steer correlates with inferred roll at r = 0.64 with handbrake, 0.13 without); *tuned by error sweeps* were the 0.5 magnitude gate, the 0.15 s expiry, the 4-frame span cap and the off-by-default speed-drop gate. This section replaces the tuned constants by quantities measured on the input process itself (no state error involved); the tuned gates remain as `--legacy-persist-gates` for comparison. It supersedes the defaults named in the two sections below.

**Span cap removed.** With no cap (any bracketing pair in the active phase; a constant control over the pair beats no control), one-step train car rotation p50/p90/p99 is 1.731/7.432/19.481 deg for 12 and 40 frames alike, versus 1.743/7.438/19.475 with the tuned 4 frames / 0.15 s. The default is now effectively unbounded.

**Calibration tool.** `cargo run --release --bin calibrate_air_control_persistence -- replays/train [max_span_seconds]` fits the constant control for every pair of consecutive fresh car angular packets in the air (416,600 spans on the 60 train replays; refuses paths containing `test`) and pairs fitted controls of the same actor lifetime at increasing lag. Findings:

- Persistence is strongly axis dependent. Least-squares rho(lag) is about 0.51 / 0.59 / 0.75 (pitch / yaw / roll) at lag 0-0.033 s, and 0.32 / 0.36 / 0.72 at 0.067-0.10 s. Pitch and yaw reach about 0 by 0.15-0.2 s, roll still 0.49 at 0.27-0.30 s.
- There is no threshold structure in the conditional mean (later/earlier ratio roughly constant in magnitude), and a fitted roll of about 0.69 recurs at every magnitude: it is the roll that balances RocketSim's roll damping at the 5.5 rad/s angular speed cap (4.8 x 5.5 / 38.3), so a fit at the cap is a lower bound on a held input and persists. Regressions on the previous control and a cap flag add little (R2 0.11 for pitch/yaw, 0.54-0.57 for roll).
- Conditioning on the joint magnitude of the other axes does not raise the pitch hold probability (0.43-0.49 at every level), so the tuned gate's joint rule has no support in the input process.
- Errors are judged by quantiles of absolute error, for which the optimal point prediction of an uncertain input is its conditional median. The default (`air_persist_calibrated`) therefore multiplies each fitted control by the measured median-later-control ratio `AIR_CONTROL_MEDIAN_RATIO[axis][lag band][|u| bin]` (bands 0.033-0.083, 0.083-0.133, 0.133-0.2 s; nothing beyond 0.2 s or below 0.1 magnitude), and uses observed steer for yaw (or for roll while the handbrake is held) instead of a persisted value. The table is copied from the tool's train output.

Alternatives evaluated on the input process and then on train horizon-four rotation p50/p90/p99 (deg): least-squares shrinkage rho(lag) 3.221/16.180/42.548; persist-iff-P(held)>0.5 3.202/16.682/43.825; **conditional-median ratios 3.187/16.097/42.147**; no persistence 3.227/20.130/45.784; tuned gate 3.189/15.372/39.268. Ignoring observed steer (persisting fitted yaw/roll) made no material tail difference (16.559/43.809).

Masked car rotation (p50 / p90 / p99 deg), no persistence versus calibrated default versus tuned legacy gates. Same samples per row (train 10,120/9,931/9,877/10,053; validation 10,405/10,623/10,479/10,401 at horizons 1-4):

| Split / h | No persistence | Calibrated (default) | Tuned legacy gate |
| --- | --- | --- | --- |
| train 1 | 1.835/8.919/22.652 | 1.809/8.219/20.736 | 1.810/7.944/19.691 |
| train 2 | 1.858/9.247/23.375 | 1.806/8.474/21.059 | 1.813/8.012/20.286 |
| train 3 | 2.642/15.301/38.702 | 2.566/12.329/36.510 | 2.595/11.833/34.193 |
| train 4 | 3.227/20.130/45.784 | 3.187/16.097/42.147 | 3.189/15.372/39.268 |
| validation 1 | 1.859/8.609/22.000 | 1.821/7.890/20.206 | 1.832/7.753/19.150 |
| validation 2 | 1.845/9.175/23.292 | 1.806/8.183/21.953 | 1.791/7.830/21.645 |
| validation 3 | 2.460/14.596/38.075 | 2.382/11.800/34.916 | 2.419/11.278/32.836 |
| validation 4 | 3.060/19.916/46.867 | 3.019/16.438/44.443 | 3.013/15.631/41.734 |

The alternate mask schedule (`--mask-seed 239847`) on validation horizon four: 3.130/20.500/47.885 (previous default without roll/persistence) and 3.046/15.156/42.351 (tuned) versus 3.044/16.027/44.411 (calibrated). Position is unchanged at every quantile (validation h4 15.953/39.604/76.639 UU). Per replay at horizon four, calibrated versus no persistence: rotation p90 improved/worse/tied 54/2/4 (validation) and 59/1/0 (train); p50 32/21/7 and 31/19/10 (worst regression +0.79 deg). Calibrated versus the tuned gate is *worse* at p90 in 43/60 validation replays (worst +6.0 deg) and 40/60 train replays.

**Honest trade-off.** The measured model gives up about 5% of the tail (p90/p99) that the tuned gate captured, while matching its p50, and it needs no error-tuned constant. Diagnosis: forcing *pitch* to persist at full magnitude for |u| >= 0.5 within 0.15 s (a temporary switch, removed) recovers the tail (train h4 3.173/15.275/40.924), while forcing yaw or roll changes nothing. Fitted pitch is bimodal (held or released, about 50/50) so the control-space median cannot decide it, yet state error favors holding. A hypothesis that time-averaging over multi-frame fitted spans attenuates the calibration target was tested by restricting calibration to spans of at most 0.045 s (76,738 spans): pitch rho at lag 0-0.033 s stayed 0.51 and the |u|~1 median ratio 0.45, so the hypothesis is not supported. Open question: a calibration objective in state space (fitted on train with a held-out check on validation, low-dimensional, per axis) may explain the pitch gap.

Reproduce: `evaluate_corpus replays/<split> ...` with defaults, `--no-persist-past-air-controls`, and `--legacy-persist-gates`; `replays/test` remains sealed.

## Residual harm in causal persistence and a speed-drop gate (2026-09-29)

`python/analyze_persistence_harm.py` joins the previous-default and new-default horizon-four **train** traces (10,052 windows) and keeps the 2,541 windows where persistence changed the first-interval pitch/roll (1,417 better and 529 worse by more than 1 deg; median -2.517, mean -4.599 deg). Features use only packets before the mask window. Harm concentrates where prior angular speed had been **falling** between the two fitted packets: trend below -0.2 rad/s gave 110 better versus 140 worse (mean +1.5 deg), while flat trends gave 1,040 better versus 238 worse (median -3.785) and speeds at or above 5.48 rad/s gave 1,014 versus 192 (median -4.261). Speed 5 to 5.48 rad/s (121 windows, 44 better/62 worse) and pitch-dominant persisted controls (657/306, median -1.349, versus roll-dominant 760/223, median -3.854) were the weakest groups. Age since the latest packet (up to 0.12 s), handbrake, altitude and steer magnitude did not isolate harm.

Following that, `--air-persist-max-speed-drop s` skips persistence when angular speed dropped by more than `s` rad/s between the two fitted packets. Train horizon-four rotation p50/p90/p99 (deg) for the current default and gates: none 3.189/15.372/39.268; 0.5 3.182/15.247/39.268; **0.2 3.155/15.213/39.268**; 0.0 3.181/17.220/43.340; -0.1 3.225/19.239/45.702 (tight gates discard helpful windows whose speed merely dips). With 0.2, validation horizon four is 3.013/15.631/41.734 to 2.990/15.561/41.703, the alternate mask schedule 3.046/15.156/42.351 to 3.014/15.004/42.351 (validation) and 3.251/15.917/41.289 to 3.222/15.854/41.133 (train), while horizons one and two move by at most +/-0.015 deg at p50 (validation h2 p50 1.791 to 1.806). Per replay at horizon four, p50 improved/worse/tied 28/10/22 (train) and 26/13/21 (validation); **p90 was 20/7/33 (train) but 16/14/30 on validation with a 3.0 deg worst replay**. The pooled gain (about 0.03 deg p50, 0.06-0.15 deg p90) is small and the per-replay evidence on validation is a wash, so the gate remains **off by default** (`air_persist_max_speed_drop` = 1e6). Keep it as an ablation and revisit only with a stronger discriminator.

Reproduce: run `evaluate_corpus` on `replays/train` with `--no-infer-air-roll-from-handbrake --no-persist-past-air-controls --rotation-trace target/trace-prev.jsonl` and with defaults plus `--rotation-trace target/trace-fin.jsonl`, then `python python/analyze_persistence_harm.py target/trace-prev.jsonl target/trace-fin.jsonl`. `replays/test` remains sealed.

## Causal past-control persistence and handbrake air roll (2026-09-29)

Two changes target *masked* (causal) prediction, which the gap-spanning inverse above leaves unchanged. Both are enabled by default; `--no-infer-air-roll-from-handbrake` and `--no-persist-past-air-controls` restore the previous behavior (verified: the two flags together reproduce the earlier default report exactly).

**1. Handbrake air roll.** When airborne with the replicated handbrake held, steer now drives RocketSim `roll` instead of `yaw`. Evidence (five train replays, 13,426 airborne samples with an inferred pitch/roll control and |steer| >= 0.3): with handbrake held (1,683 samples) steer correlates with the span-inferred roll at r = 0.641 and yaw at 0.296; without handbrake (11,743) yaw 0.662 and roll 0.126. This matches RL's common "powerslide = air roll" binding. The correlation uses the offline inverse only to establish the direction of the mapping; the control itself uses observed steer and handbrake, which are available inside withheld windows. Alone it improves horizon-four train masked rotation p50/p90/p99 from 3.251/20.859/46.592 to 3.227/20.151/45.784 deg.

**2. Causal persistence.** `past_persisted_air_controls` solves the constant control (same forward-model solver) between the two most recent fresh car angular packets at or before the interval and keeps it for up to 0.15 s after the latest packet, whenever no later bracketing packet is available. It reads nothing after the interval; a unit test shows that adding a later packet leaves the controls at earlier frames unchanged. Persisted values are used only when the larger of |pitch| and |roll| is at least 0.5 (`--air-persist-min-control`), the same altitude guard applies, and dodge/flip intervals are excluded.

Paired train windows (10,052 horizon-four rotation targets, joined by replay hash, actor lifetime and window start; `python/analyze_persisted_air_controls.py`) with **ungated** persistence showed why a magnitude gate is needed. Delta = persistence minus default rotation error at horizon four:

| Persisted max(abs pitch, abs roll) | Windows | Better by >1 deg | Worse by >1 deg | Median / mean delta (deg) |
| --- | ---: | ---: | ---: | --- |
| none | 6,664 | 13 | 11 | 0.000 / 0.000 |
| below 0.3 | 703 | 71 | 202 | +0.150 / +0.611 |
| 0.3 to 0.7 | 1,364 | 676 | 305 | -0.933 / -2.584 |
| 0.7 and above | 1,321 | 755 | 297 | -3.352 / -5.710 |

Prior angular speed 5.48 rad/s or higher (the apparent cap) gained most (median -1.757 deg, mean -5.715, 989 better versus 187 worse); 5 to 5.48 rad/s was net harmful (+0.899 mean). Small fitted controls are noise; large ones are sustained inputs. Threshold sweep on **train** (horizon-four rotation p50/p90/p99 deg; roll routing on): no gate 3.305/15.432/39.233, 0.2 3.278/15.296/39.236, 0.3 3.251/15.322/39.236, **0.5 3.189/15.372/39.268**, 0.7 3.185/17.202/42.064. Gain 0.5 without a gate was worse at p50 (3.327) than gain 1 (3.299), and a 0.08 s expiry was similar to 0.15 s, so neither was adopted. The 0.5 threshold was chosen on train before validation was run.

Masked car errors, RocketSim p50 / p90 / p99, previous default (roll routing and persistence off) versus new default. Samples are identical (train 10,120/9,931/9,877/10,053 rotation targets at horizons 1-4; validation 10,405/10,623/10,479/10,401):

| Split / horizon | Rotation deg: previous | Rotation deg: new | Angular rad/s: previous | Angular rad/s: new |
| --- | --- | --- | --- | --- |
| train h1 | 1.840/8.994/22.616 | 1.810/7.944/19.691 | 0.415/2.543/5.835 | 0.355/2.260/5.307 |
| train h2 | 1.872/9.482/23.586 | 1.813/8.012/20.286 | 0.456/2.597/5.947 | 0.408/2.283/5.385 |
| train h3 | 2.659/15.985/39.032 | 2.595/11.833/34.193 | 0.565/3.194/6.391 | 0.528/2.683/6.008 |
| train h4 | 3.251/20.859/46.592 | **3.189/15.372/39.268** | 0.580/3.467/6.382 | 0.579/2.972/6.215 |
| validation h1 | 1.861/8.855/22.029 | 1.832/7.744/19.150 | 0.391/2.517/5.848 | 0.337/2.242/5.462 |
| validation h2 | 1.850/9.339/23.352 | 1.791/7.830/21.645 | 0.436/2.677/5.941 | 0.392/2.366/5.503 |
| validation h3 | 2.475/15.155/38.429 | 2.419/11.264/32.836 | 0.516/3.131/6.209 | 0.487/2.661/5.873 |
| validation h4 | 3.078/20.512/47.126 | **3.013/15.631/41.734** | 0.579/3.511/6.396 | 0.561/3.044/6.174 |

Position is unchanged to within 0.15 UU at every quantile (validation h4 15.967/39.609/76.588 to 15.952/39.587/76.639 UU). The independent mask schedule (`--mask-seed 239847`) agrees: validation h4 rotation 3.130/20.500/47.885 to 3.046/15.156/42.351 deg and angular p50/p90 0.587/3.487 to 0.574/2.960; train h4 3.324/21.430/46.687 to 3.251/15.917/41.289 deg. Horizon-four rotation by validation game size (p50/p90/p99): 1v1 4.142/25.089/47.902 to 3.938/19.773/44.102, 2v2 3.028/20.812/48.140 to 2.924/16.235/42.626, 3v3 2.899/18.923/45.813 to 2.850/13.962/39.851; train sizes improve likewise (1v1 4.661/28.164/52.239 to 4.530/21.202/42.746). Per replay at horizon four, rotation **p90 improved in 60/60 train and 60/60 validation replays**. Rotation **p50 is mixed per replay**: improved/worse/tied 33/19/8 on train (largest regression 1.078 deg) and 36/16/8 on validation (0.342 deg) even though the pooled p50 improves. Angular p50 was 26/29/5 (train) and 37/18/5 (validation), p90 58/1/1 and 53/4/3 (largest validation regression 1.034 rad/s). Unmasked one-step car error changed slightly and positively (validation rotation 1.721/7.545/19.833 to 1.703/7.382/19.356 deg; linear velocity p99 571.3 to 572.3 UU/s, effectively noise).

This is the first low-air candidate that improves p50, p90 and p99 pooled at every horizon and both mask schedules, unlike the earlier holds and feedback controls. Limits: the masked protocol still supplies observed steer, throttle, handbrake and boost inputs at withheld frames (only physics fields are withheld), persistence assumes a player keeps an input for at most about 0.15 s, per-replay medians are not uniformly better, and the fitted pitch/roll remain RocketSim-equivalent estimates rather than recovered player inputs. `replays/test` remains sealed. Reproduce with `cargo run --release --bin evaluate_corpus -- replays/<split> target/<split>-final.json` (and the two `--no-*` flags for the previous default); for the window table run the ungated variant (`--persist-past-air-controls --air-persist-min-control 0`) and the default with `--rotation-trace` for each, then `python python/analyze_persisted_air_controls.py BASE.jsonl PERSIST.jsonl`.

## Gap-spanning aerial inverse (2026-09-29)

**Finding.** The offline aerial-control inverse used only the *next replay frame*, but car rigid-body packets usually arrive every 2-3 frames (see the raw-cadence audit). Most airborne intervals therefore kept zero pitch/roll while RocketSim's damping decayed spin the player was actually sustaining, which explains much of the long-standing full-match airborne angular deficit against hold. `span_lookahead_air_controls` now finds the fresh car angular packets that bracket each interval (same actor lifetime and player link, active phase, fresh position above the altitude guard at both ends, no odd dodge counter inside the span) and applies one constant control solved over the whole span to every interval inside it. Defaults are now spans up to 4 frames / 0.15 s, one forward-model refinement pass, and the 50-100 UU band included (`--air-lookahead-frames`, `--air-lookahead-seconds`, `--air-lookahead-refine`, `--no-infer-transition-air-lookahead`; `--air-lookahead-frames 1 --air-lookahead-seconds 0.05 --air-lookahead-refine 0 --no-infer-transition-air-lookahead` reproduces the previous behavior; the one-frame configuration was verified to give an identical train report).

The refinement replays RocketSim's per-tick air torque and damping (`air_angular_velocity_forward`, which mirrors `update_air_torque` in the pinned `rocketsim` source: torque only when a control is nonzero, `(1-|input|)` damping scaling for pitch and yaw, unscaled roll damping, 5.5 rad/s cap) and corrects the analytic inverse, because the analytic solve freezes damping at the span start. A unit test checks the round trip.

**This is offline reconstruction, not causal prediction.** It uses a packet from after the interval. The evaluator therefore hands the masked conversion a `withheld_frames` barrier; a span that contains any withheld frame is refused, and a unit test covers it. Masked four-frame results are consequently unchanged (below). The controls are RocketSim-equivalent estimates, not recovered player inputs.

**Which numbers are independent.** The inverse is fitted so that simulated angular velocity at the span end matches the observed packet, so the one-step *angular* error at those packets is partly **in-sample**. The inverse uses only the start orientation and the two angular velocities, so end **rotation**, position and velocity are *not* fitted. The rotation gains are the evidence.

One-step pre-correction car error, all fresh packets, RocketSim p50 / p90 / p99 (train 60/60 and validation 60/60 replays converted, zero failures; `replays/test` untouched):

| Split | Setting | Rotation deg | Angular rad/s (in-sample) | Air angular | 50-100 UU angular |
| --- | --- | --- | --- | --- | --- |
| train | previous default (1-frame) | 1.885 / 8.864 / 22.064 | 0.372 / 2.552 / 5.782 | 0.891 / 2.994 / 6.560 | 2.013 / 4.616 / 6.638 |
| train | span 4 + refine + 50 UU (new default) | **1.751 / 7.514 / 19.723** | 0.160 / 2.170 / 5.379 | 0.106 / 2.410 / 6.088 | 1.165 / 4.050 / 6.102 |
| validation | previous default | 1.855 / 8.750 / 21.954 | 0.366 / 2.528 / 5.782 | 0.887 / 2.979 / 6.495 | 1.999 / 4.661 / 6.785 |
| validation | new default | **1.721 / 7.545 / 19.833** | 0.169 / 2.186 / 5.394 | 0.126 / 2.436 / 6.035 | 1.297 / 4.154 / 6.255 |

Hold baselines are 8.03 / 28.34 / 39.47 deg rotation (train), and on train air angular 0.826 / 2.449 / 6.099 and 50-100 UU angular 0.688 / 2.789 / 5.679 rad/s. Sample counts are identical across settings (998,832 train and 1,056,577 validation rotation samples; 298,706 / 314,236 air; 142,390 / 148,464 transition). Car linear-velocity one-step quantiles moved by under 0.2%.

Train span sweep (rotation p50/p90; refine 0 and 50 UU guard off unless stated): 1 frame 1.885/8.864, 2 frames 1.825/8.372, 3 frames 1.786/8.109; 4 frames + 50 UU guard + refine 1: 1.751/7.514; 6 frames (0.22 s) + guard: 1.747/7.501, a negligible extra gain, so 4 was kept. The 50 UU guard alone at 4 frames lowers validation rotation p90 from 8.063 to 7.545 (rotation p50 1.749 to 1.721). Refinement mainly reduces the in-sample angular fit (train air p50 0.221 to 0.106) and only marginally the rotation (1.755 to 1.751); one versus three passes made no material difference.

Per-replay behavior (new default vs previous, one-step car rotation): p50 improved/worse/tied in 60/0/0 train and 59/0/1 validation replays; p90 60/0/0 and 59/0/1. Angular p50 and p90 improved in every replay in both splits. Per game size (p50/p90/p99 deg): train 1v1 2.442/11.63/24.93 to 2.204/9.284/22.85, 2v2 1.806/8.921/22.50 to 1.673/7.627/20.31, 3v3 1.790/8.072/20.56 to 1.666/6.914/18.18; validation 1v1 2.359/11.55/24.79 to 2.152/9.518/23.07, 2v2 1.847/8.978/22.58 to 1.709/7.657/20.46, 3v3 1.746/7.957/20.24 to 1.615/6.899/18.07. Velocity per-replay medians moved in both directions by at most 1.5 UU/s (p90), noise relative to 100+ UU/s errors. The 50 UU guard was chosen after the train sweep and its per-replay rotation effect checked on both splits (p50 improved in 60/0/0 train and 59/0/1 validation replays).

**Masked (causal) prediction is unchanged**, as intended: horizon-four car rotation p50/p90/p99 remains 3.251/20.859/46.592 deg on train and 3.078/20.512/47.126 on validation (validation angular p90 3.521 to 3.511 rad/s), so this change does not improve forward prediction across withheld intervals. Remaining limits: the 50-100 UU one-step angular p50 (1.17 train, 1.30 validation) is still worse than hold (0.69 / 0.67), dominated by recent dodge/flip intervals that the inverse deliberately excludes; spans assume constant input; and pitch/roll cannot be verified against real player inputs.

Reproduce (ignored outputs under `target/`): `cargo run --release --bin evaluate_corpus -- replays/<split> target/<split>-span-default.json`, and the previous behavior with the four flags above. `cargo test --all-targets` and the Python tests pass; a validation 1v1 replay exported to Parquet restored 8,126/8,126 snapshots.

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
