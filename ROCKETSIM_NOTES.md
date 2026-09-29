# RocketSim observations for its developers

This project does **not** modify RocketSim. It is pinned to `rocketsim` commit `79f4d22f` (native Rust `v3-rust`, 2026-08-26), and where a difference from the game matters, the converter works around it in its own code. This file records apparent inaccuracies, with the evidence, the RocketSim source location, the workaround used here, and how sure we are, as hints for the RocketSim developers.

**How to read the evidence.** Replay packets are exact 120 Hz server states (`RESULTS.md`, "Replay packets are exact server ticks"): where RocketSim, started from one packet, disagrees with a later packet at that packet's exact tick, and the cause is not an unobserved player input, the discrepancy points at RocketSim. Sparse packets and missing inputs (pitch, roll, hold lengths) limit how much can be attributed. All measurements use the 60 `replays/train` replays unless noted; `replays/test` is untouched.

Status labels: **verified** (isolated with a reproduction), **suspected** (mechanism visible in the source or data but not isolated), **open** (unexplained residual).

## 1. Extra ball-car hit impulse never reaches the ball (verified)

- **Symptom.** A ball hit gets about half the real impulse. Starting from a nearly exact car and ball state, RocketSim's ball impulse after a touch had a median of 526 UU/s against 1,007 UU/s in the replay (direction correct, median cosine 0.97); a 2,000 UU/s car drives through a stationary ball and sends it out at 1,531 UU/s, slower than the car (which keeps 1,757 UU/s).
- **Cause.** `Ball::on_hit` (`sim/ball/base.rs`, ~244-295) adds the extra impulse with `add_impulse(..., accum = true)`, i.e. to `accum_lin_vel`. `Arena::step_tick` clears the accumulators at its start (`sim/arena/base.rs:477`, `clear_accum_forces`), and the solver reads them when it builds its solver bodies (`bullet/dynamics/constraint_solver/solver_body.rs:47`). The hit callback (`on_car_ball_collision`, `sim/arena/base.rs` ~883-908) runs during contact processing, after that read, so the impulse is discarded on the next tick's clear. The `CarHitBall` event still reports it (`extra_hit_vel`).
- **Evidence.** Scaling `ball_hit_extra_force_scale` 1, 2, 3 changes the reported `extra_hit_vel` (1,133, 2,267, 3,400 UU/s for the 2,000 UU/s probe) but the ball's speed after the hit is identical (1,530.6 UU/s). Every RocketSim revision in the local cargo cache (2026-06-22 to 2026-08-26) has the same code. Reproduce: `simulate_hit_probe` (`EXTRA_SCALE=3`, `NO_APPLY=1`), `diagnose_contact_model replays/train [scale] [--no-apply-hit-impulse]`.
- **Workaround here.** `conversion::step_tick_with_hit_impulse` adds each `CarHitBall.extra_hit_vel` to the ball's velocity at the end of the same tick (`apply_hit_extra_impulse`). With it, the sim-hit impulse is 1,001.5 UU/s against 1,008.5 real (direction cosine 0.99-1.00), real-touch ball velocity error p50 falls from 561 to 28 UU/s, and corpus ball velocity p99 from 1,073 to 467 UU/s.
- **Hint.** The extra impulse probably needs to act on the ball's velocity immediately (or be applied to the accumulator in a place the solver still reads), rather than being accumulated for the next solver setup. Other `accum = true` impulses added from contact callbacks would have the same problem: `Ball::on_world_hit` (Heatseeker wall bounce, Snowday puck stick) matches that pattern but is outside soccar and was not tested. Car-car bump impulses use `vel_impulse_cache` instead and were not checked against replays.

## 2. Speed limits are applied at the start of the next tick, so reported states exceed them (verified)

- **Symptom.** A flipping car reports angular speed up to about 7.5 rad/s (7.44-7.71 seen) although the game's recorded states never exceed 5.50 rad/s (in flips the replay magnitude sits at exactly 5.50).
- **Cause.** `Arena::step_tick` applies `limit_vels(car::MAX_SPEED, car::MAX_ANG_SPEED)` (5.5 rad/s) and `quantize` at the beginning of the tick (`sim/arena/base.rs` ~481-495), and the flip then adds up to `flip::TORQUE` x `TICK_TIME` = 260/120 = 2.17 rad/s after the limit (`sim/car/base.rs` ~313-380). The trajectory is unaffected (the next tick limits again) but the state read after a step is not a limited state. The flip's `SPIN_CAP_X/Y` (7.44/7.23) are applied to `rb.ang_vel.x/.y`, which are world-frame components, so they cap the wrong quantity for a rotated car; they only look intended for the flip's own axes.
- **Evidence.** `simulate_flip_profile` (reported |w| 7.44-7.71 while flipping, limited 5.50); the same pattern applies to linear speed (2300 UU/s) and the ball (6 rad/s, 6000 UU/s).
- **Workaround here.** The converter limits the reported state after each step (`limit_reported_velocities`): one-step car angular velocity p90 2.081 to 1.322 rad/s.
- **Hint.** Apply the limits (and quantization) at the end of the tick, or expose a limited/quantized state getter. Replay states also appear to be quantized (ball position error floors at about 0.005 UU); reported RocketSim states are not, because quantization also happens at the start of the tick (suspected, not isolated).

## 3. Ball-car contacts missed or invented near the contact threshold (open)

- **Symptom.** With the impulse restored (item 1), hits that occur match the real impulse, but 24% (428 of 1,773) of real touches still produce no `CarHitBall` when the car starts from a packet at the ball packet's tick, and the hit rate falls from 87% to 51% when that car packet is one tick stale. In a separate cut, 1,807 intervals had a sim hit with no real touch.
- **Attribution.** Most of this is car state error at the contact tick (a car moves about 12 UU per tick), which the replay cannot resolve. Whether RocketSim's car hitbox, contact margin or hit-detection threshold also differs from the game cannot be separated with sparse packets; there is no evidence either way. Reproduce: `diagnose_contact_model`, `diagnose_contacts`.

## 4. False demolitions on actively driven cars (observed, cause not investigated)

- **Symptom.** When RocketSim was stepped with car states injected from replay packets, it sometimes marked a car demolished (and later auto-respawned it elsewhere) while the replay's pawn stayed active: 296 simulated demolition flags on each of train and validation, all corrected in the converter (`active_pawn_demo_corrections`).
- **Note.** Possibly caused by state injection (bump/supersonic timers not following the injected states) rather than by simulation itself; not isolated.

## 5. API limits for replay-driven use (limitation)

- A live arena cannot adopt an absolute tick count, RNG state, or private physics, contact and wheel caches, so a snapshot cannot be an exact continuation point. `set_car_state` requires the caller to set `is_on_ground` and wheel contacts consistently. A state restore API (or documented list of what a setter does and does not reset) would help replay tooling.

## 6. Agreement worth knowing (positive validation)

- Ball flight including gravity, drag and ground/wall bounces is reproduced to the replay's storage precision: whole-tick RocketSim stepping matches the next packet with position error p50/p99 0.005/0.013 UU and velocity 0.010/0.016 UU/s on 101,202 clean free-flight ball pairs (`audit_tick_integrality`), and airborne input-free car motion is ballistic-exact in 91.5% of 5,536 pairs (`audit_car_tick_integrality`). Roll-dominant flips match the real ramp and 5.50 rad/s roll hold; jump, dodge and aerial-torque constants were not contradicted.
- Differences that are **not** RocketSim errors: pitch-dominant and diagonal flips fade their pitch component in the replays because players cancel flips (opposite pitch input), which the replay does not carry; RocketSim models the cancel (`1 - |pitch|` on the flip pitch torque). Jump start ticks fitted against packets tend to be about 3 ticks after the jump counter's frame time with a weakly determined hold length (`diagnose_jump_timing`), which may reflect the jump force profile or what the counter marks; unresolved.

## 7. Open leads

- Ground driving: the converter's car model has a systematic velocity error (median 11-14 UU/s) and rotation p90 near 2.5 deg while driving with known throttle, steer and handbrake; not yet attributed to RocketSim's ground model versus unobserved details.

Keep this file current: add each apparent inaccuracy when found, with a reproduction command, RocketSim file and line at the pinned revision, the measured effect, the workaround, and the status.
