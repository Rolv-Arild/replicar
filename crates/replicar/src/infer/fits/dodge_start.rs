//! When an airborne dodge started.

use glam::Vec3A;
use replicar_format::FrameIndex;
use rocketsim::{Arena, CarState};

use super::{DodgePlan, FitContext, same_car, seed_car, vec3};
use crate::infer::FitQuery;

/// Given an airborne car's update at `index` and a dodge counter that turns odd (with a torque) before the
/// next update, every start tick up to the second next update (both with chained ticks) is simulated, and
/// the one whose position and velocity best match that update wins; then the pitch cancel is chosen from its
/// angular velocity. The plan drives the interval to the next update only. Refused for spans with a withheld
/// or inactive frame. The ball is simulated from its state at this update's tick; other cars are not.
pub(in crate::infer) fn fit_dodge_start(
    ctx: FitContext,
    query: &FitQuery,
    scratch: &mut Arena,
) -> Option<DodgePlan> {
    let FitQuery {
        index,
        car,
        state,
        controls: base_controls,
        ticks_before,
        now: now_tick,
        ..
    } = *query;
    let ball = &query.ball;
    let frames = ctx.frames;
    let ang0 = car.body.angular_velocity_raw.as_ref()?;
    if ang0.frame.get() != index || state.is_on_ground {
        return None;
    }
    // A car that has not dodged yet has no dodge counter (it gets one at its first dodge).
    let counter = car.inputs.dodge_active_raw.as_ref().map_or(0, |d| d.value);
    if counter % 2 == 1 {
        return None;
    }
    let same_car = same_car(car);
    let last = (index + 14).min(frames.len() - 1);
    // The activation: the first frame with an odd dodge counter of its own and a torque.
    let mut activation = None;
    for g in index + 1..=last {
        if !ctx.in_play(g) || ctx.withheld(g) {
            return None;
        }
        let other = frames[g].cars.iter().find(&same_car)?;
        if let Some(dodge) = other
            .inputs
            .dodge_active_raw
            .as_ref()
            .filter(|d| d.frame.get() == g)
            && dodge.value % 2 == 1
            && let Some(torque) = crate::simulate::dodge_torque(frames, g, other)
        {
            activation = Some((g, torque));
            break;
        }
    }
    let (activation_frame, torque) = activation?;
    // Plan only from the last update before the activation: a nearer update would reset the state under a
    // plan that ignores it.
    for g in index + 1..activation_frame {
        let other = frames[g].cars.iter().find(&same_car)?;
        if other
            .body
            .position
            .as_ref()
            .is_some_and(|p| p.frame.get() == g)
        {
            return None;
        }
    }
    // The next two updates at or after the activation frame. The first is the next reset of the state, at
    // its own tick (chained or not), and is held out; the fit uses the second, which needs a chained tick.
    let ticks = ctx.ticks?;
    let origin_tick = ctx.timeline(index) - ticks_before as i64;
    let mut fresh: Vec<(u64, Vec3A, Vec3A, Vec3A)> = Vec::new();
    let mut fresh_frames: Vec<usize> = Vec::new();
    for g in activation_frame..=(activation_frame + 12).min(frames.len() - 1) {
        if !ctx.in_play(g) || ctx.withheld(g) {
            break;
        }
        let Some(other) = frames[g].cars.iter().find(&same_car) else {
            break;
        };
        let at = |f: FrameIndex| f.get() == g;
        let b = &other.body;
        let (Some(p), Some(v), Some(w)) = (
            b.position.as_ref().filter(|x| at(x.frame)),
            b.linear_velocity.as_ref().filter(|x| at(x.frame)),
            b.angular_velocity_raw.as_ref().filter(|x| at(x.frame)),
        ) else {
            continue;
        };
        let chain = ticks
            .cars
            .get(&(car.life, FrameIndex(g as u32)))
            .map(|&t| t as f32);
        let before = if fresh.is_empty() {
            chain
                .or(ticks.car_median[g].map(|t| t as f32))
                .unwrap_or((ctx.timeline(g) - ctx.timeline(g - 1)).max(0) as f32 / 2.0)
        } else {
            let Some(before) = chain else {
                continue;
            };
            before
        };
        let tick = (ctx.timeline(g) - before.round().max(0.0) as i64) - origin_tick;
        if !(1..=45).contains(&tick) {
            break;
        }
        fresh.push((
            tick as u64,
            vec3(p.value),
            vec3(v.value),
            vec3(w.value) * 0.01,
        ));
        fresh_frames.push(g);
        if fresh.len() == 2 {
            break;
        }
    }
    let [(tick_b, ..), second] = fresh[..] else {
        return None;
    };
    let first_frame = fresh_frames[0];
    let targets = [second];
    let first_fresh_state = (fresh[0].1, fresh[0].2);
    let horizon = second.0;
    let [tx, ty, _] = torque;
    let (pitch, yaw) = ((-ty / 2.24).clamp(-1.0, 1.0), (-tx / 2.60).clamp(-1.0, 1.0));
    if (pitch * pitch + yaw * yaw).sqrt() <= 0.01 {
        return None;
    }
    let mut start = *state;
    start.has_jumped = true;
    start.is_jumping = false;
    start.air_time_since_jump = start.air_time_since_jump.max(0.05);
    let mut base = *base_controls;
    base.jump = false;
    // The path with no dodge, saved tick by tick, so that each candidate resumes from its start tick.
    let mut path = vec![start];
    let mut path_ticks = vec![now_tick];
    let mut path_ball = vec![*ball];
    scratch.set_ball_state(*ball);
    seed_car(scratch, start, now_tick);
    scratch.set_car_controls(0, base);
    for _ in 0..horizon {
        scratch.step_tick();
        path.push(*scratch.get_car_state(0));
        path_ticks.push(scratch.tick_count());
        path_ball.push(*scratch.get_ball_state());
    }
    // The states at every tick from `dodge_tick` to the horizon for a dodge at `dodge_tick` with `cancel`.
    let run_all = |scratch: &mut Arena, dodge_tick: u64, cancel: f32| -> Vec<CarState> {
        let resume = dodge_tick as usize - 1;
        scratch.set_ball_state(path_ball[resume]);
        seed_car(scratch, path[resume], path_ticks[resume]);
        let mut after = Vec::new();
        for tick in dodge_tick..=horizon {
            let mut controls = base;
            if tick == dodge_tick {
                controls.jump = true;
                controls.pitch = pitch;
                controls.yaw = yaw;
                controls.roll = 0.0;
            } else {
                let sign = scratch.get_car_state(0).flip_rel_torque.y.signum();
                controls.pitch = cancel * sign;
            }
            scratch.set_car_controls(0, controls);
            scratch.step_tick();
            let mut end = *scratch.get_car_state(0);
            let speed = end.phys.ang_vel.length();
            if speed > 5.5 {
                end.phys.ang_vel *= 5.5 / speed;
            }
            after.push(end);
        }
        after
    };
    // The states at every target tick for a dodge at `dodge_tick` with `cancel`.
    let run = |scratch: &mut Arena, dodge_tick: u64, cancel: f32| -> Vec<CarState> {
        let after = run_all(scratch, dodge_tick, cancel);
        targets
            .iter()
            .map(|&(tick, ..)| {
                if tick < dodge_tick {
                    path[tick as usize]
                } else {
                    after[(tick - dodge_tick) as usize]
                }
            })
            .collect()
    };
    let position_velocity_error = |states: &[CarState]| -> f32 {
        targets
            .iter()
            .zip(states)
            .map(|(target, end)| {
                (end.phys.pos - target.1).length() + 0.1 * (end.phys.vel - target.2).length()
            })
            .sum()
    };
    let mut best: Option<(u64, f32)> = None;
    for dodge_tick in 1..=horizon {
        let error = position_velocity_error(&run(scratch, dodge_tick, 0.0));
        if best.is_none_or(|(_, e)| error < e - 1e-4) {
            best = Some((dodge_tick, error));
        }
    }
    let (dodge_tick, _) = best?;
    // A start after the next update is not driven here (that update resets the state); the trigger at the
    // activation frame applies instead.
    let mut deferred = dodge_tick > tick_b;
    let mut best_cancel: Option<(f32, f32)> = None;
    for step in 0..=4 {
        let cancel = step as f32 * 0.25;
        let states = run(scratch, dodge_tick, cancel);
        // The angular velocity counts only at updates at or after the start.
        let error: f32 = targets
            .iter()
            .zip(&states)
            .filter(|(target, _)| target.0 >= dodge_tick)
            .map(|(target, end)| (end.phys.ang_vel - target.3).length())
            .sum();
        if best_cancel.is_none_or(|(_, e)| error < e - 1e-4) {
            best_cancel = Some((cancel, error));
        }
    }
    let cancel = best_cancel?.0;
    // The first update after the activation has no chained tick (a dodge breaks the motion the chains rely
    // on), so its tick is only known to lie within the frame gap before its frame: with the start and cancel
    // fitted on the second update, the tick in that range at which the fitted path reproduces this update
    // (position and velocity) is its tick. The gap is floored at 4 ticks (30 frames per second).
    let mut first_update = None;
    if ctx.dodge_first_update_tick {
        let frame_tick = ctx.timeline(first_frame) - origin_tick;
        let gap = (ctx.timeline(first_frame) - ctx.timeline(first_frame.saturating_sub(1))).max(4);
        let (lo, hi) = (
            (frame_tick - gap).max(1),
            frame_tick.min(second.0 as i64 - 1),
        );
        if lo <= hi {
            let after = run_all(scratch, dodge_tick, cancel);
            let target = &first_fresh_state;
            let mut best_tick: Option<(u64, f32)> = None;
            for tb in lo as u64..=hi as u64 {
                let st = if tb < dodge_tick {
                    &path[tb as usize]
                } else {
                    &after[(tb - dodge_tick) as usize]
                };
                let error =
                    (st.phys.pos - target.0).length() + 0.1 * (st.phys.vel - target.1).length();
                if best_tick.is_none_or(|(_, e)| error < e - 1e-4) {
                    best_tick = Some((tb, error));
                }
            }
            if let Some((tb, _)) = best_tick {
                first_update = Some((first_frame, (frame_tick - tb as i64).max(0) as u64, tb));
                deferred = dodge_tick > tb;
            }
        }
    }
    Some(DodgePlan {
        activation_frame,
        start_offset: dodge_tick,
        duration: match first_update {
            Some((_, _, tb)) if !deferred => tb,
            _ if deferred => second.0,
            _ => tick_b,
        },
        pitch,
        yaw,
        cancel,
        first_update: first_update.map(|(frame, before, _)| (frame, before)),
    })
}
