//! A jump from the ground and the dodge that follows it, fitted together.

use glam::Vec3A;
use replicar_format::FrameIndex;
use rocketsim::{Arena, CarControls, CarState};

use super::jump_timing::{SHIFTS, control_entries, controls_at};
use super::{
    DodgePlan, FitContext, GroundFlip, GroundSchedule, network_controls, same_car, seed_car, vec3,
};
use crate::decode::{NetworkCar, NetworkValue};
use crate::infer::FitQuery;

/// Both the jump and the dodge counter turn odd before the second next update. One shift of the jump
/// counter's switches and the dodge press tick (relative to the midpoint-rule tick of the activation frame)
/// are searched on the second next update (chained tick; position and velocity), the press from a saved
/// no-dodge path per shift, then the pitch cancel from its angular velocity. The plan drives the interval to
/// the next update (the jump per tick, and the dodge if its press falls inside it). Refused for a car not on a
/// surface, changing double-jump or flip counters, spans over 45 ticks, or a withheld or inactive frame. The
/// ball is simulated from its state at this update's tick; other cars are not.
pub(in crate::infer) fn fit_ground_flip(
    ctx: FitContext,
    query: &FitQuery,
    scratch: &mut Arena,
) -> Option<GroundFlip> {
    let FitQuery {
        index,
        car,
        state,
        ticks_before,
        now: now_tick,
        ..
    } = *query;
    let ball = &query.ball;
    let frames = ctx.frames;
    let ticks = ctx.ticks?;
    if !ctx.in_play(index) || !state.is_on_ground {
        return None;
    }
    let same_car = same_car(car);
    let value = |v: &Option<NetworkValue<u8>>| v.as_ref().map(|v| v.value);
    let others = |c: &NetworkCar| {
        [
            value(&c.inputs.double_jump_active_raw),
            value(&c.inputs.flip_car_active_raw),
        ]
    };
    let jump_odd = |c: &NetworkCar| value(&c.inputs.jump_active_raw).is_some_and(|v| v % 2 == 1);
    let a_others = others(car);
    if a_others.iter().flatten().any(|c| c % 2 == 1)
        || jump_odd(car)
        || value(&car.inputs.dodge_active_raw).is_some_and(|d| d % 2 == 1)
    {
        return None;
    }
    let t_a = ctx.timeline(index) - ticks_before as i64;
    // The activation: the first frame with an odd dodge counter of its own and a torque.
    let last = (index + 14).min(frames.len() - 1);
    let mut activation = None;
    for g in index + 1..=last {
        if !ctx.in_play(g) || ctx.withheld(g) {
            return None;
        }
        let other = frames[g].cars.iter().find(&same_car)?;
        if others(other) != a_others {
            return None;
        }
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
    let [tx, ty, _] = torque;
    let (pitch, yaw) = ((-ty / 2.24).clamp(-1.0, 1.0), (-tx / 2.60).clamp(-1.0, 1.0));
    if (pitch * pitch + yaw * yaw).sqrt() <= 0.01 {
        return None;
    }
    // The next two updates after a: the first is the next reset of the state (its own tick, chained or not)
    // and stays held out; the second needs a chained tick and is the fit target.
    let mut fresh: Vec<(usize, i64, Vec3A, Vec3A, Vec3A)> = Vec::new();
    for g in index + 1..=(index + 16).min(frames.len() - 1) {
        if !ctx.in_play(g) || ctx.withheld(g) {
            return None;
        }
        let other = frames[g].cars.iter().find(&same_car)?;
        if others(other) != a_others {
            return None;
        }
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
        fresh.push((
            g,
            ctx.timeline(g) - before.round().max(0.0) as i64,
            vec3(p.value),
            vec3(v.value),
            vec3(w.value) * 0.01,
        ));
        if fresh.len() == 2 {
            break;
        }
    }
    let [
        first_fresh,
        (last_frame, t_c, target_pos, target_vel, target_ang),
    ] = fresh[..]
    else {
        return None;
    };
    let (_, t_b, ..) = first_fresh;
    let (ticks_ab, ticks_ac) = (t_b - t_a, t_c - t_a);
    if ticks_ab < 1 || ticks_ac <= ticks_ab || ticks_ac > 45 || activation_frame > last_frame {
        return None;
    }
    let entries = control_entries(ctx, index, last_frame, car);
    // The jump counter must turn odd after a and no later than the activation.
    if !entries
        .iter()
        .any(|e| e.6 && e.0 > t_a - 4 && e.0 <= ctx.timeline(activation_frame) + 4)
    {
        return None;
    }
    let act_start = entries
        .iter()
        .find(|e| e.0 == ctx.timeline(activation_frame))
        .map(|e| e.1)?;
    let own = network_controls(car);
    let own_entry = (own.throttle, own.steer, own.handbrake, own.boost);
    let controls_at = |shift: i64, tau: i64| controls_at(&entries, own_entry, shift, tau);
    let horizon = ticks_ac as usize;
    // (jump shift, press tick relative to a, cost)
    let mut best: Option<(i64, i64, f32)> = None;
    for shift in SHIFTS {
        // The path with the jump at this shift and no dodge, saved tick by tick.
        let mut path = vec![*state];
        let mut path_ticks = vec![now_tick];
        let mut path_ball = vec![*ball];
        scratch.set_ball_state(*ball);
        seed_car(scratch, *state, now_tick);
        for step in 1..=horizon {
            let c = controls_at(shift, t_a + step as i64);
            scratch.set_car_controls(
                0,
                CarControls {
                    throttle: c.0,
                    steer: c.1,
                    handbrake: c.2,
                    boost: c.3,
                    jump: c.4,
                    ..CarControls::default()
                },
            );
            scratch.step_tick();
            path.push(*scratch.get_car_state(0));
            path_ticks.push(scratch.tick_count());
            path_ball.push(*scratch.get_ball_state());
        }
        for d in SHIFTS {
            let press = act_start + d - t_a;
            if !(2..=horizon as i64).contains(&press) {
                continue;
            }
            let press = press as usize;
            scratch.set_ball_state(path_ball[press - 1]);
            seed_car(scratch, path[press - 1], path_ticks[press - 1]);
            for step in press..=horizon {
                let c = controls_at(shift, t_a + step as i64);
                let mut controls = CarControls {
                    throttle: c.0,
                    steer: c.1,
                    handbrake: c.2,
                    boost: c.3,
                    ..CarControls::default()
                };
                if step == press {
                    controls.jump = true;
                    controls.pitch = pitch;
                    controls.yaw = yaw;
                }
                scratch.set_car_controls(0, controls);
                scratch.step_tick();
            }
            let end = scratch.get_car_state(0);
            let cost =
                (end.phys.pos - target_pos).length() + 0.1 * (end.phys.vel - target_vel).length();
            if best.is_none_or(|(_, _, c)| cost < c - 1e-4) {
                best = Some((shift, press as i64, cost));
            }
        }
    }
    let (shift, press, _) = best?;
    let press_u = press as usize;
    // Simulates the whole horizon with the fitted jump, the press and `cancel`, the states at every tick.
    let run = |scratch: &mut Arena, cancel: f32| -> Vec<CarState> {
        scratch.set_ball_state(*ball);
        seed_car(scratch, *state, now_tick);
        let mut states = vec![*state];
        for step_tick in 1..=horizon {
            let c = controls_at(shift, t_a + step_tick as i64);
            let mut controls = CarControls {
                throttle: c.0,
                steer: c.1,
                handbrake: c.2,
                boost: c.3,
                jump: c.4,
                ..CarControls::default()
            };
            if step_tick == press_u {
                controls.jump = true;
                controls.pitch = pitch;
                controls.yaw = yaw;
            } else if step_tick > press_u {
                controls.jump = false;
                let sign = scratch.get_car_state(0).flip_rel_torque.y.signum();
                controls.pitch = cancel * sign;
            }
            scratch.set_car_controls(0, controls);
            scratch.step_tick();
            states.push(*scratch.get_car_state(0));
        }
        states
    };
    // The pitch cancel from the angular velocity at the second next update.
    let mut best_cancel: Option<(f32, f32)> = None;
    for step in 0..=4 {
        let cancel = step as f32 * 0.25;
        let mut end = *run(scratch, cancel).last().expect("the horizon has ticks");
        let speed = end.phys.ang_vel.length();
        if speed > 5.5 {
            end.phys.ang_vel *= 5.5 / speed;
        }
        let error = (end.phys.ang_vel - target_ang).length();
        if best_cancel.is_none_or(|(_, e)| error < e - 1e-4) {
            best_cancel = Some((cancel, error));
        }
    }
    let cancel = best_cancel?.0;
    // The first update after the activation has no chained tick: its tick is the one within 0-4 ticks before
    // its frame at which the fitted path reproduces it (position and velocity).
    let mut ticks_ab_eff = ticks_ab;
    let mut first_update = None;
    if ctx.dodge_first_update_tick && first_fresh.0 >= activation_frame {
        let states = run(scratch, cancel);
        let (frame_b, _, pos_b, vel_b, _) = first_fresh;
        let frame_tick = ctx.timeline(frame_b) - t_a;
        // Floored at 4 ticks like the dodge-start fit's window.
        let gap = (ctx.timeline(frame_b) - ctx.timeline(frame_b.saturating_sub(1))).max(4);
        let (lo, hi) = ((frame_tick - gap).max(1), frame_tick.min(ticks_ac - 1));
        let mut best_tick: Option<(i64, f32)> = None;
        for tb in lo..=hi {
            let st = &states[tb as usize];
            let error = (st.phys.pos - pos_b).length() + 0.1 * (st.phys.vel - vel_b).length();
            if best_tick.is_none_or(|(_, e)| error < e - 1e-4) {
                best_tick = Some((tb, error));
            }
        }
        if let Some((tb, _)) = best_tick {
            ticks_ab_eff = tb;
            first_update = Some((frame_b, (frame_tick - tb).max(0) as u64));
        }
    }
    // The jump schedule for the interval to the next update, up to the dodge press if it falls in it.
    let mut entries_out = Vec::new();
    let mut previous = None;
    let until = if press <= ticks_ab_eff {
        press - 1
    } else {
        ticks_ab_eff
    };
    for step in 1..=until {
        let c = controls_at(shift, t_a + step);
        if previous != Some(c) {
            entries_out.push((now_tick + step as u64, c.0, c.1, c.2, c.3, Some(c.4)));
            previous = Some(c);
        }
    }
    Some(GroundFlip {
        schedule: GroundSchedule {
            end_tick: now_tick + ticks_ab_eff as u64,
            entries: entries_out,
            shift: None,
        },
        first_update,
        dodge: Some(DodgePlan {
            activation_frame,
            start_offset: press as u64,
            duration: if press <= ticks_ab_eff {
                ticks_ab_eff as u64
            } else {
                ticks_ac as u64
            },
            pitch,
            yaw,
            cancel,
            first_update: None,
        }),
    })
}
