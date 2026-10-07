//! When the observed throttle, steer, handbrake and boost changes took effect.

use glam::Vec3A;
use replicar_format::FrameIndex;
use rocketsim::{Arena, CarControls};

use super::{FitContext, GroundSchedule, network_controls, same_car, seed_car, vec3};
use crate::decode::NetworkCar;
use crate::infer::FitQuery;

/// Shifts, in ticks later than the midpoint rule, tried for the observed control changes.
const SHIFTS: std::ops::RangeInclusive<i64> = -8..=40;

/// A control change is first seen in the frame after it happened, and each frame's state is 0-4 ticks older
/// than its time, so the change tick is uncertain by several ticks per event. For a car on a surface (floor,
/// wall, ramp or ceiling) with an update at `index`, one shift of every control switch (relative to the
/// midpoint rule) is chosen by simulating the span to the second next update in a scratch arena and comparing
/// angular velocity (per 0.3 rad/s) and velocity (per 50 UU/s) with it; the schedule covers the interval to
/// the next update. The scratch arena has the simulation's ball at this update, so a touch near it is simulated
/// (refusing within 400 UU of the ball, as before, left a third of the changes of a bot match to the rule;
/// RESULTS.md, "Inputs between frames"); other cars are not modelled. Refused without chained update ticks for
/// both later updates, with a withheld or inactive frame, an action counter change, an action in progress, or no
/// control change near the span.
pub(in crate::infer) fn fit_ground_timing(
    ctx: FitContext,
    query: &FitQuery,
    scratch: &mut Arena,
) -> Option<GroundSchedule> {
    let FitQuery {
        index,
        car,
        state,
        ticks_before,
        now: now_tick,
        ball,
        ..
    } = *query;
    let frames = ctx.frames;
    let ticks = ctx.ticks?;
    if !ctx.in_play(index) || !state.is_on_ground {
        return None;
    }
    let same_car = same_car(car);
    let counters = |c: &NetworkCar| {
        [
            c.inputs.jump_active_raw.as_ref().map(|v| v.value),
            c.inputs.double_jump_active_raw.as_ref().map(|v| v.value),
            c.inputs.dodge_active_raw.as_ref().map(|v| v.value),
            c.inputs.flip_car_active_raw.as_ref().map(|v| v.value),
        ]
    };
    let a_counters = counters(car);
    if a_counters.iter().flatten().any(|c| c % 2 == 1) {
        return None;
    }
    let t_a = ctx.timeline(index) - ticks_before as i64;
    // The next two updates with chained ticks.
    let mut found: Vec<(usize, i64, Vec3A, Vec3A)> = Vec::new();
    for g in index + 1..=(index + 12).min(frames.len() - 1) {
        if !ctx.in_play(g) || ctx.withheld(g) {
            return None;
        }
        let other = frames[g].cars.iter().find(&same_car)?;
        if counters(other) != a_counters {
            return None;
        }
        let at = |f: FrameIndex| f.get() == g;
        let b = &other.body;
        let (Some(_), Some(v), Some(w)) = (
            b.position.as_ref().filter(|x| at(x.frame)),
            b.linear_velocity.as_ref().filter(|x| at(x.frame)),
            b.angular_velocity_raw.as_ref().filter(|x| at(x.frame)),
        ) else {
            continue;
        };
        let Some(&before) = ticks.cars.get(&(car.life, FrameIndex(g as u32))) else {
            continue;
        };
        found.push((
            g,
            ctx.timeline(g) - i64::from(before),
            vec3(v.value),
            vec3(w.value) * 0.01,
        ));
        if found.len() == 2 {
            break;
        }
    }
    let [first, second] = found[..] else {
        return None;
    };
    let (_, t_b, _, _) = first;
    let (last_frame, t_c, target_vel, target_ang) = if ctx.fit_on_next_update {
        first
    } else {
        second
    };
    let (ticks_ab, ticks_ac) = (t_b - t_a, t_c - t_a);
    if ticks_ab < 1 || (ticks_ac <= ticks_ab && !ctx.fit_on_next_update) || ticks_ac > 24 {
        return None;
    }
    // The observed controls of the frames around the span, at their midpoint-rule switch ticks.
    let mut entries: Vec<(i64, f32, f32, bool, bool)> = Vec::new();
    for g in index.saturating_sub(3)..=(last_frame + 1).min(frames.len() - 1) {
        if !ctx.in_play(g) {
            continue;
        }
        let Some(other) = frames[g].cars.iter().find(&same_car) else {
            continue;
        };
        let controls = network_controls(other);
        entries.push((
            ctx.control_change_tick(g),
            controls.throttle,
            controls.steer,
            controls.handbrake,
            controls.boost,
        ));
    }
    let own = network_controls(car);
    let own_entry = (t_a, own.throttle, own.steer, own.handbrake, own.boost);
    let changes = entries
        .iter()
        .filter(|e| e.0 >= t_a - 16 && e.0 <= t_c + 16)
        .collect::<Vec<_>>()
        .windows(2)
        .any(|w| {
            (w[0].1 - w[1].1).abs() > 0.1 || (w[0].2 - w[1].2).abs() > 0.1 || w[0].3 != w[1].3
        });
    if !changes {
        return None;
    }
    let controls_at = |shift: i64, tau: i64| {
        let i = entries.partition_point(|e| e.0 + shift <= tau);
        if i == 0 { own_entry } else { entries[i - 1] }
    };
    // The scratch arena holds this car and the ball as the simulation has it; other cars are not modelled.
    let scratch_ball = ball;
    // Shifts whose switches fall on the same ticks of the span give the same simulation: one cost each.
    let mut costs: Vec<(i64, f32)> = Vec::new();
    let mut simulated: Vec<(Vec<u32>, f32)> = Vec::new();
    for shift in SHIFTS {
        let key: Vec<u32> = (t_a + 1..=t_c)
            .map(|tau| entries.partition_point(|e| e.0 + shift <= tau) as u32)
            .collect();
        if let Some((_, cost)) = simulated.iter().find(|(k, _)| *k == key) {
            costs.push((shift, *cost));
            continue;
        }
        scratch.set_ball_state(scratch_ball);
        seed_car(scratch, *state, now_tick);
        for tau in t_a + 1..=t_c {
            let e = controls_at(shift, tau);
            scratch.set_car_controls(
                0,
                CarControls {
                    throttle: e.1,
                    steer: e.2,
                    handbrake: e.3,
                    boost: e.4,
                    ..CarControls::default()
                },
            );
            scratch.step_tick();
        }
        let end = scratch.get_car_state(0);
        let cost = (end.phys.ang_vel - target_ang).length() / 0.3
            + (end.phys.vel - target_vel).length() / 50.0;
        simulated.push((key, cost));
        costs.push((shift, cost));
    }
    // The midpoint rule (shift 0) unless another shift is strictly better.
    let mut best = costs.iter().position(|(shift, _)| *shift == 0)?;
    for (i, (_, cost)) in costs.iter().enumerate() {
        if *cost < costs[best].1 - 1e-6 {
            best = i;
        }
    }
    let shift = costs[best].0;
    // The schedule for the interval to the next update, in sim ticks.
    let mut schedule = vec![(
        now_tick,
        own_entry.1,
        own_entry.2,
        own_entry.3,
        own_entry.4,
        None,
    )];
    for e in &entries {
        let tick = now_tick as i64 + (e.0 + shift - t_a);
        if tick > now_tick as i64 {
            schedule.push((tick as u64, e.1, e.2, e.3, e.4, None));
        } else {
            // A switch at or before the update replaces the starting controls.
            schedule[0] = (now_tick, e.1, e.2, e.3, e.4, None);
        }
    }
    let zero_cost = costs.iter().find(|(s, _)| *s == 0).map(|c| c.1);
    Some(GroundSchedule {
        end_tick: now_tick + ticks_ab as u64,
        entries: schedule,
        shift: zero_cost
            .filter(|&z| costs[best].1 < z - 1e-6)
            .map(|_| shift),
    })
}
