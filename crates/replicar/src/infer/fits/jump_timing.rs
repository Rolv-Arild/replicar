//! When a jump started.

use glam::Vec3A;
use replicar_format::FrameIndex;
use rocketsim::{Arena, CarControls};

use super::{FitContext, GroundSchedule, network_controls, same_car, seed_car, vec3};
use crate::decode::NetworkCar;
use crate::infer::FitQuery;

/// Shifts, in ticks later than the midpoint rule, tried for the jump counter's switches.
pub(super) const SHIFTS: std::ops::RangeInclusive<i64> = -8..=16;

/// The jump counter turns odd in the frame after the press, and each frame's state is 0-4 ticks older than its
/// time; the fitted start is 0-3 ticks after the midpoint rule in two thirds of the events and up to 12 later
/// in the rest. For a car on a surface with an update at `index` and an even jump counter that turns odd
/// before the second next update, one shift of the counter's switches (press and release together) is chosen
/// by simulating the span to that update (position error plus 0.1 x velocity error); the interval to the next
/// update is driven with it, the other controls on the midpoint rule. Refused without chained update ticks for
/// both later updates, for spans over 30 ticks, a withheld or inactive frame, or a change of the double-jump,
/// dodge or flip counter. The ball is simulated from its state at this update's tick; other cars are not.
pub(in crate::infer) fn fit_jump_timing(
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
        ..
    } = *query;
    let ball = &query.ball;
    let frames = ctx.frames;
    let ticks = ctx.ticks?;
    if !ctx.in_play(index) || !state.is_on_ground {
        return None;
    }
    let same_car = same_car(car);
    let others = |c: &NetworkCar| {
        [
            c.inputs.double_jump_active_raw.as_ref().map(|v| v.value),
            c.inputs.dodge_active_raw.as_ref().map(|v| v.value),
            c.inputs.flip_car_active_raw.as_ref().map(|v| v.value),
        ]
    };
    let a_others = others(car);
    let jump_odd = |c: &NetworkCar| {
        c.inputs
            .jump_active_raw
            .as_ref()
            .is_some_and(|v| v.value % 2 == 1)
    };
    if a_others.iter().flatten().any(|c| c % 2 == 1) || jump_odd(car) {
        return None;
    }
    let t_a = ctx.timeline(index) - ticks_before as i64;
    let mut found: Vec<(usize, i64, Vec3A, Vec3A)> = Vec::new();
    for g in index + 1..=(index + 12).min(frames.len() - 1) {
        if !ctx.in_play(g) || ctx.withheld(g) {
            return None;
        }
        let other = frames[g].cars.iter().find(&same_car)?;
        if others(other) != a_others {
            return None;
        }
        let at = |f: FrameIndex| f.get() == g;
        let b = &other.body;
        let (Some(p), Some(v)) = (
            b.position.as_ref().filter(|x| at(x.frame)),
            b.linear_velocity.as_ref().filter(|x| at(x.frame)),
        ) else {
            continue;
        };
        let Some(&before) = ticks.cars.get(&(car.life, FrameIndex(g as u32))) else {
            continue;
        };
        found.push((
            g,
            ctx.timeline(g) - i64::from(before),
            vec3(p.value),
            vec3(v.value),
        ));
        if found.len() == 2 {
            break;
        }
    }
    let [first, second] = found[..] else {
        return None;
    };
    let (_, t_b, _, _) = first;
    let (last_frame, t_c, target_pos, target_vel) = if ctx.fit_on_next_update {
        first
    } else {
        second
    };
    let (ticks_ab, ticks_ac) = (t_b - t_a, t_c - t_a);
    if ticks_ab < 1 || (ticks_ac <= ticks_ab && !ctx.fit_on_next_update) || ticks_ac > 30 {
        return None;
    }
    let entries = control_entries(ctx, index, last_frame, car);
    // The counter must turn odd after a and no later than the second next update.
    if !entries
        .iter()
        .any(|e| e.6 && e.0 > t_a - 4 && e.0 <= t_c + 4)
    {
        return None;
    }
    let own = network_controls(car);
    let own_entry = (own.throttle, own.steer, own.handbrake, own.boost);
    let controls_at = |shift: i64, tau: i64| controls_at(&entries, own_entry, shift, tau);
    let mut costs: Vec<(i64, f32)> = Vec::new();
    for shift in SHIFTS {
        scratch.set_ball_state(*ball);
        seed_car(scratch, *state, now_tick);
        for tau in t_a + 1..=t_c {
            let c = controls_at(shift, tau);
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
        }
        let end = scratch.get_car_state(0);
        let cost =
            (end.phys.pos - target_pos).length() + 0.1 * (end.phys.vel - target_vel).length();
        costs.push((shift, cost));
    }
    // The midpoint rule (shift 0) unless another shift is strictly better.
    let mut best = costs.iter().position(|(shift, _)| *shift == 0)?;
    for (i, (_, cost)) in costs.iter().enumerate() {
        if *cost < costs[best].1 - 1e-4 {
            best = i;
        }
    }
    let shift = costs[best].0;
    // The schedule for the interval to the next update, in sim ticks.
    let mut schedule = Vec::new();
    let mut previous = None;
    for step in 1..=ticks_ab {
        let c = controls_at(shift, t_a + step);
        if previous != Some(c) {
            schedule.push((now_tick + step as u64, c.0, c.1, c.2, c.3, Some(c.4)));
            previous = Some(c);
        }
    }
    Some(GroundSchedule {
        end_tick: now_tick + ticks_ab as u64,
        entries: schedule,
        shift: None,
    })
}

/// A frame's controls for the jump fits: (replay tick, midpoint-rule switch tick, throttle, steer, handbrake,
/// boost, jump counter odd).
pub(super) type JumpEntry = (i64, i64, f32, f32, bool, bool, bool);

/// The controls of the frames from three before `index` to one after `last_frame`, in play.
pub(super) fn control_entries(
    ctx: FitContext,
    index: usize,
    last_frame: usize,
    car: &NetworkCar,
) -> Vec<JumpEntry> {
    let same_car = same_car(car);
    let mut entries = Vec::new();
    for g in index.saturating_sub(3)..=(last_frame + 1).min(ctx.frames.len() - 1) {
        if !ctx.in_play(g) {
            continue;
        }
        let Some(other) = ctx.frames[g].cars.iter().find(&same_car) else {
            continue;
        };
        let controls = network_controls(other);
        let jump = other
            .inputs
            .jump_active_raw
            .as_ref()
            .is_some_and(|v| v.value % 2 == 1);
        entries.push((
            ctx.timeline(g),
            ctx.control_change_tick(g),
            controls.throttle,
            controls.steer,
            controls.handbrake,
            controls.boost,
            jump,
        ));
    }
    entries
}

/// The controls at replay tick `tau` with the jump counter's switches moved by `shift`: (throttle, steer,
/// handbrake, boost, jump).
pub(super) fn controls_at(
    entries: &[JumpEntry],
    own: (f32, f32, bool, bool),
    shift: i64,
    tau: i64,
) -> (f32, f32, bool, bool, bool) {
    let i = entries.partition_point(|e| e.1 <= tau);
    let base = if i == 0 {
        own
    } else {
        let e = entries[i - 1];
        (e.2, e.3, e.4, e.5)
    };
    let j = entries.partition_point(|e| e.1 + shift <= tau);
    let jump = j > 0 && entries[j - 1].6;
    (base.0, base.1, base.2, base.3, jump)
}
