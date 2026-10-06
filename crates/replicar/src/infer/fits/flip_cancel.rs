//! How much of a flip's pitch the player cancelled.

use glam::{Mat3A, Vec3A};
use replicar_format::FrameIndex;
use rocketsim::{Arena, CarControls};

use super::{FitContext, seed_car, vec3};
use crate::infer::FitQuery;
use crate::update_ticks::quaternion;

/// Later updates the flip-cancel fit scores against.
const UPDATES_SCORED: usize = 1;

/// Candidate cancels (opposite pitch input of 0, 0.25, ..., 1) are simulated from the current state through
/// the next updates of the flip (the state reset to each, as the simulation does), and the one with the
/// smallest summed angular-velocity error wins; it is used for the interval to the next update. With
/// `flip_cancel_holdout` that first interval is left out of the sum. Refused for spans with a withheld or
/// inactive frame or a change of the dodge counter. `ball` is the simulation's ball at this update's tick:
/// every candidate starts from it, whatever an earlier fit left in the shared scratch arena.
pub(in crate::infer) fn fit_flip_cancel(
    ctx: FitContext,
    query: &FitQuery,
    base_controls: &CarControls,
    scratch: &mut Arena,
) -> Option<f32> {
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
    let ang0 = car.body.angular_velocity_raw.as_ref()?;
    if ang0.frame.get() != index {
        return None;
    }
    let counter = car.inputs.dodge_active_raw.as_ref()?.value;
    if !ctx.in_play(index) {
        return None;
    }
    // The flip's next updates (within 80 ticks, while the dodge counter is unchanged): the angular velocity to
    // score and the state to reset to.
    let mut targets: Vec<(i64, Vec3A, Vec3A, Mat3A, Vec3A)> = Vec::new();
    for candidate in index + 1..=(index + 24).min(frames.len() - 1) {
        let searching_more = !targets.is_empty();
        if !ctx.in_play(candidate) || ctx.withheld(candidate) {
            if searching_more {
                break;
            }
            return None;
        }
        let Some(other) = frames[candidate]
            .cars
            .iter()
            .find(|c| c.life == car.life && c.player == car.player)
        else {
            if searching_more {
                break;
            }
            return None;
        };
        let at = |f: FrameIndex| f.get() == candidate;
        let b = &other.body;
        let (Some(pos), Some(vel), Some(rot), Some(ang)) = (
            b.position.as_ref().filter(|x| at(x.frame)),
            b.linear_velocity.as_ref().filter(|x| at(x.frame)),
            b.rotation.as_ref().filter(|x| at(x.frame)),
            b.angular_velocity_raw.as_ref().filter(|x| at(x.frame)),
        ) else {
            continue;
        };
        if other.inputs.dodge_active_raw.as_ref().map(|d| d.value) != Some(counter) {
            if searching_more {
                break;
            }
            return None;
        }
        let before = match ctx.ticks {
            Some(ticks) => ticks
                .cars
                .get(&(car.life, FrameIndex(candidate as u32)))
                .copied()
                .or(ticks.car_median[candidate])
                .map_or(
                    (ctx.timeline(candidate) - ctx.timeline(candidate - 1)).max(0) / 2,
                    i64::from,
                ),
            None => 0,
        };
        let ticks =
            (ctx.timeline(candidate) - before) - (ctx.timeline(index) - ticks_before as i64);
        let previous_ticks = targets.last().map_or(0, |t| t.0);
        if !(previous_ticks + 1..=80).contains(&ticks) {
            if searching_more {
                break;
            }
            return None;
        }
        let Some(quat) = quaternion(rot.value) else {
            continue;
        };
        targets.push((
            ticks,
            vec3(pos.value),
            vec3(vel.value),
            Mat3A::from_quat(quat),
            vec3(ang.value) * 0.01,
        ));
        if targets.len() == UPDATES_SCORED {
            break;
        }
    }
    if targets.is_empty() {
        return None;
    }
    let sign = state.flip_rel_torque.y.signum();
    let first_scored = usize::from(ctx.flip_cancel_holdout && targets.len() > 1);
    let mut best: Option<(f32, f32)> = None;
    for step in 0..=4 {
        let cancel = step as f32 * 0.25;
        let mut start = *state;
        // The tick count `start` belongs to: the simulation's at the first target, the scratch arena's after.
        let mut start_tick = now_tick;
        let mut previous_ticks = 0;
        let mut total = 0.0f32;
        scratch.set_ball_state(*ball);
        for (j, target) in targets.iter().enumerate() {
            seed_car(scratch, start, start_tick);
            let mut controls = *base_controls;
            controls.jump = false;
            controls.pitch = cancel * sign;
            scratch.set_car_controls(0, controls);
            for _ in 0..(target.0 - previous_ticks) {
                scratch.step_tick();
            }
            let mut end = *scratch.get_car_state(0);
            let speed = end.phys.ang_vel.length();
            let mut clamped = end.phys.ang_vel;
            if speed > 5.5 {
                clamped *= 5.5 / speed;
            }
            if j >= first_scored {
                total += (clamped - target.4).length();
            }
            end.phys.pos = target.1;
            end.phys.vel = target.2;
            end.phys.rot_mat = target.3;
            end.phys.ang_vel = target.4;
            start = end;
            start_tick = scratch.tick_count();
            previous_ticks = target.0;
        }
        if best.is_none_or(|(_, e)| total < e - 1e-4) {
            best = Some((cancel, total));
        }
    }
    best.map(|(cancel, _)| cancel)
}
