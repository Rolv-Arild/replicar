//! Offline input-timing fits in scratch arenas, scored on later packets: ground control timing,
//! jump timing, a jump followed by a dodge, the dodge start, and the flip's pitch cancel.

use super::*;

/// The physical tick (timeline) from which the controls first seen in frame `g` act: `2 + spacing / 2`
/// ticks before the frame time.
pub(super) fn control_change_tick(observations: &ObservedReplay, first_time: f32, g: usize) -> i64 {
    let frames = &observations.frames;
    let timeline = |frame: usize| -> i64 {
        ((f64::from(frames[frame].time) - f64::from(first_time)) * 120.0).round() as i64
    };
    let spacing = if g == 0 {
        4
    } else {
        timeline(g) - timeline(g - 1)
    };
    timeline(g) - 2 - spacing / 2
}

/// Shifts, in ticks later than the midpoint rule, tried for the observed control changes.
pub(super) const GROUND_TIMING_SHIFTS: std::ops::RangeInclusive<i64> = -8..=40;

/// Fits when the observed throttle, steer, handbrake and boost changes took effect. A control change
/// is first seen in the frame after it happened and each frame's state is 0-4 ticks older than its
/// time, so the change tick is uncertain by several ticks per event (`diagnose_control_latency`
/// shows the best shift is spread over the whole range and does not carry from one interval to
/// the next). For a grounded car with a fresh packet at `index`, one common shift of every control
/// switch (relative to the midpoint rule of the lookahead ground controls) is chosen by simulating the
/// span to the *second* next fresh packet in a scratch arena and comparing angular velocity (per
/// 0.3 rad/s) and velocity (per 50 UU/s) with it. The returned schedule covers only the interval
/// to the *next* fresh packet, so that packet is not used by the fit and the residual there stays a
/// held-out check. Uses later packets (offline reconstruction). Refused without exact chain lags for
/// both later packets, with a withheld or inactive frame, a jump/dodge counter change, a car that
/// is not on a surface at the first packet (any surface: floor, wall, ramp or ceiling), or the ball
/// within 400 UU (the scratch
/// arena's ball is parked; other cars are not modelled either, and refusing spans near them removed
/// coverage without protecting the fit: near-other-car velocity p90 65.5 to 54.5 UU/s without it).
#[allow(clippy::too_many_arguments)]
pub(super) fn fit_ground_control_timing(
    observations: &ObservedReplay,
    options: &ConvertOptions,
    packet_lags: &Option<PacketLags>,
    first_time: f32,
    index: usize,
    car: &observations::Car,
    state: &CarState,
    lag_a: u64,
    slot: usize,
    now_tick: u64,
    scratch: &mut Arena,
) -> Option<GroundSchedule> {
    let frames = &observations.frames;
    let lags = packet_lags.as_ref()?;
    let timeline = |frame: usize| -> i64 {
        ((f64::from(frames[frame].time) - f64::from(first_time)) * 120.0).round() as i64
    };
    let active = |frame: usize| {
        frames[frame]
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active")
    };
    let withheld = |frame: usize| {
        options
            .withheld_frames
            .as_ref()
            .is_some_and(|w| w.get(frame).copied().unwrap_or(false))
    };
    // On any surface (floor, wall, ramp or ceiling); the counters rule out a jump or dodge in the span.
    if !active(index) || !state.is_on_ground {
        return None;
    }
    let same_car = |c: &&observations::Car| {
        c.actor_id == car.actor_id
            && c.actor_created_frame == car.actor_created_frame
            && c.player_key == car.player_key
    };
    let counters = |c: &observations::Car| {
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
    let clear = |g: usize, pos: Vec3A| {
        frames[g].ball.as_ref().is_none_or(|ball| {
            ball.position
                .as_ref()
                .is_none_or(|p| (vec3(p.value) - pos).length() > 400.0)
        })
    };
    if !clear(index, state.phys.pos) {
        return None;
    }
    let t_a = timeline(index) - lag_a as i64;
    // The next two fresh packets with exact chain lags.
    let mut found: Vec<(usize, i64, Vec3A, Vec3A)> = Vec::new();
    for g in index + 1..=(index + 12).min(frames.len() - 1) {
        if !active(g) || withheld(g) {
            return None;
        }
        let other = frames[g].cars.iter().find(same_car)?;
        if counters(other) != a_counters {
            return None;
        }
        let b = &other.body;
        let (Some(p), Some(v), Some(w)) = (
            b.position.as_ref().filter(|x| x.frame == g),
            b.linear_velocity.as_ref().filter(|x| x.frame == g),
            b.angular_velocity_replay_units
                .as_ref()
                .filter(|x| x.frame == g),
        ) else {
            continue;
        };
        let Some(lag) = lags
            .car_actor
            .get(&(car.actor_id, car.actor_created_frame, g))
            .copied()
        else {
            continue;
        };
        if !clear(g, vec3(p.value)) {
            return None;
        }
        found.push((
            g,
            timeline(g) - lag.round().max(0.0) as i64,
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
    let (last_frame, t_c, target_vel, target_ang) = if options.fit_on_next_packet {
        first
    } else {
        second
    };
    let (ticks_ab, ticks_ac) = (t_b - t_a, t_c - t_a);
    if ticks_ab < 1 || (ticks_ac <= ticks_ab && !options.fit_on_next_packet) || ticks_ac > 24 {
        return None;
    }
    // Observed controls of the frames around the span, with midpoint-rule switch ticks.
    let mut entries: Vec<(i64, f32, f32, bool, bool)> = Vec::new();
    for g in index.saturating_sub(3)..=(last_frame + 1).min(frames.len() - 1) {
        if !active(g) {
            continue;
        }
        let Some(other) = frames[g].cars.iter().find(same_car) else {
            continue;
        };
        let controls = controls_from_observation(other);
        entries.push((
            control_change_tick(observations, first_time, g),
            controls.throttle,
            controls.steer,
            controls.handbrake,
            controls.boost,
        ));
    }
    let own = controls_from_observation(car);
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
    let controls_at = |entries: &[(i64, f32, f32, bool, bool)], shift: i64, tau: i64| {
        let i = entries.partition_point(|e| e.0 + shift <= tau);
        if i == 0 { own_entry } else { entries[i - 1] }
    };
    let mut costs: Vec<(i64, f32)> = Vec::new();
    // The scratch arena holds only this car: the ball is parked out of reach so that it cannot
    // touch the car (the fit uses spans with no ball or car nearby).
    let mut parked = rocketsim::BallState::default();
    parked.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
    if (state.phys.pos - parked.phys.pos).length() < 600.0 {
        // A car on the ceiling would touch the parked ball.
        parked.phys.pos = Vec3A::new(3000.0, 4000.0, 300.0);
    }
    // Shifts whose control switches fall on the same ticks of the span (or all outside it) give the
    // same simulation, so the cost is computed once per distinct schedule.
    let mut simulated: Vec<(Vec<u32>, f32)> = Vec::new();
    for shift in GROUND_TIMING_SHIFTS {
        let schedule_key: Vec<u32> = (t_a + 1..=t_c)
            .map(|tau| entries.partition_point(|e| e.0 + shift <= tau) as u32)
            .collect();
        if let Some((_, cost)) = simulated.iter().find(|(key, _)| *key == schedule_key) {
            costs.push((shift, *cost));
            continue;
        }
        scratch.set_ball_state(parked);
        seed_scratch_car(scratch, *state, now_tick);
        for tau in t_a + 1..=t_c {
            let e = controls_at(&entries, shift, tau);
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
        simulated.push((schedule_key, cost));
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
    // The schedule for the interval to the next packet, in arena ticks.
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
            // A switch at or before the packet replaces the starting controls.
            schedule[0] = (now_tick, e.1, e.2, e.3, e.4, None);
        }
    }
    let zero_cost = costs.iter().find(|(s, _)| *s == 0).map(|c| c.1);
    Some(GroundSchedule {
        slot,
        end_tick: now_tick + ticks_ab as u64,
        entries: schedule,
        shift: zero_cost
            .filter(|&z| costs[best].1 < z - 1e-6)
            .map(|_| shift),
    })
}

/// Shifts, in ticks later than the midpoint rule, tried for the jump counter's switches.
pub(super) const JUMP_TIMING_SHIFTS: std::ops::RangeInclusive<i64> = -8..=16;

/// Fits when a jump physically started. The jump counter turns odd in the frame after the press was
/// applied and each frame's state is 0-4 ticks older than its time, and the two windows do not
/// line up: the fitted start is at 0-3 ticks after the midpoint-rule tick in two thirds of the
/// events and up to 12 ticks later in the rest, per event (`diagnose_jump_latency`: a per-player
/// median of other events' shifts does not remove the tail, while a per-event fit does). For a car
/// on a surface (floor, wall, ramp or ceiling) with a fresh packet at `index` and an even jump counter that turns odd before
/// the second next fresh packet, one shift of the jump counter's switches (press and release move
/// together) is chosen by simulating the span to that packet in a scratch arena (position error plus
/// 0.1 x velocity error), and the interval to the *next* fresh packet is driven with it, so that
/// packet is not used by the fit and its residual stays a held-out check. Other controls use the
/// midpoint rule. Uses later packets (offline reconstruction). Refused without exact chain lags for
/// both later packets, spans over 30 ticks, a withheld or inactive frame, a change of double-jump,
/// dodge or flip counter. The ball is simulated (its state at this packet's time in the main
/// arena), so jumps at the ball are fitted with their contacts; other cars are not modelled.
#[allow(clippy::too_many_arguments)]
pub(super) fn fit_jump_timing(
    observations: &ObservedReplay,
    options: &ConvertOptions,
    packet_lags: &Option<PacketLags>,
    first_time: f32,
    index: usize,
    car: &observations::Car,
    state: &CarState,
    lag_a: u64,
    slot: usize,
    now_tick: u64,
    ball: &rocketsim::BallState,
    scratch: &mut Arena,
) -> Option<GroundSchedule> {
    let frames = &observations.frames;
    let lags = packet_lags.as_ref()?;
    let timeline = |frame: usize| -> i64 {
        ((f64::from(frames[frame].time) - f64::from(first_time)) * 120.0).round() as i64
    };
    let active = |frame: usize| {
        frames[frame]
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active")
    };
    let withheld = |frame: usize| {
        options
            .withheld_frames
            .as_ref()
            .is_some_and(|w| w.get(frame).copied().unwrap_or(false))
    };
    // On any surface (floor, wall, ramp or ceiling): a jump leaves it along the surface normal.
    if !active(index) || !state.is_on_ground {
        return None;
    }
    let same_car = |c: &&observations::Car| {
        c.actor_id == car.actor_id
            && c.actor_created_frame == car.actor_created_frame
            && c.player_key == car.player_key
    };
    let others = |c: &observations::Car| {
        [
            c.inputs.double_jump_active_raw.as_ref().map(|v| v.value),
            c.inputs.dodge_active_raw.as_ref().map(|v| v.value),
            c.inputs.flip_car_active_raw.as_ref().map(|v| v.value),
        ]
    };
    let a_others = others(car);
    let jump_odd = |c: &observations::Car| {
        c.inputs
            .jump_active_raw
            .as_ref()
            .is_some_and(|v| v.value % 2 == 1)
    };
    if a_others.iter().flatten().any(|c| c % 2 == 1) || jump_odd(car) {
        return None;
    }
    let t_a = timeline(index) - lag_a as i64;
    let mut found: Vec<(usize, i64, Vec3A, Vec3A)> = Vec::new();
    for g in index + 1..=(index + 12).min(frames.len() - 1) {
        if !active(g) || withheld(g) {
            return None;
        }
        let other = frames[g].cars.iter().find(same_car)?;
        if others(other) != a_others {
            return None;
        }
        let b = &other.body;
        let (Some(p), Some(v)) = (
            b.position.as_ref().filter(|x| x.frame == g),
            b.linear_velocity.as_ref().filter(|x| x.frame == g),
        ) else {
            continue;
        };
        let Some(lag) = lags
            .car_actor
            .get(&(car.actor_id, car.actor_created_frame, g))
            .copied()
        else {
            continue;
        };
        found.push((
            g,
            timeline(g) - lag.round().max(0.0) as i64,
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
    let (last_frame, t_c, target_pos, target_vel) = if options.fit_on_next_packet {
        first
    } else {
        second
    };
    let (ticks_ab, ticks_ac) = (t_b - t_a, t_c - t_a);
    if ticks_ab < 1 || (ticks_ac <= ticks_ab && !options.fit_on_next_packet) || ticks_ac > 30 {
        return None;
    }
    // (nominal tick, midpoint-rule start tick, throttle, steer, handbrake, boost, jump) per frame.
    let mut entries: Vec<(i64, i64, f32, f32, bool, bool, bool)> = Vec::new();
    for g in index.saturating_sub(3)..=(last_frame + 1).min(frames.len() - 1) {
        if !active(g) {
            continue;
        }
        let Some(other) = frames[g].cars.iter().find(same_car) else {
            continue;
        };
        let mut controls = controls_from_observation(other);
        controls.jump = jump_odd(other);
        entries.push((
            timeline(g),
            control_change_tick(observations, first_time, g),
            controls.throttle,
            controls.steer,
            controls.handbrake,
            controls.boost,
            controls.jump,
        ));
    }
    // The counter must turn odd after a and no later than the second next packet.
    if !entries
        .iter()
        .any(|e| e.6 && e.0 > t_a - 4 && e.0 <= t_c + 4)
    {
        return None;
    }
    let own = controls_from_observation(car);
    let own_entry = (own.throttle, own.steer, own.handbrake, own.boost);
    // Controls at arena-independent tick `tau` for a jump shift `shift`.
    let controls_at = |shift: i64, tau: i64| -> (f32, f32, bool, bool, bool) {
        let i = entries.partition_point(|e| e.1 <= tau);
        let base = if i == 0 {
            own_entry
        } else {
            let e = entries[i - 1];
            (e.2, e.3, e.4, e.5)
        };
        let j = entries.partition_point(|e| e.1 + shift <= tau);
        let jump = j > 0 && entries[j - 1].6;
        (base.0, base.1, base.2, base.3, jump)
    };
    let mut costs: Vec<(i64, f32)> = Vec::new();
    for shift in JUMP_TIMING_SHIFTS {
        scratch.set_ball_state(*ball);
        seed_scratch_car(scratch, *state, now_tick);
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
    // Per-tick schedule for the interval to the next packet, in arena ticks.
    let mut schedule: Vec<(u64, f32, f32, bool, bool, Option<bool>)> = Vec::new();
    let mut previous = None;
    for step in 1..=ticks_ab {
        let c = controls_at(shift, t_a + step);
        if previous != Some(c) {
            schedule.push((now_tick + step as u64, c.0, c.1, c.2, c.3, Some(c.4)));
            previous = Some(c);
        }
    }
    Some(GroundSchedule {
        slot,
        end_tick: now_tick + ticks_ab as u64,
        entries: schedule,
        shift: None,
    })
}

/// A ground jump followed by a dodge, fitted together: the jump schedule for the interval to the next
/// fresh packet and, when the dodge press falls inside it, the dodge plan.
pub(super) struct FlipFit {
    pub(super) schedule: GroundSchedule,
    pub(super) dodge: Option<DodgePlan>,
    /// The first fresh packet after the activation and the lag (ticks) inferred for it.
    pub(super) first_packet: Option<(usize, u64)>,
}

/// Fits a jump from the ground and the dodge that follows it (both counters turn odd before the
/// second-next fresh packet). One shift of the jump counter's switches and the dodge press tick
/// (relative to the midpoint-rule tick of the activation frame) are searched on the *second* next
/// fresh packet (exact chain lag; position and velocity), the dodge press from a saved no-dodge
/// path per jump shift, then the pitch cancel from its angular velocity. The plan drives only the
/// interval to the *next* fresh packet (the jump input per tick, and the dodge if its press falls
/// inside it), so that packet is not used by the fit. Uses later packets (offline reconstruction).
/// Refused for a car not on a surface, uneven double-jump or flip counters, spans over 45
/// ticks, or a withheld or inactive frame. The ball is simulated (its state at this packet's time in
/// the main arena); other cars are not.
#[allow(clippy::too_many_arguments)]
pub(super) fn fit_ground_flip_timing(
    observations: &ObservedReplay,
    options: &ConvertOptions,
    packet_lags: &Option<PacketLags>,
    first_time: f32,
    index: usize,
    car: &observations::Car,
    state: &CarState,
    lag_a: u64,
    slot: usize,
    now_tick: u64,
    ball: &rocketsim::BallState,
    scratch: &mut Arena,
) -> Option<FlipFit> {
    let frames = &observations.frames;
    let lags = packet_lags.as_ref()?;
    let timeline = |frame: usize| -> i64 {
        ((f64::from(frames[frame].time) - f64::from(first_time)) * 120.0).round() as i64
    };
    let active = |frame: usize| {
        frames[frame]
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active")
    };
    let withheld = |frame: usize| {
        options
            .withheld_frames
            .as_ref()
            .is_some_and(|w| w.get(frame).copied().unwrap_or(false))
    };
    if !active(index) || !state.is_on_ground {
        return None;
    }
    let same_car = |c: &&observations::Car| {
        c.actor_id == car.actor_id
            && c.actor_created_frame == car.actor_created_frame
            && c.player_key == car.player_key
    };
    let parity = |v: &Option<observations::Value<u8>>| v.as_ref().map(|v| v.value);
    let others = |c: &observations::Car| {
        [
            parity(&c.inputs.double_jump_active_raw),
            parity(&c.inputs.flip_car_active_raw),
        ]
    };
    let jump_odd =
        |c: &observations::Car| parity(&c.inputs.jump_active_raw).is_some_and(|v| v % 2 == 1);
    let dodge_of = |c: &observations::Car| parity(&c.inputs.dodge_active_raw);
    let a_others = others(car);
    if a_others.iter().flatten().any(|c| c % 2 == 1)
        || jump_odd(car)
        || dodge_of(car).is_some_and(|d| d % 2 == 1)
    {
        return None;
    }
    let t_a = timeline(index) - lag_a as i64;
    // The dodge activation: the first frame with a fresh odd dodge counter and a fresh torque.
    let last = (index + 14).min(frames.len() - 1);
    let mut activation = None;
    for g in index + 1..=last {
        if !active(g) || withheld(g) {
            return None;
        }
        let other = frames[g].cars.iter().find(same_car)?;
        if others(other) != a_others {
            return None;
        }
        if let Some(dodge) = other
            .inputs
            .dodge_active_raw
            .as_ref()
            .filter(|d| d.frame == g)
            && dodge.value % 2 == 1
            && let Some(torque) = activation_torque(frames, g, other)
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
    // The next two fresh packets after a: the first is the next reset of the state (its own lag,
    // exact or not) and stays held out; the second needs an exact chain lag and is the fit target.
    let mut fresh: Vec<(usize, i64, Vec3A, Vec3A, Vec3A)> = Vec::new();
    for g in index + 1..=(index + 16).min(frames.len() - 1) {
        if !active(g) || withheld(g) {
            return None;
        }
        let other = frames[g].cars.iter().find(same_car)?;
        if others(other) != a_others {
            return None;
        }
        let b = &other.body;
        let (Some(p), Some(v), Some(w)) = (
            b.position.as_ref().filter(|x| x.frame == g),
            b.linear_velocity.as_ref().filter(|x| x.frame == g),
            b.angular_velocity_replay_units
                .as_ref()
                .filter(|x| x.frame == g),
        ) else {
            continue;
        };
        let chain = lags
            .car_actor
            .get(&(car.actor_id, car.actor_created_frame, g))
            .copied();
        let lag = if fresh.is_empty() {
            chain
                .or(lags.cars[g])
                .unwrap_or((timeline(g) - timeline(g - 1)).max(0) as f32 / 2.0)
        } else {
            let Some(lag) = chain else {
                continue;
            };
            lag
        };
        fresh.push((
            g,
            timeline(g) - lag.round().max(0.0) as i64,
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
    if ticks_ab < 1 {
        return None;
    }
    if ticks_ac <= ticks_ab {
        return None;
    }
    if ticks_ac > 45 {
        return None;
    }
    if activation_frame > last_frame {
        return None;
    }
    // (nominal tick, midpoint-rule start tick, throttle, steer, handbrake, boost, jump) per frame.
    let mut entries: Vec<(i64, i64, f32, f32, bool, bool, bool)> = Vec::new();
    for g in index.saturating_sub(3)..=(last_frame + 1).min(frames.len() - 1) {
        if !active(g) {
            continue;
        }
        let Some(other) = frames[g].cars.iter().find(same_car) else {
            continue;
        };
        let mut controls = controls_from_observation(other);
        controls.jump = jump_odd(other);
        entries.push((
            timeline(g),
            control_change_tick(observations, first_time, g),
            controls.throttle,
            controls.steer,
            controls.handbrake,
            controls.boost,
            controls.jump,
        ));
    }
    // The jump counter must turn odd after a and no later than the activation.
    if !entries
        .iter()
        .any(|e| e.6 && e.0 > t_a - 4 && e.0 <= timeline(activation_frame) + 4)
    {
        return None;
    }
    let act_start = entries
        .iter()
        .find(|e| e.0 == timeline(activation_frame))
        .map(|e| e.1)?;
    let own = controls_from_observation(car);
    let own_entry = (own.throttle, own.steer, own.handbrake, own.boost);
    let controls_at = |shift: i64, tau: i64| -> (f32, f32, bool, bool, bool) {
        let i = entries.partition_point(|e| e.1 <= tau);
        let base = if i == 0 {
            own_entry
        } else {
            let e = entries[i - 1];
            (e.2, e.3, e.4, e.5)
        };
        let j = entries.partition_point(|e| e.1 + shift <= tau);
        let jump = j > 0 && entries[j - 1].6;
        (base.0, base.1, base.2, base.3, jump)
    };
    let horizon = ticks_ac as usize;
    let mut best: Option<(i64, i64, f32)> = None; // (jump shift, press tick relative to a, cost)
    for shift in JUMP_TIMING_SHIFTS {
        // The path with the jump at this shift and no dodge, saved tick by tick.
        let mut path = vec![*state];
        let mut path_ticks = vec![now_tick];
        let mut path_ball = vec![*ball];
        scratch.set_ball_state(*ball);
        seed_scratch_car(scratch, *state, now_tick);
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
        for d in JUMP_TIMING_SHIFTS {
            let press = act_start + d - t_a;
            if !(2..=horizon as i64).contains(&press) {
                continue;
            }
            let press = press as usize;
            scratch.set_ball_state(path_ball[press - 1]);
            seed_scratch_car(scratch, path[press - 1], path_ticks[press - 1]);
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
    // The pitch cancel from the angular velocity at the second-next packet.
    let press_u = press as usize;
    let mut best_cancel: Option<(f32, f32)> = None;
    for step in 0..=4 {
        let cancel = step as f32 * 0.25;
        // Rebuild the state at the press for this jump shift.
        scratch.set_ball_state(*ball);
        seed_scratch_car(scratch, *state, now_tick);
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
        }
        let mut end = *scratch.get_car_state(0);
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
    // The first fresh packet after the activation has no chain lag (see `fit_dodge_start`): its tick is
    // the one within the lag range 0-4 ticks before its frame time at which the path with the fitted
    // jump, start and cancel reproduces it (position and velocity).
    let mut ticks_ab_eff = ticks_ab;
    let mut first_packet = None;
    if options.infer_dodge_first_packet_tick && first_fresh.0 >= activation_frame {
        scratch.set_ball_state(*ball);
        seed_scratch_car(scratch, *state, now_tick);
        let mut states: Vec<CarState> = vec![*state];
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
        let (frame_b, _, pos_b, vel_b, _) = first_fresh;
        let frame_tick = timeline(frame_b) - t_a;
        // Floored at 4 ticks like the dodge-start fit's window (see `fit_dodge_start`).
        let gap = (timeline(frame_b) - timeline(frame_b.saturating_sub(1))).max(4);
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
            first_packet = Some((frame_b, (frame_tick - tb).max(0) as u64));
        }
    }
    // The jump schedule for the interval to the next packet, up to the dodge press if it falls in it.
    let mut entries_out: Vec<(u64, f32, f32, bool, bool, Option<bool>)> = Vec::new();
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
    Some(FlipFit {
        schedule: GroundSchedule {
            slot,
            end_tick: now_tick + ticks_ab_eff as u64,
            entries: entries_out,
            shift: None,
        },
        first_packet,
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
            first_packet: None,
        }),
    })
}

/// Fits the flip's pitch-cancel amount from this fresh car packet. Candidate cancels (opposite pitch
/// input of 0, 0.25, ..., 1) are simulated in a scratch arena from the current corrected state
/// through the next `FLIP_CANCEL_PACKETS` fresh packets (the state reset to each, as the converter
/// does), and the one whose summed angular-velocity error is smallest wins; it is used for the
/// interval to the next packet. With `flip_cancel_holdout` that first interval is left out of the
/// sum. Fitting the next packet alone (the default, one packet) is in sample there, and once the
/// flip's speed saturates at 5.5 rad/s the candidates barely differ and the choice can alternate
/// between packets. Uses later packets, so it is offline reconstruction; spans containing a withheld
/// frame, an inactive frame, or a change of dodge counter are refused. `ball` is the main arena's ball
/// at this packet's tick: every candidate starts from it (the previous-interval sources park the ball
/// instead, since its state at the earlier packet is not at hand), so the result does not depend on
/// what an earlier fit left in the shared scratch arena.
#[allow(clippy::too_many_arguments)]
pub(super) fn fit_flip_cancel(
    observations: &ObservedReplay,
    options: &ConvertOptions,
    packet_lags: &Option<PacketLags>,
    first_time: f32,
    index: usize,
    car: &observations::Car,
    state: &CarState,
    base_controls: &CarControls,
    lag_a: u64,
    now_tick: u64,
    ball: &rocketsim::BallState,
    scratch: &mut Arena,
) -> Option<f32> {
    let frames = &observations.frames;
    let ang0 = car.body.angular_velocity_replay_units.as_ref()?;
    if ang0.frame != index {
        return None;
    }
    let counter = car.inputs.dodge_active_raw.as_ref()?.value;
    let timeline = |frame: usize| -> i64 {
        ((f64::from(frames[frame].time) - f64::from(first_time)) * 120.0).round() as i64
    };
    let active = |frame: usize| {
        frames[frame]
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active")
    };
    let withheld = |frame: usize| {
        options
            .withheld_frames
            .as_ref()
            .is_some_and(|w| w.get(frame).copied().unwrap_or(false))
    };
    if !active(index) {
        return None;
    }
    // The next fresh packets of the flip (up to `FLIP_CANCEL_PACKETS`, within 80 ticks, while the dodge
    // counter is unchanged): angular velocity to score and the full physical state to reset to, as
    // the converter does at each packet.
    let max_packets = FLIP_CANCEL_PACKETS;
    let mut targets: Vec<(i64, Vec3A, Vec3A, Mat3A, Vec3A)> = Vec::new();
    for candidate in index + 1..=(index + 24).min(frames.len() - 1) {
        let searching_more = !targets.is_empty();
        if !active(candidate) || withheld(candidate) {
            if searching_more {
                break;
            }
            return None;
        }
        let Some(other) = frames[candidate].cars.iter().find(|c| {
            c.actor_id == car.actor_id
                && c.actor_created_frame == car.actor_created_frame
                && c.player_key == car.player_key
        }) else {
            if searching_more {
                break;
            }
            return None;
        };
        let b = &other.body;
        let (Some(pos), Some(vel), Some(rot), Some(ang)) = (
            b.position.as_ref().filter(|x| x.frame == candidate),
            b.linear_velocity.as_ref().filter(|x| x.frame == candidate),
            b.rotation_xyzw.as_ref().filter(|x| x.frame == candidate),
            b.angular_velocity_replay_units
                .as_ref()
                .filter(|x| x.frame == candidate),
        ) else {
            continue;
        };
        if other.inputs.dodge_active_raw.as_ref().map(|d| d.value) != Some(counter) {
            if searching_more {
                break;
            }
            return None;
        }
        let lag_b = match packet_lags {
            Some(lags) => lags
                .car_actor
                .get(&(car.actor_id, car.actor_created_frame, candidate))
                .copied()
                .or(lags.cars[candidate])
                .map_or(
                    (timeline(candidate) - timeline(candidate - 1)).max(0) / 2,
                    |lag| lag.round().max(0.0) as i64,
                ),
            None => 0,
        };
        let ticks = (timeline(candidate) - lag_b) - (timeline(index) - lag_a as i64);
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
        if targets.len() == max_packets {
            break;
        }
    }
    if targets.is_empty() {
        return None;
    }
    let sign = state.flip_rel_torque.y.signum();
    // One cancel for all the intervals: each candidate is simulated interval by interval from the
    // packet, the state reset to each later packet as the converter does, and the angular-velocity
    // errors are summed. With `flip_cancel_holdout` the first interval (the one the cancel is used
    // for) is left out of the sum when there are later ones, so its residual stays a check.
    let first_scored = usize::from(options.flip_cancel_holdout && targets.len() > 1);
    let mut best: Option<(f32, f32)> = None;
    for step in 0..=4 {
        let cancel = step as f32 * 0.25;
        let mut start = *state;
        // The tick counter `start` belongs to: the main arena's at the first target, the scratch arena's
        // after the steps for the next ones (`last_extra_hit_tick` is an absolute tick).
        let mut start_tick = now_tick;
        let mut previous_ticks = 0;
        let mut total = 0.0f32;
        // Every candidate starts from the main arena's ball at this packet's tick (as the dodge start fit
        // does), not from wherever an earlier fit left the shared scratch arena's ball.
        scratch.set_ball_state(*ball);
        for (j, target) in targets.iter().enumerate() {
            seed_scratch_car(scratch, start, start_tick);
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

/// A dodge start plan: press `jump` with the dodge direction `start_offset` ticks after the current
/// packet, holding `cancel` of the flip's pitch torque cancelled until `duration` ticks after it.
pub(super) struct DodgePlan {
    pub(super) activation_frame: usize,
    pub(super) start_offset: u64,
    pub(super) duration: u64,
    pub(super) pitch: f32,
    pub(super) yaw: f32,
    pub(super) cancel: f32,
    /// The first fresh packet after the activation and the lag (ticks) inferred for it.
    pub(super) first_packet: Option<(usize, u64)>,
}

/// Fits when a dodge physically started. Given a fresh airborne car packet at `index` and a dodge
/// counter that turns odd (with a fresh `DodgeTorque`) before the next fresh car packet, every start
/// tick up to the *second* next fresh packet (both with exact chain lags) is simulated in a scratch
/// arena and the one whose position and velocity best match that packet wins; the pitch cancel is
/// then chosen from its angular velocity. The plan drives only the interval to the *next* packet,
/// so that packet is not used by the fit and its residual stays a held-out check (fitting the next
/// packet itself was in-sample and made later angular velocity worse). Uses later packets (offline
/// reconstruction); spans with a withheld or inactive frame are refused. The ball is simulated too
/// (its state at this packet's time in the main arena), so dodges at the ball are fitted with their
/// contacts; other cars are not modelled.
#[allow(clippy::too_many_arguments)]
pub(super) fn fit_dodge_start(
    observations: &ObservedReplay,
    options: &ConvertOptions,
    packet_lags: &Option<PacketLags>,
    first_time: f32,
    index: usize,
    car: &observations::Car,
    state: &CarState,
    base_controls: &CarControls,
    lag_a: u64,
    now_tick: u64,
    ball: &rocketsim::BallState,
    scratch: &mut Arena,
) -> Option<DodgePlan> {
    let frames = &observations.frames;
    let ang0 = car.body.angular_velocity_replay_units.as_ref()?;
    if ang0.frame != index || state.is_on_ground {
        return None;
    }
    // A car that has not dodged yet has no dodge counter (it is created with one at its first dodge).
    let counter = car.inputs.dodge_active_raw.as_ref().map_or(0, |d| d.value);
    if counter % 2 == 1 {
        return None;
    }
    let timeline = |frame: usize| -> i64 {
        ((f64::from(frames[frame].time) - f64::from(first_time)) * 120.0).round() as i64
    };
    let active = |frame: usize| {
        frames[frame]
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active")
    };
    let withheld = |frame: usize| {
        options
            .withheld_frames
            .as_ref()
            .is_some_and(|w| w.get(frame).copied().unwrap_or(false))
    };
    let same_car = |candidate: &&observations::Car| {
        candidate.actor_id == car.actor_id
            && candidate.actor_created_frame == car.actor_created_frame
            && candidate.player_key == car.player_key
    };
    let last = (index + 14).min(frames.len() - 1);
    // Activation: the first frame whose fresh dodge counter is odd with a fresh torque.
    let mut activation = None;
    for g in index + 1..=last {
        if !active(g) || withheld(g) {
            return None;
        }
        let other = frames[g].cars.iter().find(same_car)?;
        if let Some(dodge) = other
            .inputs
            .dodge_active_raw
            .as_ref()
            .filter(|d| d.frame == g)
            && dodge.value % 2 == 1
            && let Some(torque) = activation_torque(frames, g, other)
        {
            activation = Some((g, torque));
            break;
        }
    }
    let (activation_frame, torque) = activation?;
    // Plan only from the last fresh packet before the activation: a nearer packet would reset the
    // state under a plan that ignores it.
    for g in index + 1..activation_frame {
        let other = frames[g].cars.iter().find(same_car)?;
        if other.body.position.as_ref().is_some_and(|p| p.frame == g) {
            return None;
        }
    }
    // The next two fresh packets with exact chain lags at or after the activation frame. The first
    // is the next reset of the state and is held out; the fit uses the second.
    let lags = packet_lags.as_ref()?;
    let origin_tick = timeline(index) - lag_a as i64;
    let mut fresh: Vec<(u64, Vec3A, Vec3A, Vec3A)> = Vec::new();
    let mut fresh_frames: Vec<usize> = Vec::new();
    for g in activation_frame..=(activation_frame + 12).min(frames.len() - 1) {
        if !active(g) || withheld(g) {
            break;
        }
        let Some(other) = frames[g].cars.iter().find(same_car) else {
            break;
        };
        let b = &other.body;
        if let (Some(p), Some(v), Some(w)) = (
            b.position.as_ref().filter(|x| x.frame == g),
            b.linear_velocity.as_ref().filter(|x| x.frame == g),
            b.angular_velocity_replay_units
                .as_ref()
                .filter(|x| x.frame == g),
        ) {
            // The first fresh packet after the activation is the next reset of the state: it ends the
            // plan at the tick the converter injects it (its own lag, exact or not). The fit target
            // is the next one and needs an exact chain lag.
            let chain = lags
                .car_actor
                .get(&(car.actor_id, car.actor_created_frame, g))
                .copied();
            let lag = if fresh.is_empty() {
                chain
                    .or(lags.cars[g])
                    .map_or((timeline(g) - timeline(g - 1)).max(0) as f32 / 2.0, |lag| {
                        lag
                    })
            } else {
                let Some(lag) = chain else {
                    continue;
                };
                lag
            };
            let tick = (timeline(g) - lag.round().max(0.0) as i64) - origin_tick;
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
    // The path with no dodge, saved tick by tick, so each candidate resumes from its start tick.
    let mut path = vec![start];
    let mut path_ticks = vec![now_tick];
    let mut path_ball = vec![*ball];
    scratch.set_ball_state(*ball);
    seed_scratch_car(scratch, start, now_tick);
    scratch.set_car_controls(0, base);
    for _ in 0..horizon {
        scratch.step_tick();
        path.push(*scratch.get_car_state(0));
        path_ticks.push(scratch.tick_count());
        path_ball.push(*scratch.get_ball_state());
    }
    // States at every tick from `dodge_tick` to the horizon for a dodge at `dodge_tick` with `cancel`.
    let run_all = |scratch: &mut Arena, dodge_tick: u64, cancel: f32| -> Vec<CarState> {
        scratch.set_ball_state(path_ball[dodge_tick as usize - 1]);
        seed_scratch_car(
            scratch,
            path[dodge_tick as usize - 1],
            path_ticks[dodge_tick as usize - 1],
        );
        let mut after: Vec<CarState> = Vec::new();
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
    // States at every target tick for a dodge at `dodge_tick` with `cancel`.
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
    // A start after the next packet is not driven here (that packet resets the state); the normal
    // trigger at the activation frame applies instead.
    let mut deferred = dodge_tick > tick_b;

    let mut best_cancel: Option<(f32, f32)> = None;
    for step in 0..=4 {
        let cancel = step as f32 * 0.25;
        let states = run(scratch, dodge_tick, cancel);
        // Angular velocity only counts at packets at or after the start.
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
    // The first fresh packet after the activation has no chain lag (a dodge breaks the motion the chain
    // inference relies on), so its tick is only known to lie within the frame gap (0-4 ticks at 30 fps)
    // before its frame time. With the start and cancel fitted on the exact second packet, the tick in that
    // range at which the simulated path reproduces this packet (position and velocity) is its tick.
    let mut first_packet = None;
    let final_cancel = best_cancel?.0;
    if options.infer_dodge_first_packet_tick {
        let frame_tick = timeline(first_frame) - origin_tick;
        // A packet was generated within its frame window: the lag is at most the frame gap. The gap is
        // floored at 4 ticks (30 fps; a replay with 3-tick gaps occurs on 0.1% of frames in the corpus, and
        // none is faster), so for a faster replay (60 fps: 2 ticks) the search window is wider than the
        // gap allows and may pick a lag above it; the fitted path is exact there, so that is only a
        // risk with noisy packets. Untested on such a replay.
        let gap = (timeline(first_frame) - timeline(first_frame.saturating_sub(1))).max(4);
        let (lo, hi) = (
            (frame_tick - gap).max(1),
            frame_tick.min(second.0 as i64 - 1),
        );
        if lo <= hi {
            let after = run_all(scratch, dodge_tick, final_cancel);
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
                let lag = frame_tick - tb as i64;
                first_packet = Some((first_frame, lag.max(0) as u64, tb));
                deferred = dodge_tick > tb;
            }
        }
    }
    Some(DodgePlan {
        activation_frame,
        start_offset: dodge_tick,
        duration: match first_packet {
            Some((_, _, tb)) if !deferred => tb,
            _ if deferred => second.0,
            _ => tick_b,
        },
        pitch,
        yaw,
        cancel: final_cancel,
        first_packet: first_packet.map(|(frame, lag, _)| (frame, lag)),
    })
}
