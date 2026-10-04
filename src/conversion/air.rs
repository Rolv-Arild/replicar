//! Air controls: the inverse of RocketSim's air torque model, the span and boundary-value solves of
//! pitch, yaw and roll between fresh packets, and the causal persistence of a past control.

use super::*;

pub(super) const PI: f32 = std::f32::consts::PI;
pub(super) const TORQUE_APPLY_SCALE: f32 = 2.0 * PI / 65536.0 * 1000.0;
pub(super) const TORQUE_PITCH: f32 = 130.0 * TORQUE_APPLY_SCALE;
pub(super) const TORQUE_YAW: f32 = 95.0 * TORQUE_APPLY_SCALE;
pub(super) const TORQUE_ROLL: f32 = 400.0 * TORQUE_APPLY_SCALE;
pub(super) const DAMPING_PITCH: f32 = 30.0 * TORQUE_APPLY_SCALE;
pub(super) const DAMPING_YAW: f32 = 20.0 * TORQUE_APPLY_SCALE;
pub(super) const DAMPING_ROLL: f32 = 50.0 * TORQUE_APPLY_SCALE;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AirControls {
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
}

/// Analytically inverts RocketSim's air torque equations from consecutive angular velocities.
pub fn solve_inverse_air_controls(
    rot_mat_start: Mat3A,
    ang_vel_start: Vec3A,
    ang_vel_end: Vec3A,
    dt: f32,
) -> AirControls {
    if dt <= 0.0 || !dt.is_finite() {
        return AirControls::default();
    }

    let forward = rot_mat_start.x_axis;
    let right = rot_mat_start.y_axis;
    let up = rot_mat_start.z_axis;

    let dir_pitch = -right;
    let dir_yaw = up;
    let dir_roll = -forward;

    let tau_world = (ang_vel_end - ang_vel_start) / dt;

    let tau_p = dir_pitch.dot(tau_world);
    let tau_y = dir_yaw.dot(tau_world);
    let tau_r = dir_roll.dot(tau_world);

    let omega_p = dir_pitch.dot(ang_vel_start);
    let omega_y = dir_yaw.dot(ang_vel_start);
    let omega_r = dir_roll.dot(ang_vel_start);

    // Solve pitch: tau_p = u_p * T_p - omega_p * D_p * (1 - |u_p|)
    let rhs_p = tau_p + omega_p * DAMPING_PITCH;
    let denom_p = TORQUE_PITCH + rhs_p.signum() * omega_p * DAMPING_PITCH;
    let pitch = if denom_p.abs() > 1e-4 {
        (rhs_p / denom_p).clamp(-1.0, 1.0)
    } else {
        0.0
    };

    // Solve yaw: tau_y = u_y * T_y - omega_y * D_y * (1 - |u_y|)
    let rhs_y = tau_y + omega_y * DAMPING_YAW;
    let denom_y = TORQUE_YAW + rhs_y.signum() * omega_y * DAMPING_YAW;
    let yaw = if denom_y.abs() > 1e-4 {
        (rhs_y / denom_y).clamp(-1.0, 1.0)
    } else {
        0.0
    };

    // Solve roll: tau_r = u_r * T_r - omega_r * D_r (no damping reduction in RocketSim)
    let rhs_r = tau_r + omega_r * DAMPING_ROLL;
    let roll = (rhs_r / TORQUE_ROLL).clamp(-1.0, 1.0);

    AirControls { pitch, yaw, roll }
}

pub(super) const AIR_MAX_ANGULAR_SPEED: f32 = 5.5;

/// Integrates RocketSim's air torque and damping for `ticks` 120 Hz ticks under constant controls.
/// Returns the final world angular velocity. Flips, contact, and boost/throttle effects are absent.
pub fn air_angular_velocity_forward(
    rot_mat_start: Mat3A,
    ang_vel_start: Vec3A,
    controls: AirControls,
    ticks: u32,
) -> Vec3A {
    const TICK: f32 = 1.0 / 120.0;
    let mut rot = rot_mat_start;
    let mut omega = ang_vel_start;
    for _ in 0..ticks {
        let dir_pitch = -rot.y_axis;
        let dir_yaw = rot.z_axis;
        let dir_roll = -rot.x_axis;
        let any = controls.pitch != 0.0 || controls.yaw != 0.0 || controls.roll != 0.0;
        let torque = if any {
            dir_pitch * (controls.pitch * TORQUE_PITCH)
                + dir_yaw * (controls.yaw * TORQUE_YAW)
                + dir_roll * (controls.roll * TORQUE_ROLL)
        } else {
            Vec3A::ZERO
        };
        let damping = dir_pitch
            * (dir_pitch.dot(omega) * DAMPING_PITCH * (1.0 - controls.pitch.abs()))
            + dir_yaw * (dir_yaw.dot(omega) * DAMPING_YAW * (1.0 - controls.yaw.abs()))
            + dir_roll * (dir_roll.dot(omega) * DAMPING_ROLL);
        omega += (torque - damping) * TICK;
        let speed = omega.length();
        if speed > AIR_MAX_ANGULAR_SPEED {
            omega *= AIR_MAX_ANGULAR_SPEED / speed;
        }
        let step = omega * TICK;
        if step.length_squared() > 0.0 {
            rot = Mat3A::from_quat(Quat::from_scaled_axis(step.into())) * rot;
        }
    }
    omega
}

/// Constant controls over `ticks` that carry `ang_vel_start` to `ang_vel_end`. Starts from the
/// analytic inverse and applies forward-model corrections (`iterations` = 0 gives the analytic
/// result over `ticks / 120` seconds).
pub fn solve_span_air_controls(
    rot_mat_start: Mat3A,
    ang_vel_start: Vec3A,
    ang_vel_end: Vec3A,
    ticks: u32,
    iterations: usize,
) -> AirControls {
    let dt = ticks as f32 / 120.0;
    let mut virtual_target = ang_vel_end;
    let mut controls = solve_inverse_air_controls(rot_mat_start, ang_vel_start, virtual_target, dt);
    for _ in 0..iterations {
        let reached = air_angular_velocity_forward(rot_mat_start, ang_vel_start, controls, ticks);
        virtual_target += ang_vel_end - reached;
        controls = solve_inverse_air_controls(rot_mat_start, ang_vel_start, virtual_target, dt);
    }
    controls
}

/// Integrates the free-flight rotation of a car (RocketSim's air torque and damping, see
/// `air_angular_velocity_forward`) through consecutive segments of constant controls, given as
/// (controls, ticks). Returns the final rotation and world angular velocity.
pub fn air_state_forward(
    rot_mat_start: Mat3A,
    ang_vel_start: Vec3A,
    segments: &[(AirControls, u32)],
) -> (Mat3A, Vec3A) {
    const TICK: f32 = 1.0 / 120.0;
    let mut rot = rot_mat_start;
    let mut omega = ang_vel_start;
    for &(controls, ticks) in segments {
        for _ in 0..ticks {
            let dir_pitch = -rot.y_axis;
            let dir_yaw = rot.z_axis;
            let dir_roll = -rot.x_axis;
            let any = controls.pitch != 0.0 || controls.yaw != 0.0 || controls.roll != 0.0;
            let torque = if any {
                dir_pitch * (controls.pitch * TORQUE_PITCH)
                    + dir_yaw * (controls.yaw * TORQUE_YAW)
                    + dir_roll * (controls.roll * TORQUE_ROLL)
            } else {
                Vec3A::ZERO
            };
            let damping = dir_pitch
                * (dir_pitch.dot(omega) * DAMPING_PITCH * (1.0 - controls.pitch.abs()))
                + dir_yaw * (dir_yaw.dot(omega) * DAMPING_YAW * (1.0 - controls.yaw.abs()))
                + dir_roll * (dir_roll.dot(omega) * DAMPING_ROLL);
            omega += (torque - damping) * TICK;
            let speed = omega.length();
            if speed > AIR_MAX_ANGULAR_SPEED {
                omega *= AIR_MAX_ANGULAR_SPEED / speed;
            }
            let step = omega * TICK;
            if step.length_squared() > 0.0 {
                rot = Mat3A::from_quat(Quat::from_scaled_axis(step.into())) * rot;
            }
        }
    }
    (rot, omega)
}

/// The rotation vector (radians) that takes `from` to `to`, in the world frame.
pub(super) fn rotation_vector(from: Mat3A, to: Mat3A) -> Vec3A {
    let delta = Quat::from_mat3a(&(to * from.transpose())).normalize();
    let (axis, angle) = delta.to_axis_angle();
    let angle = if angle > PI { angle - 2.0 * PI } else { angle };
    Vec3A::from(axis) * angle
}

/// Solves the boundary-value problem of free flight: per-segment air controls (segments of `ticks`
/// ticks each) that carry the car from its start rotation and angular velocity to its end rotation
/// and angular velocity, as a Levenberg-Marquardt fit of the forward model that stays as close as
/// possible to `prior` (one control per segment). Six end conditions against three unknowns per
/// segment: two or three segments of a span pin the end state down; with more segments the prior
/// chooses among the solutions. Returns the controls and the remaining end error (radians of
/// rotation, radians per second of angular velocity).
pub fn solve_air_bvp(
    rot_start: Mat3A,
    omega_start: Vec3A,
    rot_end: Mat3A,
    omega_end: Vec3A,
    ticks: &[u32],
    prior: &[AirControls],
) -> (Vec<AirControls>, f32, f32) {
    let mut analytic =
        |segments: &[(AirControls, u32)]| air_state_forward(rot_start, omega_start, segments);
    solve_bvp_with(&mut analytic, rot_end, omega_end, ticks, prior)
}

/// `solve_air_bvp` with the forward model as a function of the per-segment controls and their tick
/// counts (RocketSim itself for a flipping car).
pub fn solve_bvp_with(
    forward: &mut dyn FnMut(&[(AirControls, u32)]) -> (Mat3A, Vec3A),
    rot_end: Mat3A,
    omega_end: Vec3A,
    ticks: &[u32],
    prior: &[AirControls],
) -> (Vec<AirControls>, f32, f32) {
    let n = ticks.len();
    let dim = 3 * n;
    let to_vec = |c: &[AirControls]| -> Vec<f32> {
        c.iter().flat_map(|c| [c.pitch, c.yaw, c.roll]).collect()
    };
    let from_vec = |v: &[f32]| -> Vec<AirControls> {
        v.chunks(3)
            .map(|c| AirControls {
                pitch: c[0],
                yaw: c[1],
                roll: c[2],
            })
            .collect()
    };
    // Residual scales: half a degree of rotation and 0.05 rad/s of angular velocity are one unit.
    const ROT_SCALE: f32 = 0.5 * PI / 180.0;
    const OMEGA_SCALE: f32 = 0.05;
    // Weight of staying at the prior, per unit of control.
    const PRIOR_WEIGHT: f32 = 0.05;
    let forward = std::cell::RefCell::new(forward);
    let residual = |u: &[f32]| -> [f32; 6] {
        let segments: Vec<(AirControls, u32)> =
            from_vec(u).into_iter().zip(ticks.iter().copied()).collect();
        let (rot, omega) = (forward.borrow_mut())(&segments);
        let dr = rotation_vector(rot, rot_end) / ROT_SCALE;
        let dw = (omega_end - omega) / OMEGA_SCALE;
        [dr.x, dr.y, dr.z, dw.x, dw.y, dw.z]
    };
    let u0 = to_vec(prior);
    let mut u = u0.clone();
    let mut lambda = 1.0f32;
    let cost_of = |u: &[f32], r: &[f32; 6]| -> f32 {
        let prior_cost: f32 = u.iter().zip(&u0).map(|(a, b)| (a - b) * (a - b)).sum();
        r.iter().map(|x| x * x).sum::<f32>() + PRIOR_WEIGHT * PRIOR_WEIGHT * prior_cost
    };
    let mut r = residual(&u);
    let mut current = cost_of(&u, &r);
    // The Jacobian at `u` is kept while a step is rejected: `u` does not change then, and neither
    // does the matrix (only `lambda` does), so recomputing it would repeat `dim` forward solves.
    let mut jac_cache: Option<Vec<[f32; 6]>> = None;
    for _ in 0..12 {
        let jac = &*jac_cache.get_or_insert_with(|| {
            // Finite-difference Jacobian (6 x dim).
            let mut jac = vec![[0.0f32; 6]; dim];
            for k in 0..dim {
                let mut up = u.clone();
                up[k] += 0.02;
                let rp = residual(&up);
                for i in 0..6 {
                    jac[k][i] = (rp[i] - r[i]) / 0.02;
                }
            }
            jac
        });
        // (J^T J + w^2 I + lambda I) delta = -(J^T r + w^2 (u - u0)).
        let mut a = vec![vec![0.0f32; dim]; dim];
        let mut g = vec![0.0f32; dim];
        for p in 0..dim {
            for q in 0..dim {
                a[p][q] = (0..6).map(|i| jac[p][i] * jac[q][i]).sum();
            }
            a[p][p] += PRIOR_WEIGHT * PRIOR_WEIGHT + lambda;
            g[p] = -((0..6).map(|i| jac[p][i] * r[i]).sum::<f32>()
                + PRIOR_WEIGHT * PRIOR_WEIGHT * (u[p] - u0[p]));
        }
        // Gaussian elimination with partial pivoting.
        let mut delta = g.clone();
        let mut m = a.clone();
        let mut ok = true;
        for col in 0..dim {
            let pivot = (col..dim)
                .max_by(|&x, &y| m[x][col].abs().total_cmp(&m[y][col].abs()))
                .unwrap_or(col);
            if m[pivot][col].abs() < 1e-9 {
                ok = false;
                break;
            }
            m.swap(col, pivot);
            delta.swap(col, pivot);
            for row in col + 1..dim {
                let factor = m[row][col] / m[col][col];
                for k in col..dim {
                    m[row][k] -= factor * m[col][k];
                }
                delta[row] -= factor * delta[col];
            }
        }
        if !ok {
            break;
        }
        for col in (0..dim).rev() {
            let tail: f32 = (col + 1..dim).map(|k| m[col][k] * delta[k]).sum();
            delta[col] = (delta[col] - tail) / m[col][col];
        }
        let candidate: Vec<f32> = u
            .iter()
            .zip(&delta)
            .map(|(a, d)| (a + d).clamp(-1.0, 1.0))
            .collect();
        let candidate_r = residual(&candidate);
        let candidate_cost = cost_of(&candidate, &candidate_r);
        if candidate_cost < current {
            let improvement = current - candidate_cost;
            u = candidate;
            r = candidate_r;
            jac_cache = None;
            current = candidate_cost;
            lambda = (lambda * 0.3).max(1e-4);
            if improvement < 1e-4 {
                break;
            }
        } else {
            lambda *= 4.0;
            if lambda > 1e4 {
                break;
            }
        }
    }
    let rot_error = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt() * ROT_SCALE;
    let omega_error = (r[3] * r[3] + r[4] * r[4] + r[5] * r[5]).sqrt() * OMEGA_SCALE;
    (from_vec(&u), rot_error, omega_error)
}

pub(super) fn vec3(value: [f32; 3]) -> Vec3A {
    Vec3A::new(value[0], value[1], value[2])
}

/// Projects displacement onto mean velocity and rounds the implied interval to 120 Hz.
/// Uses both endpoint positions and velocities, so it cannot predict the second position.
pub fn estimate_car_packet_interval(
    pos_start: [f32; 3],
    pos_end: [f32; 3],
    vel_start: [f32; 3],
    vel_end: [f32; 3],
    nominal_dt: f32,
) -> Option<OfflineIntervalEstimate> {
    if !nominal_dt.is_finite() || nominal_dt <= 0.0 || nominal_dt > 0.5 {
        return None;
    }
    let p0 = vec3(pos_start);
    let p1 = vec3(pos_end);
    let v0 = vec3(vel_start);
    let v1 = vec3(vel_end);

    let v_mean = (v0 + v1) * 0.5;
    let v_sq = v_mean.length_squared();
    if !v_sq.is_finite() || v_sq < 100.0 * 100.0 {
        return None;
    }

    let delta_p = p1 - p0;
    let dt_cont = delta_p.dot(v_mean) / v_sq;
    if !dt_cont.is_finite() || dt_cont <= 0.0 || dt_cont > 0.5 {
        return None;
    }

    let k = (dt_cont * 120.0).round().max(1.0) as u32;
    let effective_seconds = k as f32 / 120.0;
    let scale = effective_seconds / nominal_dt;

    Some(OfflineIntervalEstimate {
        effective_seconds,
        effective_ticks: k,
        scale,
    })
}

/// Offline aerial controls for the interval starting at `index`. The interval lies inside a span
/// between two fresh angular packets of the same car actor lifetime; one constant control solved
/// over the whole span is applied to every interval inside it. Adjacent-frame spans reproduce the
/// original one-frame lookahead. Uses a packet from after `index`, so it is offline reconstruction.
pub(super) fn span_lookahead_air_controls(
    observations: &ObservedReplay,
    index: usize,
    car: &observations::Car,
    min_z: f32,
    options: &ConvertOptions,
) -> Option<AirControls> {
    let ang0 = car.body.angular_velocity_replay_units.as_ref()?;
    let rot0 = car.body.rotation_xyzw.as_ref()?;
    let start = ang0.frame;
    if start > index
        || rot0.frame != start
        || index - start >= AIR_LOOKAHEAD_MAX_FRAMES
        || !car
            .body
            .position
            .as_ref()
            .is_some_and(|p| p.frame == start && p.value[2] > min_z)
    {
        return None;
    }
    let same_car = |candidate: &&observations::Car| {
        candidate.actor_id == car.actor_id
            && candidate.actor_created_frame == car.actor_created_frame
            && candidate.player_key == car.player_key
            && candidate.player_link_active == car.player_link_active
    };
    let active = |frame: &observations::Frame| {
        frame
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active")
    };
    let start_frame = observations.frames.get(start)?;
    if !active(start_frame)
        || !observations
            .frames
            .get(start)?
            .cars
            .iter()
            .any(|c| same_car(&c))
    {
        return None;
    }
    for earlier in start + 1..=index {
        let candidate = observations
            .frames
            .get(earlier)?
            .cars
            .iter()
            .find(same_car)?;
        if candidate
            .inputs
            .dodge_active_raw
            .as_ref()
            .is_some_and(|d| d.frame == earlier && d.value % 2 == 1)
        {
            return None;
        }
    }
    let last = (start + AIR_LOOKAHEAD_MAX_FRAMES).min(observations.frames.len() - 1);
    let mut end = None;
    for candidate_index in index + 1..=last {
        let candidate_frame = &observations.frames[candidate_index];
        if !active(candidate_frame) {
            return None;
        }
        let candidate = candidate_frame.cars.iter().find(same_car)?;
        if candidate
            .inputs
            .dodge_active_raw
            .as_ref()
            .is_some_and(|d| d.frame == candidate_index && d.value % 2 == 1)
        {
            return None;
        }
        if candidate
            .body
            .angular_velocity_replay_units
            .as_ref()
            .is_some_and(|a| a.frame == candidate_index)
        {
            end = Some((candidate_index, candidate));
            break;
        }
    }
    let (end_index, end_car) = end?;
    if let Some(withheld) = options.withheld_frames.as_ref()
        && (start + 1..end_index).any(|i| withheld.get(i).copied().unwrap_or(false))
    {
        return None;
    }
    let ang1 = end_car.body.angular_velocity_replay_units.as_ref()?;
    if !end_car
        .body
        .position
        .as_ref()
        .is_some_and(|p| p.frame == end_index && p.value[2] > min_z)
    {
        return None;
    }
    let dt = observations.frames[end_index].time - start_frame.time;
    if dt.is_nan() || dt <= 0.0 {
        return None;
    }
    let q0 = quaternion(rot0.value)?;
    Some(solve_span_air_controls(
        Mat3A::from_quat(q0),
        vec3(ang0.value) * 0.01,
        vec3(ang1.value) * 0.01,
        (dt * 120.0).round().max(1.0) as u32,
        AIR_LOOKAHEAD_REFINE_ITERATIONS,
    ))
}

/// Conditional-median persistence of a fitted aerial control, measured on the 60 train replays by
/// `calibrate_air_control_persistence` (416,600 fitted spans): the median later fitted control,
/// aligned with the earlier control's sign and expressed per unit of the earlier magnitude.
/// Indexed `[axis][lag band][|u| bin]` with axes pitch, yaw, roll; lag bands 0.033-0.083 s (also
/// anything shorter), 0.083-0.133 s and 0.133-0.200 s between span midpoints; |u| bins
/// [0.1,0.3), [0.3,0.5), [0.5,0.7), [0.7,0.9), [0.9,1]. Errors are judged by quantiles of absolute
/// error, for which the optimal point prediction of an uncertain input is its conditional median.
/// Roll ratios near 1 for |u| >= 0.5 reflect fits at the 5.5 rad/s angular speed cap: the fitted
/// roll there is the minimum that balances RocketSim's roll damping (about 0.69) and it persists.
pub(super) const AIR_CONTROL_MEDIAN_RATIO: [[[f32; 5]; 3]; 3] = [
    [
        [0.210, 0.547, 0.758, 0.535, 0.464],
        [0.119, 0.234, 0.524, 0.397, 0.268],
        [0.014, 0.020, 0.130, 0.043, 0.019],
    ],
    [
        [0.407, 0.676, 0.627, 0.562, 0.508],
        [0.282, 0.412, 0.407, 0.438, 0.325],
        [0.121, 0.109, 0.092, 0.093, 0.051],
    ],
    [
        [0.087, 0.547, 1.031, 0.896, 0.705],
        [0.162, 0.425, 1.022, 0.892, 0.699],
        [0.125, 0.272, 1.004, 0.852, 0.642],
    ],
];

/// Median persistence ratio for a fitted control on `axis` (0 pitch, 1 yaw, 2 roll) with the
/// given `magnitude`, `lag` seconds after its span midpoint. Below the calibrated magnitude range
/// (fit noise) or beyond 0.2 s there is no evidence, so nothing persists.
pub(super) fn air_control_median_ratio(axis: usize, lag: f32, magnitude: f32) -> f32 {
    if !lag.is_finite() || !(0.0..0.2).contains(&lag) || !(0.1..=1.0 + 1e-3).contains(&magnitude) {
        return 0.0;
    }
    let band = if lag < 2.5 / 30.0 {
        0
    } else if lag < 4.0 / 30.0 {
        1
    } else {
        2
    };
    let bin = [0.3, 0.5, 0.7, 0.9]
        .iter()
        .position(|&edge| magnitude < edge)
        .unwrap_or(4);
    AIR_CONTROL_MEDIAN_RATIO[axis][band][bin]
}

/// Causal aerial controls for the interval starting at `index`: the constant control that carried
/// the car from its second-latest to its latest fresh angular packet, both at or before `index`.
/// Nothing after `index` is read, so it is valid for prediction across withheld packets.
pub(super) fn past_persisted_air_controls(
    observations: &ObservedReplay,
    index: usize,
    car: &observations::Car,
    min_z: f32,
) -> Option<(AirControls, f32)> {
    let ang1 = car.body.angular_velocity_replay_units.as_ref()?;
    let end = ang1.frame;
    if end == 0
        || end > index
        || !car
            .body
            .position
            .as_ref()
            .is_some_and(|p| p.frame == end && p.value[2] > min_z)
    {
        return None;
    }
    let end_frame = observations.frames.get(end)?;
    observations.frames.get(index)?;
    let same_car = |candidate: &&observations::Car| {
        candidate.actor_id == car.actor_id
            && candidate.actor_created_frame == car.actor_created_frame
            && candidate.player_key == car.player_key
            && candidate.player_link_active == car.player_link_active
    };
    let active = |frame: &observations::Frame| {
        frame
            .game_state
            .as_ref()
            .is_some_and(|state| state.value == "Active")
    };
    let before = observations
        .frames
        .get(end - 1)?
        .cars
        .iter()
        .find(same_car)?;
    let ang0 = before.body.angular_velocity_replay_units.as_ref()?;
    let rot0 = before.body.rotation_xyzw.as_ref()?;
    let start = ang0.frame;
    if start >= end
        || rot0.frame != start
        || !before
            .body
            .position
            .as_ref()
            .is_some_and(|p| p.frame == start && p.value[2] > min_z)
    {
        return None;
    }
    let dt = end_frame.time - observations.frames.get(start)?.time;
    if dt.is_nan() || dt <= 0.0 {
        return None;
    }
    for frame_index in start..=end {
        let frame = &observations.frames[frame_index];
        let candidate = frame.cars.iter().find(same_car)?;
        if !active(frame)
            || (frame_index > start
                && candidate
                    .inputs
                    .dodge_active_raw
                    .as_ref()
                    .is_some_and(|d| d.frame == frame_index && d.value % 2 == 1))
        {
            return None;
        }
    }
    let q0 = quaternion(rot0.value)?;
    let solved = solve_span_air_controls(
        Mat3A::from_quat(q0),
        vec3(ang0.value) * 0.01,
        vec3(ang1.value) * 0.01,
        (dt * 120.0).round().max(1.0) as u32,
        AIR_LOOKAHEAD_REFINE_ITERATIONS,
    );
    let span_mid = 0.5 * (observations.frames.get(start)?.time + end_frame.time);
    let interval_end = observations
        .frames
        .get(index + 1)
        .map_or(observations.frames.get(index)?.time, |frame| frame.time);
    let interval_mid = 0.5 * (observations.frames.get(index)?.time + interval_end);
    let lag = interval_mid - span_mid;
    Some((solved, lag))
}

/// Per-tick air controls for one car over the interval to its next fresh packet (`plan_air_bvp`):
/// (first arena tick, controls), in order.
pub(super) struct AirSchedule {
    pub(super) slot: usize,
    pub(super) end_tick: u64,
    pub(super) entries: Vec<(u64, AirControls)>,
}

/// Plans the air controls of an airborne car for the interval from its fresh packet at `index` to its
/// next fresh packet as a boundary-value problem (`solve_air_bvp`): controls that may change every
/// few ticks carry the car to the next packet's rotation and angular velocity (not just the angular
/// velocity with one constant control, as the span solve does), starting from the constant span
/// solution. The packets' ticks are their frame times minus their lags. Refused for a car that is
/// not in free flight at both packets, a dodge in the span, a withheld or inactive frame, or a
/// solution that does not reach the end state. Uses the next packet: offline reconstruction.
#[allow(clippy::too_many_arguments)]
pub(super) fn plan_air_bvp(
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
    scratch: Option<&mut Arena>,
    pending: &[PendingDodge],
    // Lags a fit has set for single packets (as `car_lag` reads them): the end packet's tick is the one
    // the main loop injects it at.
    lag_overrides: &HashMap<(i32, usize, usize), u64>,
) -> Option<(AirSchedule, i32)> {
    static TUNING: OnceLock<(f32, f32, f32)> = OnceLock::new();
    let (min_z_value, rot_tol_deg, omega_tol) = *TUNING.get_or_init(|| {
        let get = |k: &str, d: f32| {
            std::env::var(k)
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(d)
        };
        (
            get("AIR_BVP_MIN_Z", 30.0),
            get("AIR_BVP_ROT_TOL", 3.0),
            get("AIR_BVP_OMEGA_TOL", 0.5),
        )
    });
    let press = pending
        .iter()
        .find(|d| d.slot == slot && d.start_tick > now_tick)
        .copied();
    if state.is_on_ground || ((state.is_flipping || press.is_some()) && scratch.is_none()) {
        return None;
    }
    let frames = &observations.frames;
    let lags = packet_lags.as_ref()?;
    let timeline = |frame: usize| -> i64 {
        ((f64::from(frames[frame].time) - f64::from(first_time)) * 120.0).round() as i64
    };
    let fresh_rotation = |c: &observations::Car, frame: usize| -> Option<(Mat3A, Vec3A, f32)> {
        let p = c.body.position.as_ref().filter(|v| v.frame == frame)?;
        let r = c.body.rotation_xyzw.as_ref().filter(|v| v.frame == frame)?;
        let w = c
            .body
            .angular_velocity_replay_units
            .as_ref()
            .filter(|v| v.frame == frame)?;
        Some((
            Mat3A::from_quat(quaternion(r.value)?),
            vec3(w.value) * 0.01,
            p.value[2],
        ))
    };
    let (rot_a, omega_a, z_a) = fresh_rotation(car, index)?;
    if z_a < min_z_value {
        return None;
    }
    let active = |frame: usize| {
        frames[frame]
            .game_state
            .as_ref()
            .is_some_and(|g| g.value == "Active")
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
    let same_car = |c: &&observations::Car| {
        c.actor_id == car.actor_id
            && c.actor_created_frame == car.actor_created_frame
            && c.player_key == car.player_key
    };
    let t_a = timeline(index) - lag_a as i64;
    let mut end = None;
    for g in index + 1..=(index + 24).min(frames.len() - 1) {
        if !active(g) || withheld(g) {
            return None;
        }
        let other = frames[g].cars.iter().find(same_car)?;
        let dodge_in_span = other
            .inputs
            .dodge_active_raw
            .as_ref()
            .is_some_and(|d| d.frame == g && d.value % 2 == 1);
        if dodge_in_span
            && !pending
                .iter()
                .any(|d| d.slot == slot && d.start_tick > now_tick)
        {
            return None;
        }
        if let Some((rot_b, omega_b, z_b)) = fresh_rotation(other, g) {
            // The lag as the main loop applies it (`car_lag`): at most the frame gap.
            let gap = (timeline(g) - timeline(g - 1)).max(0);
            let lag = match lag_overrides.get(&(car.actor_id, car.actor_created_frame, g)) {
                Some(&fitted) => fitted as i64,
                None => lags
                    .car_actor
                    .get(&(car.actor_id, car.actor_created_frame, g))
                    .copied()
                    .or(lags.cars[g])
                    .map_or(gap / 2, |lag| lag.round().max(0.0) as i64),
            }
            .min(gap);
            end = Some((g, rot_b, omega_b, z_b, timeline(g) - lag));
            break;
        }
    }
    let (_, rot_b, omega_b, z_b, t_b) = end?;
    let total = t_b - t_a;
    if z_b < min_z_value || !(2..=90).contains(&total) {
        return None;
    }
    let total = total as u32;
    // Segments of about four ticks (a frame at 30 fps).
    let parts = total.div_ceil(4).max(1);
    let ticks: Vec<u32> = (0..parts)
        .map(|i| total * (i + 1) / parts - total * i / parts)
        .collect();
    let constant = solve_span_air_controls(
        rot_a,
        omega_a,
        omega_b,
        total,
        AIR_LOOKAHEAD_REFINE_ITERATIONS,
    );
    let prior = vec![constant; ticks.len()];
    let mut shift = 0i32;
    let (solved, rot_error, omega_error) = match scratch {
        Some(scratch) if state.is_flipping || press.is_some() => {
            // A flip is not free flight: its torque, pitch lock and cancel are RocketSim's, so the
            // forward model is RocketSim itself (a single car in a scratch arena, from the packet).
            // The tick of the flip's start is only known to a few ticks (a fitted dodge press, or the
            // flip time a simulated flip has reached), so a shift of it is tried too: the first
            // that reaches the end state, else the one that gets closest.
            let mut best: Option<(Vec<AirControls>, f32, f32, i32)> = None;
            // Offsets beyond three ticks are rare and cost 20% of a conversion: limiting them from six to
            // three left the interior flip error unchanged (rotation p90 4.70 to 4.73 deg).
            let max_offset = std::env::var("AIR_BVP_MAX_OFFSET")
                .ok()
                .and_then(|v| v.parse::<i32>().ok())
                .unwrap_or(3);
            for offset in [0i32, -1, 1, -2, 2, -3, 3, -4, 4, -5, 5, -6, 6]
                .into_iter()
                .filter(|o| o.abs() <= max_offset)
            {
                let mut start = *state;
                start.phys.rot_mat = rot_a;
                start.phys.ang_vel = omega_a;
                let mut press_tick = press.map(|d| d.start_tick);
                if let Some(tick) = press_tick.as_mut() {
                    let shifted = *tick as i64 + i64::from(offset);
                    if shifted <= now_tick as i64 || shifted > now_tick as i64 + i64::from(total) {
                        continue;
                    }
                    *tick = shifted as u64;
                } else {
                    let shifted = start.flip_time + offset as f32 / 120.0;
                    if shifted < 0.0 {
                        continue;
                    }
                    start.flip_time = shifted;
                }
                let mut forward = |segments: &[(AirControls, u32)]| -> (Mat3A, Vec3A) {
                    // The ball is parked far from the car: a contact in the scratch would not be the
                    // real one.
                    let mut parked = rocketsim::BallState::default();
                    parked.phys.pos = Vec3A::new(0.0, 0.0, 1800.0);
                    if (start.phys.pos - parked.phys.pos).length() < 600.0 {
                        parked.phys.pos = Vec3A::new(3000.0, 4000.0, 300.0);
                    }
                    scratch.set_ball_state(parked);
                    seed_scratch_car(scratch, start, now_tick);
                    let mut tick = now_tick;
                    for &(controls, n) in segments {
                        for _ in 0..n {
                            tick += 1;
                            let mut c = CarControls {
                                pitch: controls.pitch,
                                yaw: controls.yaw,
                                roll: controls.roll,
                                ..CarControls::default()
                            };
                            if let (Some(d), Some(at)) = (press, press_tick)
                                && tick == at
                            {
                                c.jump = true;
                                c.pitch = d.pitch;
                                c.yaw = d.yaw;
                                c.roll = 0.0;
                            }
                            scratch.set_car_controls(0, c);
                            scratch.step_tick();
                        }
                    }
                    let end = scratch.get_car_state(0);
                    let mut omega = end.phys.ang_vel;
                    let speed = omega.length();
                    if speed > AIR_MAX_ANGULAR_SPEED {
                        omega *= AIR_MAX_ANGULAR_SPEED / speed;
                    }
                    (end.phys.rot_mat, omega)
                };
                let (solved, rot_error, omega_error) =
                    solve_bvp_with(&mut forward, rot_b, omega_b, &ticks, &prior);
                let within = rot_error <= rot_tol_deg.to_radians() && omega_error <= omega_tol;
                if let Ok(range) = std::env::var("AIR_BVP_DEBUG") {
                    let mut parts = range.split('-').filter_map(|v| v.parse::<usize>().ok());
                    let (lo, hi) = (parts.next().unwrap_or(0), parts.next().unwrap_or(0));
                    if (lo..=hi).contains(&index) {
                        eprintln!(
                            "BVPDEBUG frame {index} slot {slot} ticks {total} flipping {} flip_time {:.3} press {:?} offset {offset}: rot error {:.2} deg, omega error {:.3} rad/s; omega a {:.2} b {:.2}",
                            state.is_flipping,
                            state.flip_time,
                            press_tick.map(|t| t as i64 - now_tick as i64),
                            rot_error.to_degrees(),
                            omega_error,
                            omega_a.length(),
                            omega_b.length()
                        );
                    }
                }
                let better = best.as_ref().is_none_or(|b| {
                    rot_error / rot_tol_deg.to_radians() + omega_error / omega_tol
                        < b.1 / rot_tol_deg.to_radians() + b.2 / omega_tol
                });
                if better {
                    best = Some((solved, rot_error, omega_error, offset));
                }
                if within {
                    break;
                }
            }
            let (solved, rot_error, omega_error, offset) = best?;
            shift = offset;
            (solved, rot_error, omega_error)
        }
        _ => solve_air_bvp(rot_a, omega_a, rot_b, omega_b, &ticks, &prior),
    };
    // A solution that cannot reach the end state means the free-flight model does not hold (a
    // contact, a wall, an unseen flip): leave the interval to the other control paths.
    // (Written so that a NaN solve is refused too.)
    if !(rot_error <= rot_tol_deg.to_radians() && omega_error <= omega_tol) {
        if std::env::var_os("AIR_NOSOL").is_some() {
            eprintln!(
                "NOSOL frame {index} pos {:.2} {:.2} ticks {total} flip_time {:.3} flipping {} press {} z {:.0} speed {:.0} rot_err {:.1} deg omega_err {:.2} omega_a {:.2} omega_b {:.2}",
                state.phys.pos.x,
                state.phys.pos.y,
                state.flip_time,
                state.is_flipping,
                press.is_some(),
                state.phys.pos.z,
                state.phys.vel.length(),
                rot_error.to_degrees(),
                omega_error,
                omega_a.length(),
                omega_b.length()
            );
        }
        return None;
    }
    let mut entries = Vec::with_capacity(ticks.len());
    let mut tick = now_tick + 1;
    for (controls, n) in solved.iter().zip(&ticks) {
        entries.push((tick, *controls));
        tick += u64::from(*n);
    }
    Some((
        AirSchedule {
            slot,
            end_tick: now_tick + u64::from(total),
            entries,
        },
        shift,
    ))
}
