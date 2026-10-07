//! Air controls: RocketSim's air torque model, its analytic inverse, the constant control over a span between
//! two updates, and the persistence of a past control (docs/glossary.md, "Air controls").

use glam::{Mat3A, Quat, Vec3A};
use replicar_format::FrameIndex;

use crate::decode::{GameState, NetworkCar, NetworkFrame};
use crate::update_ticks::quaternion;

const PI: f32 = std::f32::consts::PI;
const TORQUE_APPLY_SCALE: f32 = 2.0 * PI / 65536.0 * 1000.0;
const TORQUE_PITCH: f32 = 130.0 * TORQUE_APPLY_SCALE;
const TORQUE_YAW: f32 = 95.0 * TORQUE_APPLY_SCALE;
const TORQUE_ROLL: f32 = 400.0 * TORQUE_APPLY_SCALE;
const DAMPING_PITCH: f32 = 30.0 * TORQUE_APPLY_SCALE;
const DAMPING_YAW: f32 = 20.0 * TORQUE_APPLY_SCALE;
const DAMPING_ROLL: f32 = 50.0 * TORQUE_APPLY_SCALE;
const MAX_ANGULAR_SPEED: f32 = 5.5;
/// Gauss-Newton refinements of the span solve (one refines the analytic start).
const SPAN_REFINEMENTS: usize = 1;

/// Pitch, yaw and roll in RocketSim's ranges.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AirControls {
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
}

/// The constant controls that change `ang_vel_start` to `ang_vel_end` in `dt` seconds under RocketSim's air
/// torque and damping, solved analytically per axis.
#[must_use]
pub fn invert(rot_start: Mat3A, ang_vel_start: Vec3A, ang_vel_end: Vec3A, dt: f32) -> AirControls {
    if dt <= 0.0 || !dt.is_finite() {
        return AirControls::default();
    }
    let dir_pitch = -rot_start.y_axis;
    let dir_yaw = rot_start.z_axis;
    let dir_roll = -rot_start.x_axis;
    let torque = (ang_vel_end - ang_vel_start) / dt;
    let (tau_p, tau_y, tau_r) = (
        dir_pitch.dot(torque),
        dir_yaw.dot(torque),
        dir_roll.dot(torque),
    );
    let (omega_p, omega_y, omega_r) = (
        dir_pitch.dot(ang_vel_start),
        dir_yaw.dot(ang_vel_start),
        dir_roll.dot(ang_vel_start),
    );
    // Pitch and yaw: tau = u * T - omega * D * (1 - |u|).
    let damped = |tau: f32, omega: f32, torque: f32, damping: f32| {
        let rhs = tau + omega * damping;
        let denominator = torque + rhs.signum() * omega * damping;
        if denominator.abs() > 1e-4 {
            (rhs / denominator).clamp(-1.0, 1.0)
        } else {
            0.0
        }
    };
    AirControls {
        pitch: damped(tau_p, omega_p, TORQUE_PITCH, DAMPING_PITCH),
        yaw: damped(tau_y, omega_y, TORQUE_YAW, DAMPING_YAW),
        // Roll: tau = u * T - omega * D (RocketSim does not reduce the roll damping).
        roll: ((tau_r + omega_r * DAMPING_ROLL) / TORQUE_ROLL).clamp(-1.0, 1.0),
    }
}

/// RocketSim's free-flight rotation through consecutive segments of constant controls `(controls, ticks)`:
/// the final rotation and world angular velocity. Flips, contacts, boost and throttle are absent.
#[must_use]
pub fn fly(
    rot_start: Mat3A,
    ang_vel_start: Vec3A,
    segments: &[(AirControls, u32)],
) -> (Mat3A, Vec3A) {
    const TICK: f32 = 1.0 / 120.0;
    let (mut rot, mut omega) = (rot_start, ang_vel_start);
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
            if speed > MAX_ANGULAR_SPEED {
                omega *= MAX_ANGULAR_SPEED / speed;
            }
            let step = omega * TICK;
            if step.length_squared() > 0.0 {
                rot = Mat3A::from_quat(Quat::from_scaled_axis(step.into())) * rot;
            }
        }
    }
    (rot, omega)
}

/// Constant controls over `ticks` that carry `ang_vel_start` to `ang_vel_end`: the analytic inverse, refined
/// `iterations` times against the forward model.
#[must_use]
pub fn solve_span(
    rot_start: Mat3A,
    ang_vel_start: Vec3A,
    ang_vel_end: Vec3A,
    ticks: u32,
    iterations: usize,
) -> AirControls {
    let dt = ticks as f32 / 120.0;
    let mut target = ang_vel_end;
    let mut controls = invert(rot_start, ang_vel_start, target, dt);
    for _ in 0..iterations {
        let (_, reached) = fly(rot_start, ang_vel_start, &[(controls, ticks)]);
        target += ang_vel_end - reached;
        controls = invert(rot_start, ang_vel_start, target, dt);
    }
    controls
}

/// How much of a fitted air control persists, as the median later fitted control per unit of the earlier
/// one (aligned with its sign), measured on the 60 train replays (416,600 fitted spans). Indexed
/// `[axis][lag band][|u| bin]`: axes pitch, yaw, roll; lag bands 0.033-0.083 s (and shorter), 0.083-0.133 s
/// and 0.133-0.200 s between span midpoints; |u| bins [0.1, 0.3), [0.3, 0.5), [0.5, 0.7), [0.7, 0.9),
/// [0.9, 1]. Roll ratios near 1 for |u| >= 0.5 are fits at the 5.5 rad/s cap, where the fitted roll is the
/// one that balances RocketSim's roll damping and persists.
const PERSISTENCE: [[[f32; 5]; 3]; 3] = [
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

/// The median persistence of a control on `axis` (0 pitch, 1 yaw, 2 roll) of `magnitude`, `lag` seconds
/// after its span's midpoint. Below the calibrated magnitudes (fit noise) or beyond 0.2 s nothing persists.
#[must_use]
pub fn persistence(axis: usize, lag: f32, magnitude: f32) -> f32 {
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
    PERSISTENCE[axis][band][bin]
}

/// The constant control that carried a car from its second-latest to its latest update with an angular
/// velocity, both at or before frame `index`, and how long before the middle of the interval that starts at
/// `index` the middle of that span lies (seconds). Reads nothing after `index`'s own interval end time.
/// `None` when either update is below `min_z`, the span crosses a dodge or a frame out of play, or the car
/// changed in between.
#[must_use]
pub fn past_controls(
    frames: &[NetworkFrame],
    index: usize,
    car: &NetworkCar,
    min_z: f32,
) -> Option<(AirControls, f32)> {
    let ang1 = car.body.angular_velocity_raw.as_ref()?;
    let end = ang1.frame.get();
    if end == 0
        || end > index
        || !car
            .body
            .position
            .as_ref()
            .is_some_and(|p| p.frame.get() == end && p.value[2] > min_z)
    {
        return None;
    }
    let end_frame = frames.get(end)?;
    frames.get(index)?;
    let same_car = |candidate: &&NetworkCar| {
        candidate.life == car.life
            && candidate.player == car.player
            && candidate.player_link_active == car.player_link_active
    };
    let before = frames.get(end - 1)?.cars.iter().find(same_car)?;
    let ang0 = before.body.angular_velocity_raw.as_ref()?;
    let rot0 = before.body.rotation.as_ref()?;
    let start = ang0.frame.get();
    if start >= end
        || rot0.frame != FrameIndex(start as u32)
        || !before
            .body
            .position
            .as_ref()
            .is_some_and(|p| p.frame.get() == start && p.value[2] > min_z)
    {
        return None;
    }
    let dt = end_frame.time - frames.get(start)?.time;
    if dt.is_nan() || dt <= 0.0 {
        return None;
    }
    for (offset, frame) in frames[start..=end].iter().enumerate() {
        let candidate = frame.cars.iter().find(same_car)?;
        let in_play = frame
            .game_state
            .as_ref()
            .is_some_and(|s| s.value == GameState::Active);
        let dodged = offset > 0
            && candidate
                .inputs
                .dodge_active_raw
                .as_ref()
                .is_some_and(|d| d.frame == frame.index && d.value % 2 == 1);
        if !in_play || dodged {
            return None;
        }
    }
    let q0 = quaternion(rot0.value)?;
    let solved = solve_span(
        Mat3A::from_quat(q0),
        Vec3A::from(ang0.value) * 0.01,
        Vec3A::from(ang1.value) * 0.01,
        (dt * 120.0).round().max(1.0) as u32,
        SPAN_REFINEMENTS,
    );
    let span_mid = 0.5 * (frames.get(start)?.time + end_frame.time);
    let interval_end = frames
        .get(index + 1)
        .map_or(frames.get(index)?.time, |frame| frame.time);
    let interval_mid = 0.5 * (frames.get(index)?.time + interval_end);
    Some((solved, interval_mid - span_mid))
}

/// The longest span, in frames, the lookahead bridges between two updates.
const LOOKAHEAD_MAX_FRAMES: usize = 10_000;

/// The constant control over the span between the car's latest update with an angular velocity (at or before
/// `index`) and its next one: offline, it uses an update after `index`. `None` when either update is below
/// `min_z`, the span crosses a dodge, a frame out of play or a withheld frame, or the car changed.
#[must_use]
pub fn lookahead_controls(
    frames: &[NetworkFrame],
    index: usize,
    car: &NetworkCar,
    min_z: f32,
    withheld: crate::update_ticks::Withheld,
) -> Option<AirControls> {
    let ang0 = car.body.angular_velocity_raw.as_ref()?;
    let rot0 = car.body.rotation.as_ref()?;
    let start = ang0.frame.get();
    if start > index
        || rot0.frame.get() != start
        || index - start >= LOOKAHEAD_MAX_FRAMES
        || !car
            .body
            .position
            .as_ref()
            .is_some_and(|p| p.frame.get() == start && p.value[2] > min_z)
    {
        return None;
    }
    let same_car = |candidate: &&NetworkCar| {
        candidate.life == car.life
            && candidate.player == car.player
            && candidate.player_link_active == car.player_link_active
    };
    let in_play = |frame: &NetworkFrame| {
        frame
            .game_state
            .as_ref()
            .is_some_and(|s| s.value == GameState::Active)
    };
    let dodged_at = |candidate: &NetworkCar, f: usize| {
        candidate
            .inputs
            .dodge_active_raw
            .as_ref()
            .is_some_and(|d| d.frame.get() == f && d.value % 2 == 1)
    };
    let start_frame = frames.get(start)?;
    if !in_play(start_frame) || !start_frame.cars.iter().any(|c| same_car(&c)) {
        return None;
    }
    for earlier in start + 1..=index {
        let candidate = frames.get(earlier)?.cars.iter().find(same_car)?;
        if dodged_at(candidate, earlier) {
            return None;
        }
    }
    let last = (start + LOOKAHEAD_MAX_FRAMES).min(frames.len() - 1);
    let mut end = None;
    for f in index + 1..=last {
        let frame = &frames[f];
        if !in_play(frame) {
            return None;
        }
        let candidate = frame.cars.iter().find(same_car)?;
        if dodged_at(candidate, f) {
            return None;
        }
        if candidate
            .body
            .angular_velocity_raw
            .as_ref()
            .is_some_and(|a| a.frame.get() == f)
        {
            end = Some((f, candidate));
            break;
        }
    }
    let (end_index, end_car) = end?;
    if (start + 1..end_index).any(|f| withheld.contains(f)) {
        return None;
    }
    let ang1 = end_car.body.angular_velocity_raw.as_ref()?;
    if !end_car
        .body
        .position
        .as_ref()
        .is_some_and(|p| p.frame.get() == end_index && p.value[2] > min_z)
    {
        return None;
    }
    let dt = frames[end_index].time - start_frame.time;
    if dt.is_nan() || dt <= 0.0 {
        return None;
    }
    let q0 = quaternion(rot0.value)?;
    Some(solve_span(
        Mat3A::from_quat(q0),
        Vec3A::from(ang0.value) * 0.01,
        Vec3A::from(ang1.value) * 0.01,
        (dt * 120.0).round().max(1.0) as u32,
        SPAN_REFINEMENTS,
    ))
}

/// A segment of constant air controls: the controls and their number of ticks.
pub type Segment = (AirControls, u32);

/// A forward model of free flight: the end rotation and angular velocity after consecutive segments.
pub type ForwardModel<'f> = dyn FnMut(&[Segment]) -> (Mat3A, Vec3A) + 'f;

/// The finite-difference Jacobian of the residuals at a point (with the residuals there): one column per parameter.
type Jacobian<'f> = dyn FnMut(&[f32], &[f32; 6]) -> Vec<[f32; 6]> + 'f;

/// The rotation vector (radians, world frame) that takes `from` to `to`.
#[must_use]
pub fn rotation_vector(from: Mat3A, to: Mat3A) -> Vec3A {
    let delta = Quat::from_mat3a(&(to * from.transpose())).normalize();
    let (axis, angle) = delta.to_axis_angle();
    let angle = if angle > PI { angle - 2.0 * PI } else { angle };
    Vec3A::from(axis) * angle
}

// Residual units: half a degree of rotation, 0.05 rad/s of angular velocity.
const ROT_SCALE: f32 = 0.5 * PI / 180.0;
const OMEGA_SCALE: f32 = 0.05;
// The weight of staying at the prior, per unit of control.
const PRIOR_WEIGHT: f32 = 0.05;
// The step of the finite differences.
const DIFFERENCE: f32 = 0.02;

fn to_vec(c: &[AirControls]) -> Vec<f32> {
    c.iter().flat_map(|c| [c.pitch, c.yaw, c.roll]).collect()
}

fn from_vec(v: &[f32]) -> Vec<AirControls> {
    v.chunks(3)
        .map(|c| AirControls {
            pitch: c[0],
            yaw: c[1],
            roll: c[2],
        })
        .collect()
}

fn segments_of(u: &[f32], ticks: &[u32]) -> Vec<Segment> {
    from_vec(u).into_iter().zip(ticks.iter().copied()).collect()
}

/// The six end conditions' residuals, in their units.
fn residual_of(rot: Mat3A, omega: Vec3A, rot_end: Mat3A, omega_end: Vec3A) -> [f32; 6] {
    let dr = rotation_vector(rot, rot_end) / ROT_SCALE;
    let dw = (omega_end - omega) / OMEGA_SCALE;
    [dr.x, dr.y, dr.z, dw.x, dw.y, dw.z]
}

/// The boundary-value problem of free flight: per-segment air controls (segments of `ticks` ticks each) that
/// carry a car from its start rotation and angular velocity to its end ones, under the analytic air model
/// (`fly`). Returns the controls and the remaining end error (radians of rotation, rad/s of angular velocity).
///
/// The same fit as `solve_bvp_with` with `fly` as the forward model, faster: a control of segment j cannot change
/// the flight before segment j, so its finite difference flies from the state at segment j's start, kept from
/// the unperturbed flight. `fly` is a pure function stepped segment by segment, so the result is the same to
/// the bit.
#[must_use]
pub fn solve_bvp(
    rot_start: Mat3A,
    omega_start: Vec3A,
    rot_end: Mat3A,
    omega_end: Vec3A,
    ticks: &[u32],
    prior: &[AirControls],
) -> (Vec<AirControls>, f32, f32) {
    let mut residual = |u: &[f32]| -> [f32; 6] {
        let (rot, omega) = fly(rot_start, omega_start, &segments_of(u, ticks));
        residual_of(rot, omega, rot_end, omega_end)
    };
    let mut jacobian = |u: &[f32], r: &[f32; 6]| -> Vec<[f32; 6]> {
        let segments = segments_of(u, ticks);
        // The state at the start of each segment.
        let mut starts = Vec::with_capacity(segments.len());
        let mut state = (rot_start, omega_start);
        for segment in &segments {
            starts.push(state);
            state = fly(state.0, state.1, std::slice::from_ref(segment));
        }
        (0..u.len())
            .map(|k| {
                let j = k / 3;
                let mut up = u.to_vec();
                up[k] += DIFFERENCE;
                let perturbed = segments_of(&up, ticks);
                let (rot, omega) = fly(starts[j].0, starts[j].1, &perturbed[j..]);
                let rp = residual_of(rot, omega, rot_end, omega_end);
                std::array::from_fn(|i| (rp[i] - r[i]) / DIFFERENCE)
            })
            .collect()
    };
    levenberg_marquardt(&mut residual, &mut jacobian, prior)
}

/// `solve_bvp` with the forward model as a function of the segments (RocketSim itself for a flipping car): a
/// Levenberg-Marquardt fit of the six end conditions that stays as close as possible to `prior` (one control
/// per segment). Two or three segments pin the end state down; with more, the prior chooses among the
/// solutions.
pub fn solve_bvp_with(
    forward: &mut ForwardModel<'_>,
    rot_end: Mat3A,
    omega_end: Vec3A,
    ticks: &[u32],
    prior: &[AirControls],
) -> (Vec<AirControls>, f32, f32) {
    let forward = std::cell::RefCell::new(forward);
    let residual_at = |u: &[f32]| -> [f32; 6] {
        let (rot, omega) = (forward.borrow_mut())(&segments_of(u, ticks));
        residual_of(rot, omega, rot_end, omega_end)
    };
    let mut residual = |u: &[f32]| residual_at(u);
    let mut jacobian = |u: &[f32], r: &[f32; 6]| -> Vec<[f32; 6]> {
        (0..u.len())
            .map(|k| {
                let mut up = u.to_vec();
                up[k] += DIFFERENCE;
                let rp = residual_at(&up);
                std::array::from_fn(|i| (rp[i] - r[i]) / DIFFERENCE)
            })
            .collect()
    };
    levenberg_marquardt(&mut residual, &mut jacobian, prior)
}

/// The Levenberg-Marquardt fit of `solve_bvp_with`, given the residual and its finite-difference Jacobian
/// (one column of six per parameter).
fn levenberg_marquardt(
    residual: &mut dyn FnMut(&[f32]) -> [f32; 6],
    jacobian_at: &mut Jacobian<'_>,
    prior: &[AirControls],
) -> (Vec<AirControls>, f32, f32) {
    let u0 = to_vec(prior);
    let dim = u0.len();
    let cost_of = |u: &[f32], r: &[f32; 6]| -> f32 {
        let prior_cost: f32 = u.iter().zip(&u0).map(|(a, b)| (a - b) * (a - b)).sum();
        r.iter().map(|x| x * x).sum::<f32>() + PRIOR_WEIGHT * PRIOR_WEIGHT * prior_cost
    };
    let mut u = u0.clone();
    let mut lambda = 1.0f32;
    let mut r = residual(&u);
    let mut current = cost_of(&u, &r);
    // The Jacobian is kept while steps are rejected: `u` does not change then, only `lambda`.
    let mut jacobian: Option<Vec<[f32; 6]>> = None;
    for _ in 0..12 {
        if jacobian.is_none() {
            jacobian = Some(jacobian_at(&u, &r));
        }
        let jac = jacobian.as_ref().expect("computed above");
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
        let Some(delta) = solve_linear(a, g) else {
            break;
        };
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
            jacobian = None;
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

/// Solves `m x = b` by Gaussian elimination with partial pivoting; `None` for a (near) singular matrix.
fn solve_linear(mut m: Vec<Vec<f32>>, mut b: Vec<f32>) -> Option<Vec<f32>> {
    let dim = b.len();
    for col in 0..dim {
        let pivot = (col..dim)
            .max_by(|&x, &y| m[x][col].abs().total_cmp(&m[y][col].abs()))
            .unwrap_or(col);
        if m[pivot][col].abs() < 1e-9 {
            return None;
        }
        m.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..dim {
            let factor = m[row][col] / m[col][col];
            for k in col..dim {
                m[row][k] -= factor * m[col][k];
            }
            b[row] -= factor * b[col];
        }
    }
    for col in (0..dim).rev() {
        let tail: f32 = (col + 1..dim).map(|k| m[col][k] * b[k]).sum();
        b[col] = (b[col] - tail) / m[col][col];
    }
    Some(b)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pure inputs are recovered from the angular velocity they produce over one tick.
    #[test]
    fn the_inverse_recovers_pure_inputs() {
        let rot = Mat3A::IDENTITY;
        for controls in [
            AirControls {
                pitch: 0.7,
                yaw: 0.0,
                roll: 0.0,
            },
            AirControls {
                pitch: 0.0,
                yaw: -0.4,
                roll: 0.0,
            },
            AirControls {
                pitch: 0.0,
                yaw: 0.0,
                roll: 1.0,
            },
        ] {
            let (_, end) = fly(rot, Vec3A::ZERO, &[(controls, 1)]);
            let solved = invert(rot, Vec3A::ZERO, end, 1.0 / 120.0);
            assert!(
                (solved.pitch - controls.pitch).abs() < 1e-3,
                "{solved:?} {controls:?}"
            );
            assert!(
                (solved.yaw - controls.yaw).abs() < 1e-3,
                "{solved:?} {controls:?}"
            );
            assert!(
                (solved.roll - controls.roll).abs() < 1e-3,
                "{solved:?} {controls:?}"
            );
        }
    }

    /// Over a span the refinement closes most of what the analytic start misses.
    #[test]
    fn the_span_solve_reaches_the_end_angular_velocity() {
        let rot = Mat3A::from_quat(Quat::from_rotation_z(0.6));
        let start = Vec3A::new(0.5, -1.0, 2.0);
        let truth = AirControls {
            pitch: 0.5,
            yaw: -0.3,
            roll: 0.2,
        };
        let (_, end) = fly(rot, start, &[(truth, 8)]);
        let error = |iterations| {
            let solved = solve_span(rot, start, end, 8, iterations);
            (fly(rot, start, &[(solved, 8)]).1 - end).length()
        };
        assert!(error(1) < error(0));
        assert!(error(1) < 0.05);
    }

    #[test]
    fn persistence_needs_a_calibrated_magnitude_and_a_short_lag() {
        assert_eq!(persistence(0, 0.05, 0.05), 0.0);
        assert_eq!(persistence(0, 0.25, 0.5), 0.0);
        assert_eq!(persistence(0, 0.05, 0.6), 0.758);
        assert_eq!(persistence(2, 0.15, 0.95), 0.642);
    }
}
