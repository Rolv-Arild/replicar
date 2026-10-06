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
