//! Benchmark of the input-inference heuristics of the supplied `external/RLCarInputSolver` (a Rust port of
//! `Solver::Solve`, `SolveAir` and `SolveGround`) against the TRUE inputs of RLBot recordings
//! (`states.jsonl`: every server tick, every car's physics, `air_state`, flags and `last_input`).
//!
//! For each pair of states d ticks apart (d = 1, 4, 8) of every car, the solver gets the two physical
//! states (no controls, no flags) and its output is compared with the true inputs applied between them
//! (`last_input` of packets n+1 ..= n+d: the input listed with a packet is the one applied in the tick
//! that ended at it) averaged over the span, and with two trivial baselines (all zeros; the previous
//! tick's true input held). Differences of the port from the C++ are listed in RESULTS.md and marked
//! `PORT NOTE` below.
//!
//! usage: benchmark_rlcis [--spans 1,4,8] [--strides 1,2,4] [--bodies match|octane] [--max-packets N]
//!                        [--per-game] <states.jsonl>...
//! (run from a directory with `collision_meshes/`)

use std::env;
use std::error::Error;
use std::fs::File;
use std::io::BufReader;
use std::path::{Path, PathBuf};

use glam::{Mat3A, Vec3A};
use rocketsim::consts::{GRAVITY_Z, TICK_TIME, UU_TO_BT, car, curves};
use rocketsim::{
    Arena, ArenaConfig, BallState, CarBodyConfig, CarControls, CarState, GameMode, RaycastHitInfo,
    Team,
};
use serde::Deserialize;

// ---------------------------------------------------------------------------------------------
// The solver (port)
// ---------------------------------------------------------------------------------------------

/// `SolverCarState` (the C++ also carries the position; the solver code never reads it, the scratch
/// arena does).
#[derive(Clone, Copy, Debug)]
struct SState {
    pos: Vec3A,
    rot: Mat3A, // columns forward, right, up
    vel: Vec3A,
    ang: Vec3A,
}

#[derive(Clone, Copy, Debug)]
struct SolverConfig {
    input_deadzone: f32,
    input_inverse_deadzone: f32,
    apply_deadzones: bool,
    clamp_controls: bool,
    steer_is_yaw: bool,
    /// Not in the C++: switch off the in-flip detection (and with it the stall and cancel rules).
    disable_inflip: bool,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            input_deadzone: 0.1,
            input_inverse_deadzone: 0.95,
            apply_deadzones: true,
            clamp_controls: true,
            steer_is_yaw: true,
            disable_inflip: false,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct SolveResult {
    controls: CarControls,
    is_on_ground: bool,
    flip_started: bool,
    is_flipping: bool,
    double_jumping: bool,
    /// Not in the C++: the in-flip branch took its stall rule.
    stall: bool,
    /// Not in the C++: the in-flip branch took its flip-cancel rule.
    cancel: bool,
}

fn zero_controls() -> CarControls {
    CarControls {
        throttle: 0.0,
        steer: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        roll: 0.0,
        jump: false,
        boost: false,
        handbrake: false,
    }
}

impl SolveResult {
    fn controls_vec(&self) -> [f32; 8] {
        ctl_vec(&self.controls)
    }
    fn new() -> Self {
        Self {
            controls: zero_controls(),
            is_on_ground: false,
            flip_started: false,
            is_flipping: false,
            double_jumping: false,
            stall: false,
            cancel: false,
        }
    }
}

/// `RS_SGN`: -1, 0 or 1.
fn sgn(x: f32) -> f32 {
    ((x > 0.0) as i32 - (x < 0.0) as i32) as f32
}

/// `Util::IsNear(val, target, marginLess, marginMore)`.
fn is_near(val: f32, target: f32, margin_less: f32, margin_more: f32) -> bool {
    val > target - (target * margin_less).abs() && val < target + (target * margin_more).abs()
}

/// `Util::Deadzone`.
fn deadzone(val: f32, dz: f32, inverse_dz: f32) -> f32 {
    let a = val.abs();
    if a < dz {
        0.0
    } else if a > inverse_dz {
        sgn(val)
    } else {
        val
    }
}

/// `Vec::operator*(RotMat)` and `RotMat::Dot(Vec)`: the vector in the frame of the rotation matrix
/// (columns forward, right, up).
fn local(rot: Mat3A, v: Vec3A) -> Vec3A {
    Vec3A::new(rot.x_axis.dot(v), rot.y_axis.dot(v), rot.z_axis.dot(v))
}

fn limit_to_max_car_speed(v: Vec3A) -> Vec3A {
    let len_sq = v.length_squared();
    if len_sq > car::MAX_SPEED * car::MAX_SPEED {
        v / len_sq.sqrt() * car::MAX_SPEED
    } else {
        v
    }
}

/// `int tickDelta = deltaTime / RL_TICKTIME`. PORT NOTE: rounded (the C++ truncates, which turns an
/// exact 4-tick `deltaTime` into 3 whenever the float quotient lands just below 4).
fn tick_count(delta_time: f32) -> i32 {
    (delta_time / TICK_TIME).round() as i32
}

/// `ReverseAirOrientInputs` (Sam Mish's inverse of the aerial control torque, with the max-angular-speed
/// scale-up of the C++); result is (roll, pitch, yaw), each clamped to [-1, 1].
fn reverse_air_orient_inputs(
    ang_vel_before: Vec3A,
    mut ang_vel_after: Vec3A,
    rot: Mat3A,
    dt: f32,
) -> Vec3A {
    let max_ang = car::MAX_ANG_SPEED;
    if is_near(ang_vel_after.length_squared(), max_ang * max_ang, 0.01, 0.01) {
        // The C++ comment: otherwise partial inputs come out while turning at max speed.
        ang_vel_after *= 1.25;
    }
    let t = Vec3A::new(-36.0796, -12.1460, 8.9196);
    let d = Vec3A::new(-4.47166, -2.7982, -1.8865);
    // Net torque in world, then local coordinates.
    let tau = local(rot, (ang_vel_after - ang_vel_before) / dt);
    let omega_local = local(rot, ang_vel_before);
    let rhs = tau - d * omega_local;
    let result = Vec3A::new(
        rhs.x / t.x,
        rhs.y / (t.y + sgn(rhs.y) * omega_local.y * d.y),
        rhs.z / (t.z - sgn(rhs.z) * omega_local.z * d.z),
    );
    result.clamp(Vec3A::splat(-1.0), Vec3A::splat(1.0))
}

fn flat_look_at_rot(forward: Vec3A) -> Mat3A {
    // RotMat::LookAt(forward, up = (0, 0, 1)); right = up x forward.
    let up = Vec3A::Z;
    let right = up.cross(forward).normalize_or_zero();
    let up = forward.cross(right);
    Mat3A::from_cols(forward, right, up)
}

/// `SolveAir`.
fn solve_air(from: &SState, to: &SState, delta_time: f32, config: &SolverConfig) -> SolveResult {
    // PORT NOTE: the C++ constants BOOST_ACCEL and THROTTLE_AIR_FORCE are in Bullet units (the factor
    // 1 / 2.4 = 50 / 120 converts them to UU/s per tick); here they come from the pinned Rust
    // RocketSim's UU values (991.67 and 66.67 UU/s^2) times UU_TO_BT.
    let boost_accel_bt = car::boost::ACCEL_AIR * UU_TO_BT;
    let throttle_air_force_bt = car::drive::THROTTLE_AIR_ACCEL * UU_TO_BT;
    let gravity = Vec3A::new(0.0, 0.0, GRAVITY_Z);
    let tick_forces_scale = 1.0 / 2.4;

    let tick_delta = tick_count(delta_time);
    let forces_scale = tick_forces_scale * tick_delta as f32;

    let mut result = SolveResult::new();
    result.is_on_ground = false;
    let mut controls = zero_controls();

    let extrap_vel = limit_to_max_car_speed(from.vel + gravity * delta_time);
    let delta_vel = to.vel - extrap_vel;
    let delta_vel_local = local(from.rot, delta_vel);

    // Throttle and boost.
    {
        const MIN_BOOST_OR_THROTTLE_OTHER_DELTAS: f32 = 6.0;
        if delta_vel_local.y.abs() + delta_vel_local.z.abs() < MIN_BOOST_OR_THROTTLE_OTHER_DELTAS {
            const MIN_FORWARD_DELTA: f32 = 2.0;
            if delta_vel_local.x > MIN_FORWARD_DELTA {
                let expected_boost_accel = boost_accel_bt * forces_scale;
                let expected_boost_vel =
                    limit_to_max_car_speed(extrap_vel + from.rot.x_axis * expected_boost_accel);
                if expected_boost_vel.distance(to.vel) < expected_boost_accel / 2.0 {
                    controls.boost = true;
                }
            }
            if !controls.boost {
                let expected_throttle_accel = throttle_air_force_bt * forces_scale;
                if is_near(delta_vel_local.x.abs(), expected_throttle_accel, 0.2, 0.6) {
                    controls.throttle = delta_vel_local.x / expected_throttle_accel;
                }
            }
        }
    }

    // Flip start and direction.
    result.flip_started = false;
    {
        let flat_delta_vel = delta_vel.truncate().length();
        if is_near(
            flat_delta_vel,
            car::flip::INITIAL_VEL_SCALE,
            0.3,
            car::flip::BACKWARD_IMPULSE_MAX_SPEED_SCALE + 0.3,
        ) {
            let forward_speed = from.vel.dot(from.rot.x_axis);
            let forward_speed_ratio = forward_speed.abs() / car::MAX_SPEED;
            let flat_rot = flat_look_at_rot((from.rot.x_axis * Vec3A::new(1.0, 1.0, 0.0)).normalize_or_zero());
            let mut delta_vel_flip = local(flat_rot, delta_vel);

            let is_backwards_dodge = if forward_speed.abs() < 100.0 {
                delta_vel_flip.x < 0.0
            } else {
                (delta_vel_flip.x >= 0.0) != (forward_speed >= 0.0)
            };
            let max_speed_scale_x = if is_backwards_dodge {
                car::flip::BACKWARD_IMPULSE_MAX_SPEED_SCALE
            } else {
                car::flip::FORWARD_IMPULSE_MAX_SPEED_SCALE
            };
            delta_vel_flip.x /= (max_speed_scale_x - 1.0) * forward_speed_ratio + 1.0;
            delta_vel_flip.y /=
                (car::flip::SIDE_IMPULSE_MAX_SPEED_SCALE - 1.0) * forward_speed_ratio + 1.0;

            let denom = car::flip::INITIAL_VEL_SCALE.max(flat_delta_vel);
            let flip_dir_forward = delta_vel_flip.x / denom;
            let flip_dir_right = delta_vel_flip.y / denom;
            let pitch = -flip_dir_forward;
            let yaw = flip_dir_right;
            // Assume flip was done with maximum input.
            let scale_ratio = 1.0 / pitch.abs().max(yaw.abs());
            // PORT NOTE: a zero direction (NaN in the C++) becomes 0.
            let finite = |v: f32| if v.is_finite() { v } else { 0.0 };
            controls.pitch = finite(pitch * scale_ratio);
            controls.yaw = finite(yaw * scale_ratio);
            controls.jump = true;
            result.flip_started = true;
        }
    }

    // Double jump.
    result.double_jumping = false;
    if !result.flip_started && is_near(delta_vel_local.z, car::jump::IMMEDIATE_FORCE, 0.3, 0.3) {
        result.double_jumping = true;
        controls.jump = true;
    }

    if !result.flip_started && !result.double_jumping {
        let inputs = reverse_air_orient_inputs(from.ang, to.ang, from.rot, delta_time);
        controls.roll = inputs.x;
        controls.pitch = inputs.y;
        controls.yaw = inputs.z;
    }

    // In-flip detection.
    result.is_flipping = false;
    if !result.flip_started && !result.double_jumping && !config.disable_inflip {
        let tick_grav = GRAVITY_Z * TICK_TIME;
        let flip_z_damp_scale = 1.0 - car::flip::Z_DAMP_120;
        let expected_z_vel = if tick_delta > 1 {
            let m_n = flip_z_damp_scale.powi(tick_delta);
            m_n * from.vel.z + tick_grav * ((1.0 - m_n) / (1.0 - flip_z_damp_scale))
        } else {
            from.vel.z * flip_z_damp_scale + tick_grav
        };

        if is_near(to.vel.z, expected_z_vel, 0.4, 0.4) {
            result.is_flipping = true;
            let ang_vel_local_from = local(from.rot, from.ang);
            let yaw_ang_vel = ang_vel_local_from.z;
            let roll_ang_vel = -ang_vel_local_from.x;

            let is_stall = (yaw_ang_vel.abs() > 0.2 && roll_ang_vel.abs() > 0.2)
                && (sgn(yaw_ang_vel) != sgn(roll_ang_vel));
            if is_stall {
                controls.yaw = sgn(yaw_ang_vel);
                controls.roll = -controls.yaw;
                controls.jump = true;
                result.stall = true;
            } else {
                // Check for flip cancel (full only; the C++ leaves partial cancels as a TODO).
                let ang_vel_local_to = local(to.rot, to.ang);
                const MIN_PITCH_DELTA_PER_TICK: f32 = 0.05;
                if ang_vel_local_from.y.abs()
                    > ang_vel_local_to.y.abs() + MIN_PITCH_DELTA_PER_TICK * tick_delta as f32
                {
                    controls.pitch = sgn(ang_vel_local_from.y);
                    result.cancel = true;
                }
            }
        }
    }

    // Continued jump from the ground.
    if !result.flip_started && !result.is_flipping && !result.double_jumping {
        let expected_jump_accel = car::jump::ACCEL * delta_time;
        let max_other_accel = expected_jump_accel / 10.0;
        if is_near(delta_vel_local.z, expected_jump_accel, 0.35, 0.35)
            && (delta_vel_local.x.abs() + delta_vel_local.y.abs()) < max_other_accel
        {
            controls.jump = true;
        }
    }

    if config.steer_is_yaw {
        controls.steer = controls.yaw;
    }
    result.controls = controls;
    result
}

/// The quantities the C++ ground solver reads off the car after `CheckOnGround(toState)`: the turn
/// accelerations (change of the local yaw rate per second) for steer 0 and steer 1, from RocketSim's
/// friction impulses on the TO state. PORT NOTE: here a scratch arena steps one tick (see `Scratch`).
#[derive(Clone, Copy, Debug, Default)]
struct GroundInfo {
    on_ground: bool,
    steer_turn_accel: [f32; 2],
}

/// `SolveGround`.
fn solve_ground(
    from: &SState,
    to: &SState,
    delta_time: f32,
    config: &SolverConfig,
    info: &GroundInfo,
) -> SolveResult {
    let gravity = Vec3A::new(0.0, 0.0, GRAVITY_Z);

    let mut result = SolveResult::new();
    result.is_on_ground = true;
    let mut controls = zero_controls();

    let local_vel_from = local(from.rot, from.vel);
    let local_vel_to = local(to.rot, to.vel);
    let local_ang_vel_from = local(from.rot, from.ang);
    let local_ang_vel_to = local(to.rot, to.ang);
    let forward_speed = local_vel_from.x;

    let mut extrap_vel = from.vel;
    // 1. Gravity; 2. the wheels push back; 3. velocity lost to the turn; 4. max speed.
    extrap_vel += gravity * delta_time * 2.0;
    extrap_vel -= from.rot.z_axis * GRAVITY_Z * delta_time;
    let vel_conserve_fraction = from.rot.x_axis.dot(to.rot.x_axis).max(0.0);
    extrap_vel *= vel_conserve_fraction;
    extrap_vel = limit_to_max_car_speed(extrap_vel);

    let delta_vel = to.vel - extrap_vel;
    let delta_vel_local = local(from.rot, delta_vel);
    let local_accel = delta_vel_local / delta_time;

    // Steer from the simulated turn acceleration for steer 0 and 1.
    {
        let turn_accel = (local_ang_vel_to.z - local_ang_vel_from.z) / delta_time;
        let zero_steer_turn_accel = info.steer_turn_accel[0];
        let mut full_steer_turn_accel_delta = info.steer_turn_accel[1] - zero_steer_turn_accel;
        const MIN_TURN_ACCEL_DELTA: f32 = 0.02;
        if full_steer_turn_accel_delta.abs() < MIN_TURN_ACCEL_DELTA {
            full_steer_turn_accel_delta = MIN_TURN_ACCEL_DELTA * sgn(full_steer_turn_accel_delta);
        }
        controls.steer = (turn_accel - zero_steer_turn_accel) / full_steer_turn_accel_delta;
    }

    let forward_accel = local_accel.x;
    let forward_dir = sgn(local_vel_from.x);
    let rel_forward_accel = forward_accel * forward_dir;

    // Throttle and boost.
    {
        const TORQUE_CONVERT_FACTOR: f32 = 4.0 / 3.0;
        // CAR_MASS_BT (180) is `car::MASS_BT`.
        let drive_accel_const = car::drive::THROTTLE_TORQUE_AMOUNT / (car::MASS_BT / 3.0) * TORQUE_CONVERT_FACTOR;
        let forward_speed_from = from.vel.dot(to.rot.x_axis);
        let forward_speed_to = to.vel.dot(to.rot.x_axis);
        // To account for gravity along the car's forward axis when on walls.
        let gravity_speed = (gravity * delta_time).dot(to.rot.x_axis);

        let drive_accel = ((forward_speed_to - gravity_speed) - forward_speed_from) / delta_time;
        let drive_speed_scale = curves::DRIVE_SPEED_TORQUE_FACTOR.get_output(forward_speed.abs());

        if rel_forward_accel > 0.0 {
            // Accelerating.
            let expected_throttle_accel = (drive_accel_const * drive_speed_scale).max(0.01);
            let mut throttle_mag = drive_accel.abs() / expected_throttle_accel;
            const TURNING_THROTTLE_SCALE_ADD: f32 = 0.2;
            const MAX_TURN_ANGVEL: f32 = 4.4;
            let turn_scale = (local_ang_vel_from.z.abs() / MAX_TURN_ANGVEL).min(1.0);
            throttle_mag *= 1.0 + TURNING_THROTTLE_SCALE_ADD * turn_scale;
            controls.throttle = throttle_mag.clamp(0.0, 1.0) * forward_dir;

            const THROTTLE_MAG_BOOST_THRESH: f32 = 2.1;
            if throttle_mag > THROTTLE_MAG_BOOST_THRESH {
                controls.boost = true;
            }
        } else {
            // Slowing down: brake input or coasting brake.
            let brake_accel = drive_accel.abs();
            let expected_brake_accel = car::drive::BRAKE_TORQUE_AMOUNT * TORQUE_CONVERT_FACTOR;
            let expected_coast_accel = expected_brake_accel * car::drive::COASTING_BRAKE_FACTOR;
            if brake_accel - expected_coast_accel > (expected_brake_accel - expected_coast_accel) / 2.0 {
                controls.throttle = -1.0 * forward_dir;
                if controls.throttle == 1.0 && brake_accel > expected_brake_accel * 1.25 {
                    controls.boost = true;
                }
            } else {
                const MIN_COAST_BRAKE_ACCEL: f32 = 30.0;
                if brake_accel < MIN_COAST_BRAKE_ACCEL && drive_speed_scale < 0.01 {
                    controls.throttle = 1.0 * forward_dir;
                } else {
                    controls.throttle = 0.0;
                }
            }
        }
    }

    // Handbrake from the alignment trend of the velocity with the facing direction.
    {
        const MIN_HANDBRAKE_VEL: f32 = 100.0;
        // (The C++ compares the angular speed with the same 100 constant; kept.)
        if to.vel.length() > MIN_HANDBRAKE_VEL || to.ang.length() > MIN_HANDBRAKE_VEL {
            let vel_alignment_from =
                local_vel_from.x.abs() / (local_vel_from.x.abs() + local_vel_from.y.abs());
            let vel_alignment_to =
                local_vel_to.x.abs() / (local_vel_to.x.abs() + local_vel_to.y.abs());
            const HANDBRAKE_ALIGNMENT_SCALE: f32 = 0.15;
            let expected_alignment_drop =
                (car::drive::POWERSLIDE_RISE_RATE * delta_time) * HANDBRAKE_ALIGNMENT_SCALE;
            let expected_alignment_rise =
                (car::drive::POWERSLIDE_FALL_RATE * delta_time) * HANDBRAKE_ALIGNMENT_SCALE;
            if vel_alignment_to < vel_alignment_from - expected_alignment_drop {
                controls.handbrake = true;
            } else if vel_alignment_to > vel_alignment_from + expected_alignment_rise {
                controls.handbrake = false;
            } else {
                const HANDBRAKE_ALIGNMENT_THRESH: f32 = 0.91;
                controls.handbrake = vel_alignment_to < HANDBRAKE_ALIGNMENT_THRESH;
            }
        }
    }

    // Jump from the vertical velocity.
    {
        const MIN_JUMPING_VEL_DELTA: f32 = 100.0;
        const MAX_FROM_Z_VEL: f32 = 50.0;
        const MIN_TO_Z_VEL: f32 = MIN_JUMPING_VEL_DELTA * 0.7;
        if local_vel_from.z.abs() < MAX_FROM_Z_VEL
            && local_vel_to.z > MIN_TO_Z_VEL
            && delta_vel_local.z > MIN_JUMPING_VEL_DELTA
        {
            controls.jump = true;
        }
    }

    if config.steer_is_yaw {
        controls.yaw = controls.steer;
    }
    result.controls = controls;
    result
}

/// `Solver::Solve` (dispatch on the TO state's wheel contact, deadzones, clamping).
fn solve(
    from: &SState,
    to: &SState,
    delta_time: f32,
    config: &SolverConfig,
    info: &GroundInfo,
) -> SolveResult {
    assert!(delta_time >= TICK_TIME * 0.999, "delta time below one tick");
    let mut result = if info.on_ground {
        solve_ground(from, to, delta_time, config, info)
    } else {
        solve_air(from, to, delta_time, config)
    };
    let c = &mut result.controls;
    if config.apply_deadzones {
        for v in [&mut c.throttle, &mut c.steer, &mut c.pitch, &mut c.yaw, &mut c.roll] {
            *v = deadzone(*v, config.input_deadzone, config.input_inverse_deadzone);
        }
    }
    if config.clamp_controls {
        // CarControls::ClampFix (a NaN becomes 0 here).
        for v in [&mut c.throttle, &mut c.steer, &mut c.pitch, &mut c.yaw, &mut c.roll] {
            *v = if v.is_finite() { v.clamp(-1.0, 1.0) } else { 0.0 };
        }
    }
    result
}

// ---------------------------------------------------------------------------------------------
// Scratch arenas (the C++ `CheckOnGround` and the wheel-friction part of `SolveGround`)
// ---------------------------------------------------------------------------------------------

const BODY_SIZES: [[f32; 3]; 6] = [
    [120.507, 86.6994, 38.6591],
    [130.427, 85.7799, 33.8],
    [131.32, 87.1704, 31.8944],
    [133.992, 83.021, 32.8],
    [129.519, 84.6879, 36.6591],
    [123.22, 79.2103, 44.1591],
];

fn body_config(idx: usize) -> CarBodyConfig {
    [
        CarBodyConfig::OCTANE,
        CarBodyConfig::DOMINUS,
        CarBodyConfig::PLANK,
        CarBodyConfig::BREAKOUT,
        CarBodyConfig::HYBRID,
        CarBodyConfig::MERC,
    ][idx]
}

fn nearest_body(size: [f32; 3]) -> usize {
    let mut best = (f32::MAX, 0);
    for (i, s) in BODY_SIZES.iter().enumerate() {
        let d: f32 = (0..3).map(|k| (s[k] - size[k]).abs()).sum();
        if d < best.0 {
            best = (d, i);
        }
    }
    best.1
}

struct Scratch {
    arenas: Vec<Option<Arena>>,
}

impl Scratch {
    fn new() -> Self {
        Self { arenas: (0..6).map(|_| None).collect() }
    }

    fn arena(&mut self, body: usize) -> &mut Arena {
        self.arenas[body].get_or_insert_with(|| {
            let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
            arena.add_car(Team::Blue, body_config(body));
            arena
        })
    }

    fn place(&mut self, body: usize, s: &SState, on_ground_state: bool, controls: CarControls) {
        let arena = self.arena(body);
        // The C++ removes the ball from the arena (z = -500) so it does not get in the way.
        let mut ball = BallState::default();
        ball.phys.pos = Vec3A::new(0.0, 0.0, -500.0);
        arena.set_ball_state(ball);
        let mut cs = CarState::default();
        cs.phys.pos = s.pos;
        cs.phys.vel = s.vel;
        cs.phys.rot_mat = s.rot;
        cs.phys.ang_vel = s.ang;
        cs.is_on_ground = on_ground_state;
        cs.wheels_with_contact = [on_ground_state.then(RaycastHitInfo::default); 4];
        arena.set_car_state(0, cs);
        arena.refresh_car_sticky_gate(0);
        arena.set_car_controls(0, controls);
    }

    /// `CheckOnGround(toState)`. PORT NOTE: the C++ calls the car's `_PreTickUpdate` (wheel traces)
    /// and `_PostTickUpdate` without stepping; here one tick is stepped from a default car state and
    /// `is_on_ground` is read afterwards. RocketSim sets it from the wheel traces at the position
    /// before the step (3 or more wheels in contact), so the answer is the same quantity.
    fn on_ground(&mut self, body: usize, s: &SState) -> bool {
        self.place(body, s, false, zero_controls());
        let arena = self.arena(body);
        arena.step_tick();
        arena.get_car_state(0).is_on_ground
    }

    /// The simulated turn acceleration (rad/s^2, local yaw rate change per second) at steer 0 and 1.
    /// PORT NOTE: the C++ sets the front wheels' steer angle by hand and runs the wheel friction
    /// impulses of `deltaTime` on the car at the TO state; here a full tick is stepped with throttle
    /// 0, no handbrake and RocketSim's own steer angle for the TO state's speed, and the change of
    /// local yaw rate (local axes of the TO state) is divided by one tick.
    fn steer_turn_accels(&mut self, body: usize, s: &SState) -> [f32; 2] {
        let mut out = [0.0; 2];
        for (i, steer) in [0.0_f32, 1.0].into_iter().enumerate() {
            let mut c = zero_controls();
            c.steer = steer;
            self.place(body, s, true, c);
            let arena = self.arena(body);
            arena.step_tick();
            let after = arena.get_car_state(0).phys.ang_vel;
            out[i] = (local(s.rot, after).z - local(s.rot, s.ang).z) / TICK_TIME;
        }
        out
    }

    fn ground_info(&mut self, body: usize, s: &SState) -> GroundInfo {
        let on_ground = self.on_ground(body, s);
        let steer_turn_accel = if on_ground { self.steer_turn_accels(body, s) } else { [0.0; 2] };
        GroundInfo { on_ground, steer_turn_accel }
    }
}

// ---------------------------------------------------------------------------------------------
// Recording
// ---------------------------------------------------------------------------------------------

#[derive(Deserialize, Default)]
struct V3 {
    #[serde(default)]
    x: f32,
    #[serde(default)]
    y: f32,
    #[serde(default)]
    z: f32,
}

#[derive(Deserialize, Default)]
struct V2 {
    #[serde(default)]
    x: f32,
    #[serde(default)]
    y: f32,
}

#[derive(Deserialize, Default)]
struct Euler {
    #[serde(default)]
    pitch: f32,
    #[serde(default)]
    yaw: f32,
    #[serde(default)]
    roll: f32,
}

#[derive(Deserialize, Default)]
struct Phys {
    #[serde(default)]
    location: V3,
    #[serde(default)]
    rotation: Euler,
    #[serde(default)]
    velocity: V3,
    #[serde(default)]
    angular_velocity: V3,
}

#[derive(Deserialize, Default)]
struct Input {
    #[serde(default)]
    throttle: f32,
    #[serde(default)]
    steer: f32,
    #[serde(default)]
    pitch: f32,
    #[serde(default)]
    yaw: f32,
    #[serde(default)]
    roll: f32,
    #[serde(default)]
    jump: bool,
    #[serde(default)]
    boost: bool,
    #[serde(default)]
    handbrake: bool,
}

#[derive(Deserialize, Default)]
struct Hitbox {
    #[serde(default)]
    length: f32,
    #[serde(default)]
    width: f32,
    #[serde(default)]
    height: f32,
}

#[derive(Deserialize)]
struct PlayerRow {
    physics: Phys,
    #[serde(default)]
    boost: f32,
    #[serde(default)]
    air_state: u8,
    #[serde(default)]
    has_double_jumped: bool,
    #[serde(default)]
    has_dodged: bool,
    #[serde(default)]
    dodge_dir: V2,
    #[serde(default)]
    dodge_elapsed: f32,
    #[serde(default)]
    demolished_timeout: f32,
    #[serde(default)]
    is_bot: bool,
    #[serde(default)]
    last_input: Input,
    #[serde(default)]
    hitbox: Hitbox,
}

#[derive(Deserialize)]
struct MatchInfo {
    frame_num: u64,
    match_phase: u8,
}

#[derive(Deserialize)]
struct PacketRow {
    players: Vec<PlayerRow>,
    match_info: MatchInfo,
}

#[derive(Deserialize)]
struct Row {
    packet: PacketRow,
}

struct Player {
    s: SState,
    boost: f32,
    air_state: u8,
    has_double_jumped: bool,
    has_dodged: bool,
    dodge_dir: [f32; 2],
    dodge_elapsed: f32,
    demolished: bool,
    is_bot: bool,
    ctl: CarControls,
    body: usize,
}

struct Packet {
    frame: u64,
    phase: u8,
    players: Vec<Player>,
}

/// RLBot Euler angles to the rotation matrix with columns forward, right, up.
fn matrix(e: &Euler) -> Mat3A {
    let (cp, sp, cy, sy, cr, sr) =
        (e.pitch.cos(), e.pitch.sin(), e.yaw.cos(), e.yaw.sin(), e.roll.cos(), e.roll.sin());
    Mat3A::from_cols(
        Vec3A::new(cp * cy, cp * sy, sp),
        Vec3A::new(sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp),
        Vec3A::new(-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp),
    )
}

fn v3(v: &V3) -> Vec3A {
    Vec3A::new(v.x, v.y, v.z)
}

fn load(path: &Path, max_packets: usize, octane: bool) -> Result<Vec<Packet>, Box<dyn Error>> {
    use std::io::BufRead;
    let mut packets: Vec<Packet> = Vec::new();
    for line in BufReader::with_capacity(1 << 20, File::open(path)?).lines() {
        if packets.len() >= max_packets {
            break;
        }
        let Ok(row) = serde_json::from_str::<Row>(&line?) else {
            continue;
        };
        let p = row.packet;
        if packets.last().is_some_and(|q| q.frame == p.match_info.frame_num) {
            continue; // repeated packet
        }
        let players = p
            .players
            .iter()
            .map(|pl| Player {
                s: SState {
                    pos: v3(&pl.physics.location),
                    rot: matrix(&pl.physics.rotation),
                    vel: v3(&pl.physics.velocity),
                    ang: v3(&pl.physics.angular_velocity),
                },
                boost: pl.boost,
                air_state: pl.air_state,
                has_double_jumped: pl.has_double_jumped,
                has_dodged: pl.has_dodged,
                dodge_dir: [pl.dodge_dir.x, pl.dodge_dir.y],
                dodge_elapsed: pl.dodge_elapsed,
                demolished: pl.demolished_timeout >= 0.0,
                is_bot: pl.is_bot,
                ctl: CarControls {
                    throttle: pl.last_input.throttle,
                    steer: pl.last_input.steer,
                    pitch: pl.last_input.pitch,
                    yaw: pl.last_input.yaw,
                    roll: pl.last_input.roll,
                    jump: pl.last_input.jump,
                    boost: pl.last_input.boost,
                    handbrake: pl.last_input.handbrake,
                },
                body: if octane {
                    0
                } else {
                    nearest_body([pl.hitbox.length, pl.hitbox.width, pl.hitbox.height])
                },
            })
            .collect();
        packets.push(Packet { frame: p.match_info.frame_num, phase: p.match_info.match_phase, players });
    }
    let max_players = packets.iter().map(|p| p.players.len()).max().unwrap_or(0);
    packets.retain(|p| p.players.len() == max_players);
    Ok(packets)
}

// ---------------------------------------------------------------------------------------------
// Scoring
// ---------------------------------------------------------------------------------------------

const CHANNELS: [&str; 8] = ["throttle", "steer", "pitch", "yaw", "roll", "boost", "handbrake", "jump"];
const BINARY: [bool; 8] = [false, false, false, false, false, true, true, true];
const CLASSES: [&str; 4] = ["ground", "air, free flight", "air, jump/dodge state", "mixed (takeoff/landing)"];
const POPS: [&str; 3] = ["all", "bots", "humans"];
const NPRED: usize = 6;
const PREDS: [&str; NPRED] = [
    "solver",
    "solver, no in-flip rule",
    "solver, no deadzones",
    "zeros",
    "previous tick held",
    "solver, steer simulated at the FROM state",
];
const P_FROMSIM: usize = 5;
const P_SOLVER: usize = 0;
const P_NOFLIP: usize = 1;
const P_RAW: usize = 2;
const P_ZERO: usize = 3;
const P_PREV: usize = 4;
const NEV: usize = 12;
const EVENTS: [&str; 9] = [
    "flip start (has_dodged rises)",
    "double jump (has_double_jumped rises)",
    "in-flip rule (true: the flip's vertical-velocity damping acts in the span: flip time 0.15-0.65 s and falling or under 0.21 s)",
    "stall rule (true: the car is in a flip (air_state Dodging at span start) and the yaw and roll inputs of the last tick are opposite, both > 0.5)",
    "flip cancel rule (true: the car is in a flip and the mean pitch input is above 0.5 in the cancel direction, the sign of the local pitch rate)",
    "jump press (rising edge of the jump input)",
    "ground/air dispatch (positive = on ground; true: air_state OnGround)",
    "jump press, ground solver only",
    "in-flip rule against the broader label (air_state Dodging at span start)",
];

#[derive(Clone, Copy, Default)]
struct Acc {
    n: u64,
    sum_abs: f64,
    within: u64,
    tp: u64,
    fp: u64,
    fneg: u64,
    tn: u64,
}

impl Acc {
    fn add(&mut self, o: &Acc) {
        self.n += o.n;
        self.sum_abs += o.sum_abs;
        self.within += o.within;
        self.tp += o.tp;
        self.fp += o.fp;
        self.fneg += o.fneg;
        self.tn += o.tn;
    }
    fn confusion(&mut self, truth: bool, pred: bool) {
        match (truth, pred) {
            (true, true) => self.tp += 1,
            (false, true) => self.fp += 1,
            (true, false) => self.fneg += 1,
            (false, false) => self.tn += 1,
        }
    }
}

struct Stats {
    spans: usize,
    chan: Vec<Acc>,   // [game][span][class][pop(2)][chan][pred]
    events: Vec<Acc>, // [game][span][class][pop(2)][event]
    sat: Vec<Acc>,    // [game][span][saturated(2)][pitch, yaw, roll]: free-flight inverse aerial by angular speed
    cross: Vec<u64>,  // [game][span][pop(2)][throttle, steer][speed band 3][truth bucket 3][pred bucket 3], ground class
    flip_angle: Vec<Vec<f32>>, // [game][span][pop(2)]: angle error of the solver's flip direction, degrees
    flip_angle_naive: Vec<Vec<f32>>, // always-forward baseline
}

impl Stats {
    fn new(games: usize, spans: usize) -> Self {
        Self {
            spans,
            chan: vec![Acc::default(); games * spans * 4 * 2 * 8 * NPRED],
            events: vec![Acc::default(); games * spans * 4 * 2 * NEV],
            sat: vec![Acc::default(); games * spans * 2 * 3],
            cross: vec![0; games * spans * 2 * 2 * 3 * 9],
            flip_angle: vec![Vec::new(); games * spans * 2],
            flip_angle_naive: vec![Vec::new(); games * spans * 2],
        }
    }
    fn ci(&self, g: usize, s: usize, c: usize, p: usize, ch: usize, pr: usize) -> usize {
        ((((g * self.spans + s) * 4 + c) * 2 + p) * 8 + ch) * NPRED + pr
    }
    fn ei(&self, g: usize, s: usize, c: usize, p: usize, e: usize) -> usize {
        (((g * self.spans + s) * 4 + c) * 2 + p) * NEV + e
    }
    fn chan_sum(&self, games: &[usize], s: usize, class: Option<usize>, pop: usize, ch: usize, pr: usize) -> Acc {
        let mut a = Acc::default();
        for &g in games {
            for c in 0..4 {
                if class.is_some_and(|k| k != c) {
                    continue;
                }
                for p in 0..2 {
                    if pop != 0 && pop - 1 != p {
                        continue;
                    }
                    a.add(&self.chan[self.ci(g, s, c, p, ch, pr)]);
                }
            }
        }
        a
    }
    fn event_sum(&self, games: &[usize], s: usize, class: Option<usize>, pop: usize, e: usize) -> Acc {
        let mut a = Acc::default();
        for &g in games {
            for c in 0..4 {
                if class.is_some_and(|k| k != c) {
                    continue;
                }
                for p in 0..2 {
                    if pop != 0 && pop - 1 != p {
                        continue;
                    }
                    a.add(&self.events[self.ei(g, s, c, p, e)]);
                }
            }
        }
        a
    }
}

fn quantile(values: &mut [f32], q: f64) -> f32 {
    if values.is_empty() {
        return f32::NAN;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    values[((values.len() - 1) as f64 * q).round() as usize]
}

fn ctl_vec(c: &CarControls) -> [f32; 8] {
    [
        c.throttle,
        c.steer,
        c.pitch,
        c.yaw,
        c.roll,
        c.boost as u8 as f32,
        c.handbrake as u8 as f32,
        c.jump as u8 as f32,
    ]
}

fn angle_between(a: [f32; 2], b: [f32; 2]) -> f32 {
    let (la, lb) = ((a[0] * a[0] + a[1] * a[1]).sqrt(), (b[0] * b[0] + b[1] * b[1]).sqrt());
    if la < 1e-6 || lb < 1e-6 {
        return f32::NAN;
    }
    ((a[0] * b[0] + a[1] * b[1]) / (la * lb)).clamp(-1.0, 1.0).acos().to_degrees()
}

fn evaluate(
    packets: &[Packet],
    infos: &[Vec<GroundInfo>],
    game: usize,
    span_idx: usize,
    d: usize,
    stride: usize,
    stats: &mut Stats,
) {
    let n_players = packets[0].players.len();
    let dt = d as f32 * TICK_TIME;
    let cfg = SolverConfig::default();
    let cfg_noflip = SolverConfig { disable_inflip: true, ..cfg };
    let cfg_raw = SolverConfig { apply_deadzones: false, clamp_controls: false, ..cfg };

    // A sample needs d consecutive frames, all Active.
    let mut run = vec![0usize; packets.len()];
    for i in (0..packets.len().saturating_sub(1)).rev() {
        let ok = packets[i + 1].frame == packets[i].frame + 1 && packets[i].phase == 3 && packets[i + 1].phase == 3;
        run[i] = if ok { run[i + 1] + 1 } else { 0 };
    }

    for n in (0..packets.len().saturating_sub(d)).step_by(stride) {
        if run[n] < d {
            continue;
        }
        for k in 0..n_players {
            let a = &packets[n].players[k];
            let b = &packets[n + d].players[k];
            if (0..=d).any(|t| packets[n + t].players[k].demolished) {
                continue;
            }
            let pop = if a.is_bot { 0 } else { 1 };
            let states: Vec<u8> = (0..=d).map(|t| packets[n + t].players[k].air_state).collect();
            let class = if states.iter().all(|&s| s == 0) {
                0
            } else if states.iter().all(|&s| s != 0) {
                if states.iter().all(|&s| s == 4) { 1 } else { 2 }
            } else {
                3
            };

            // Truth: mean of the inputs applied in the d ticks of the span (packets n+1 ..= n+d);
            // boost counts only when there was boost to burn.
            let mut truth = [0.0f32; 8];
            for t in 1..=d {
                let mut v = ctl_vec(&packets[n + t].players[k].ctl);
                v[5] = (packets[n + t].players[k].ctl.boost && packets[n + t - 1].players[k].boost > 0.0) as u8 as f32;
                for c in 0..8 {
                    truth[c] += v[c] / d as f32;
                }
            }

            let info = &infos[n + d][k];
            let r = solve(&a.s, &b.s, dt, &cfg, info);
            let r_noflip = solve(&a.s, &b.s, dt, &cfg_noflip, info);
            let r_raw = solve(&a.s, &b.s, dt, &cfg_raw, info);
            let mut prev = ctl_vec(&a.ctl);
            prev[5] = (a.ctl.boost && (n == 0 || packets[n - 1].players[k].boost > 0.0)) as u8 as f32;
            // Variant: the steer pair of turn accelerations simulated at the FROM state instead of
            // the TO state (when the FROM state is on the ground too).
            let mut info_from = *info;
            if infos[n][k].on_ground {
                info_from.steer_turn_accel = infos[n][k].steer_turn_accel;
            }
            let r_fromsim = solve(&a.s, &b.s, dt, &cfg, &info_from);
            let preds: [[f32; 8]; NPRED] = [
                ctl_vec(&r.controls),
                ctl_vec(&r_noflip.controls),
                ctl_vec(&r_raw.controls),
                [0.0; 8],
                prev,
                ctl_vec(&r_fromsim.controls),
            ];

            for (pi, pred) in preds.iter().enumerate() {
                for ch in 0..8 {
                    let idx = stats.ci(game, span_idx, class, pop, ch, pi);
                    let acc = &mut stats.chan[idx];
                    let err = (pred[ch] - truth[ch]).abs();
                    acc.n += 1;
                    acc.sum_abs += err as f64;
                    if err <= 0.1 {
                        acc.within += 1;
                    }
                    if BINARY[ch] {
                        acc.confusion(truth[ch] >= 0.5, pred[ch] >= 0.5);
                    }
                }
            }

            // Events.
            let any_edge = |f: &dyn Fn(&Player) -> bool| (0..d).any(|t| !f(&packets[n + t].players[k]) && f(&packets[n + t + 1].players[k]));
            let flip_truth = any_edge(&|p| p.has_dodged);
            let dj_truth = any_edge(&|p| p.has_double_jumped);
            let jump_press_truth = (1..=d).any(|t| {
                packets[n + t].players[k].ctl.jump && !packets[n + t - 1].players[k].ctl.jump
            });
            let in_flip_truth = a.air_state == 3;
            let damp_truth = (0..d).any(|t| {
                let p = &packets[n + t].players[k];
                p.air_state == 3
                    && (0.15..=0.65).contains(&p.dodge_elapsed)
                    && (p.s.vel.z < 0.0 || p.dodge_elapsed < 0.21)
            });
            let spans_n = stats.spans;
            let ei = |e: usize| (((game * spans_n + span_idx) * 4 + class) * 2 + pop) * NEV + e;
            let (e0, e1, e2, e3, e4, e5, e6, e8) = (ei(0), ei(1), ei(2), ei(3), ei(4), ei(5), ei(6), ei(8));
            stats.events[e0].confusion(flip_truth, r.flip_started);
            stats.events[e1].confusion(dj_truth, r.double_jumping);
            stats.events[e2].confusion(damp_truth, r.is_flipping);
            stats.events[e8].confusion(in_flip_truth, r.is_flipping);
            {
                // Stall and cancel: scored over every sample (positives only exist in flips), so
                // false alarms in free flight count.
                let last = &packets[n + d].players[k].ctl;
                let stall_truth = in_flip_truth
                    && last.yaw.abs() > 0.5
                    && last.roll.abs() > 0.5
                    && (last.yaw > 0.0) != (last.roll > 0.0);
                stats.events[e3].confusion(stall_truth, r.stall);
                let local_pitch_rate = local(a.s.rot, a.s.ang).y;
                let cancel_truth = in_flip_truth && truth[2] * sgn(local_pitch_rate) > 0.5;
                stats.events[e4].confusion(cancel_truth, r.cancel);
            }
            stats.events[e5].confusion(jump_press_truth, r.controls.jump);
            stats.events[e6].confusion(states[d] == 0, r.is_on_ground);
            if info.on_ground {
                let e7 = ei(7);
                stats.events[e7].confusion(jump_press_truth, r.controls.jump);
            }

            if class == 1 {
                // The aerial formula cannot see a clamped angular velocity: split by angular speed.
                let saturated = (b.s.ang.length() > 5.45) as usize;
                for (c, ch) in [2usize, 3, 4].into_iter().enumerate() {
                    let acc = &mut stats.sat[((game * spans_n + span_idx) * 2 + saturated) * 3 + c];
                    let err = (r.controls_vec()[ch] - truth[ch]).abs();
                    acc.n += 1;
                    acc.sum_abs += err as f64;
                    if err <= 0.1 {
                        acc.within += 1;
                    }
                }
            }
            if class == 0 {
                let bucket = |v: f32, th: f32| if v < -th { 0 } else if v > th { 2 } else { 1 };
                let pred_vec = r.controls_vec();
                for (ch, th) in [(0usize, 0.5f32), (1, 0.25)] {
                    let speed = local(a.s.rot, a.s.vel).x.abs();
                    let band = if speed < 1000.0 { 0 } else if speed < 1400.0 { 1 } else { 2 };
                    let idx = (((((game * spans_n + span_idx) * 2 + pop) * 2 + ch) * 3 + band) * 3
                        + bucket(truth[ch], th))
                        * 3
                        + bucket(pred_vec[ch], th);
                    stats.cross[idx] += 1;
                }
            }

            // Flip direction of the true flip starts the solver also found.
            if flip_truth && r.flip_started {
                let truth_dir = b.dodge_dir; // (forward, right) of the car's latest dodge
                let pred_dir = [-r.controls.pitch, r.controls.yaw];
                let idx = (game * stats.spans + span_idx) * 2 + pop;
                let e = angle_between(pred_dir, truth_dir);
                if e.is_finite() {
                    stats.flip_angle[idx].push(e);
                    stats.flip_angle_naive[idx].push(angle_between([1.0, 0.0], truth_dir));
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------------------------

fn pct(a: u64, b: u64) -> String {
    if b == 0 { "-".into() } else { format!("{:.1}%", 100.0 * a as f64 / b as f64) }
}

fn print_channel_tables(stats: &Stats, games: &[usize], spans: &[usize], title: &str, preds: &[usize]) {
    println!("\n### {title}\n");
    for (si, &d) in spans.iter().enumerate() {
        for class in 0..4 {
            let chans: &[usize] = match class {
                0 => &[0, 1, 5, 6, 7],
                1 | 2 => &[0, 2, 3, 4, 5, 7],
                _ => &[0, 1, 2, 3, 4, 5, 6, 7],
            };
            for pop in 0..3 {
                let n0 = stats.chan_sum(games, si, Some(class), pop, 0, 0).n;
                if n0 == 0 {
                    continue;
                }
                println!("\n**{d}-tick spans, {}, {}** ({n0} car-spans)\n", CLASSES[class], POPS[pop]);
                let mut head = String::from("| channel |");
                let mut sep = String::from("| --- |");
                for &p in preds {
                    head += &format!(" {} |", PREDS[p]);
                    sep += " ---: |";
                }
                println!("{head}\n{sep}");
                for &ch in chans {
                    let mut row = format!("| {} |", CHANNELS[ch]);
                    for &p in preds {
                        let a = stats.chan_sum(games, si, Some(class), pop, ch, p);
                        if BINARY[ch] {
                            row += &format!(
                                " acc {} MAE {:.3} (TP {} FP {} FN {} TN {}) |",
                                pct(a.tp + a.tn, a.n),
                                a.sum_abs / a.n.max(1) as f64,
                                a.tp,
                                a.fp,
                                a.fneg,
                                a.tn
                            );
                        } else {
                            row += &format!(" MAE {:.3}, within 0.1 {} |", a.sum_abs / a.n.max(1) as f64, pct(a.within, a.n));
                        }
                    }
                    println!("{row}");
                }
            }
        }
    }
}

fn print_sat(stats: &Stats, games: &[usize], spans: &[usize]) {
    println!("
### Free-flight inverse aerial by angular speed at the end of the span
");
    println!("| span | angular speed | car-spans | pitch MAE / within 0.1 | yaw MAE / within 0.1 | roll MAE / within 0.1 |
| --- | --- | ---: | ---: | ---: | ---: |");
    for (si, &d) in spans.iter().enumerate() {
        for (sat, name) in ["below 5.45 rad/s", "at the 5.5 rad/s cap"].iter().enumerate() {
            let mut cells = Vec::new();
            let mut n = 0;
            for c in 0..3 {
                let mut a = Acc::default();
                for &g in games {
                    a.add(&stats.sat[((g * stats.spans + si) * 2 + sat) * 3 + c]);
                }
                n = a.n;
                cells.push(format!("{:.3} / {}", a.sum_abs / a.n.max(1) as f64, pct(a.within, a.n)));
            }
            println!("| {d} | {name} | {n} | {} | {} | {} |", cells[0], cells[1], cells[2]);
        }
    }
}

fn print_cross(stats: &Stats, games: &[usize], spans: &[usize]) {
    println!("\n### Ground throttle and steer: confusion of the solver (rows: truth mean over the span; columns: solver)\n");
    println!("Throttle buckets: below -0.5, within 0.5, above 0.5; steer buckets: below -0.25, within 0.25, above 0.25.\n");
    for (ch, name) in [(0usize, "throttle"), (1, "steer")] {
        println!("**{name}**\n");
        println!("| span | population | forward speed | truth - -> solver (-, 0, +) | truth 0 -> solver (-, 0, +) | truth + -> solver (-, 0, +) |\n| --- | --- | --- | --- | --- | --- |");
        for (si, &d) in spans.iter().enumerate() {
            for pop in 0..3 {
                for (band, bname) in ["below 1000", "1000-1400", "1400 and above"].iter().enumerate() {
                    let mut m = [[0u64; 3]; 3];
                    for &g in games {
                        for p in 0..2 {
                            if pop != 0 && pop - 1 != p {
                                continue;
                            }
                            for t in 0..3 {
                                for q in 0..3 {
                                    m[t][q] += stats.cross[(((((g * stats.spans + si) * 2 + p) * 2 + ch) * 3 + band) * 3 + t) * 3 + q];
                                }
                            }
                        }
                    }
                    let row = |t: usize| format!("{} / {} / {}", m[t][0], m[t][1], m[t][2]);
                    println!("| {d} | {} | {bname} | {} | {} | {} |", POPS[pop], row(0), row(1), row(2));
                }
            }
        }
        println!();
    }
}

fn print_events(stats: &Stats, games: &[usize], spans: &[usize]) {
    println!("\n### Events\n");
    for e in [0usize, 1, 2, 8, 3, 4, 5, 7, 6] {
        println!("\n**{}**\n", EVENTS[e]);
        println!("| span | population | truth positives | predicted positives | precision | recall | accuracy | TP | FP | FN | TN |\n| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |");
        for (si, &d) in spans.iter().enumerate() {
            for pop in 0..3 {
                let a = stats.event_sum(games, si, None, pop, e);
                if a.tp + a.fp + a.fneg + a.tn == 0 {
                    continue;
                }
                println!(
                    "| {d} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |",
                    POPS[pop],
                    a.tp + a.fneg,
                    a.tp + a.fp,
                    pct(a.tp, a.tp + a.fp),
                    pct(a.tp, a.tp + a.fneg),
                    pct(a.tp + a.tn, a.tp + a.fp + a.fneg + a.tn),
                    a.tp,
                    a.fp,
                    a.fneg,
                    a.tn
                );
            }
        }
    }
    // The flip rules by true class: where do the positive predictions fall?
    println!("\n**Flip-related rules: positive predictions by true class (bots and humans pooled)**\n");
    println!("| span | class | samples | solver dispatches to ground | in-flip rule fires | stall rule fires | cancel rule fires | double jump fires | flip start fires |\n| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |");
    for (si, &d) in spans.iter().enumerate() {
        for class in 0..4 {
            let f = |e: usize| {
                let a = stats.event_sum(games, si, Some(class), 0, e);
                a.tp + a.fp
            };
            let a = stats.event_sum(games, si, Some(class), 0, 2);
            let tot = a.tp + a.fp + a.fneg + a.tn;
            println!("| {d} | {} | {} | {} ({}) | {} ({}) | {} ({}) | {} ({}) | {} ({}) | {} ({}) |", CLASSES[class], tot, f(6), pct(f(6), tot), f(2), pct(f(2), tot), f(3), pct(f(3), tot), f(4), pct(f(4), tot), f(1), pct(f(1), tot), f(0), pct(f(0), tot));
        }
    }
    println!("\n**Flip direction error (degrees) of the solver for true flip starts it also detected; baseline: always forward**\n");
    println!("| span | population | n | solver p50 | p90 | p99 | within 30 deg | forward baseline p50 | p90 |\n| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |");
    for (si, &d) in spans.iter().enumerate() {
        for pop in 0..3 {
            let mut v = Vec::new();
            let mut nv = Vec::new();
            for &g in games {
                for p in 0..2 {
                    if pop != 0 && pop - 1 != p {
                        continue;
                    }
                    let idx = (g * stats.spans + si) * 2 + p;
                    v.extend_from_slice(&stats.flip_angle[idx]);
                    nv.extend_from_slice(&stats.flip_angle_naive[idx]);
                }
            }
            if v.is_empty() {
                continue;
            }
            let within = v.iter().filter(|&&x| x <= 30.0).count();
            println!(
                "| {d} | {} | {} | {:.1} | {:.1} | {:.1} | {} | {:.1} | {:.1} |",
                POPS[pop],
                v.len(),
                quantile(&mut v.clone(), 0.5),
                quantile(&mut v.clone(), 0.9),
                quantile(&mut v, 0.99),
                pct(within as u64, v.len() as u64),
                quantile(&mut nv.clone(), 0.5),
                quantile(&mut nv, 0.9)
            );
        }
    }
}

// ---------------------------------------------------------------------------------------------

fn parse_list(s: &str) -> Vec<usize> {
    s.split(',').filter_map(|x| x.trim().parse().ok()).collect()
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut spans = vec![1usize, 4, 8];
    let mut strides: Vec<usize> = vec![1, 2, 4];
    let mut octane = false;
    let mut max_packets = usize::MAX;
    let mut per_game = false;
    let mut files: Vec<PathBuf> = Vec::new();
    let mut args = env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--spans" => spans = parse_list(&args.next().ok_or("--spans needs a list")?),
            "--strides" => strides = parse_list(&args.next().ok_or("--strides needs a list")?),
            "--bodies" => octane = args.next().ok_or("--bodies needs match|octane")? == "octane",
            "--max-packets" => max_packets = args.next().ok_or("--max-packets needs N")?.parse()?,
            "--per-game" => per_game = true,
            _ => files.push(PathBuf::from(a)),
        }
    }
    if files.is_empty() {
        return Err("usage: benchmark_rlcis [options] <states.jsonl>...".into());
    }
    if files.iter().any(|p| p.to_string_lossy().contains("test")) {
        return Err("refusing to inspect a path containing 'test'".into());
    }
    if strides.len() < spans.len() {
        strides.resize(spans.len(), 1);
    }
    rocketsim::init(Path::new("collision_meshes"), true)?;
    let mut scratch = Scratch::new();
    let mut stats = Stats::new(files.len(), spans.len());
    for (g, path) in files.iter().enumerate() {
        let packets = load(path, max_packets, octane)?;
        eprintln!("{}: {} distinct-frame packets", path.display(), packets.len());
        // The C++ ground check and the simulated turn accelerations only depend on the TO state.
        let mut infos: Vec<Vec<GroundInfo>> = Vec::with_capacity(packets.len());
        for p in &packets {
            infos.push(
                p.players
                    .iter()
                    .map(|pl| {
                        if p.phase == 3 && !pl.demolished {
                            scratch.ground_info(pl.body, &pl.s)
                        } else {
                            GroundInfo::default()
                        }
                    })
                    .collect(),
            );
        }
        eprintln!("  ground info done");
        for (si, &d) in spans.iter().enumerate() {
            evaluate(&packets, &infos, g, si, d, strides[si], &mut stats);
            eprintln!("  span {d} done");
        }
    }

    println!("# benchmark_rlcis: {} recordings, spans {:?}, strides {:?}, bodies {}", files.len(), spans, strides, if octane { "all Octane (the C++ default)" } else { "nearest preset by hitbox" });
    for (g, f) in files.iter().enumerate() {
        println!("- recording {g}: {}", f.display());
    }
    let all: Vec<usize> = (0..files.len()).collect();
    println!("\n#### Samples (car-spans) per class and population, pooled\n");
    println!("| span | class | bots | humans |\n| --- | --- | ---: | ---: |");
    for (si, &d) in spans.iter().enumerate() {
        for class in 0..4 {
            println!(
                "| {d} | {} | {} | {} |",
                CLASSES[class],
                stats.chan_sum(&all, si, Some(class), 1, 0, 0).n,
                stats.chan_sum(&all, si, Some(class), 2, 0, 0).n
            );
        }
    }
    print_channel_tables(&stats, &all, &spans, "Channels, pooled over both recordings", &[P_SOLVER, P_ZERO, P_PREV]);
    print_channel_tables(&stats, &all, &spans, "Solver variants (in-flip rule off; deadzones and clamps off; steer simulated at the FROM state)", &[P_SOLVER, P_NOFLIP, P_RAW, P_FROMSIM]);
    print_events(&stats, &all, &spans);
    print_cross(&stats, &all, &spans);
    print_sat(&stats, &all, &spans);
    if per_game {
        for g in 0..files.len() {
            print_channel_tables(&stats, &[g], &spans, &format!("Channels, recording {g} only"), &[P_SOLVER, P_ZERO, P_PREV]);
            print_events(&stats, &[g], &spans);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn some_rotation() -> Mat3A {
        // yaw 0.7, pitch 0.3, roll -0.4 in the RLBot convention
        matrix(&Euler { pitch: 0.3, yaw: 0.7, roll: -0.4 })
    }

    /// Forward model of the aerial control torque the inverse is derived from: local angular
    /// acceleration = T * input + D * omega (roll, pitch, yaw), with the pitch and yaw input torque
    /// terms switched by the sign of the damping product as in the formula.
    fn forward_air(rot: Mat3A, omega: Vec3A, input: Vec3A, dt: f32) -> Vec3A {
        let t = Vec3A::new(-36.0796, -12.1460, 8.9196);
        let d = Vec3A::new(-4.47166, -2.7982, -1.8865);
        let om = local(rot, omega);
        // Invert rhs[i] = input[i] * gain[i] with the sign-dependent gains: the sign of rhs equals the
        // sign of input times the sign of the gain, which is that of T (the correction is small).
        let gain_x = t.x;
        let sy = sgn(input.y * t.y);
        let gain_y = t.y + sy * om.y * d.y;
        let sz = sgn(input.z * t.z);
        let gain_z = t.z - sz * om.z * d.z;
        let rhs = Vec3A::new(input.x * gain_x, input.y * gain_y, input.z * gain_z);
        let tau_local = rhs + d * om;
        omega + rot * tau_local * dt
    }

    #[test]
    fn inverse_aerial_recovers_the_forward_model() {
        let rot = some_rotation();
        let omega = Vec3A::new(0.4, -0.7, 1.1);
        let dt = TICK_TIME;
        for input in [
            Vec3A::new(0.5, -0.3, 0.8),
            Vec3A::new(-1.0, 1.0, -1.0),
            Vec3A::new(0.0, 0.0, 0.0),
            Vec3A::new(0.2, 0.6, -0.9),
        ] {
            let after = forward_air(rot, omega, input, dt);
            let got = reverse_air_orient_inputs(omega, after, rot, dt);
            assert!((got - input).abs().max_element() < 1e-3, "input {input:?}, recovered {got:?}");
        }
    }

    #[test]
    fn deadzone_and_inverse_deadzone() {
        assert_eq!(deadzone(0.05, 0.1, 0.95), 0.0);
        assert_eq!(deadzone(-0.05, 0.1, 0.95), 0.0);
        assert_eq!(deadzone(0.5, 0.1, 0.95), 0.5);
        assert_eq!(deadzone(0.96, 0.1, 0.95), 1.0);
        assert_eq!(deadzone(-3.0, 0.1, 0.95), -1.0);
    }

    #[test]
    fn near_window_and_tick_count() {
        // IsNear(500, ..., 0.3, 2.8): the open window (350, 1900).
        assert!(is_near(1000.0, 500.0, 0.3, 2.8));
        assert!(!is_near(349.0, 500.0, 0.3, 2.8));
        assert!(!is_near(1900.0, 500.0, 0.3, 2.8));
        assert_eq!(tick_count(4.0 * TICK_TIME), 4);
        assert_eq!(tick_count(8.0 * TICK_TIME), 8);
        assert_eq!(sgn(0.0), 0.0);
    }

    #[test]
    fn air_solver_finds_a_free_fall_with_no_inputs() {
        let rot = Mat3A::IDENTITY;
        let from = SState { pos: Vec3A::new(0.0, 0.0, 1000.0), rot, vel: Vec3A::new(300.0, 0.0, 100.0), ang: Vec3A::ZERO };
        let to = SState { vel: from.vel + Vec3A::new(0.0, 0.0, GRAVITY_Z * TICK_TIME), pos: from.pos, rot, ang: Vec3A::ZERO };
        let r = solve_air(&from, &to, TICK_TIME, &SolverConfig::default());
        assert!(!r.flip_started && !r.double_jumping);
        assert!(!r.controls.boost && r.controls.throttle == 0.0);
        assert_eq!((r.controls.pitch, r.controls.yaw, r.controls.roll), (0.0, 0.0, 0.0));
    }

    #[test]
    fn air_solver_reads_a_boost_tick() {
        let rot = Mat3A::IDENTITY;
        let from = SState { pos: Vec3A::new(0.0, 0.0, 1000.0), rot, vel: Vec3A::new(300.0, 0.0, 100.0), ang: Vec3A::ZERO };
        let boost = car::boost::ACCEL_AIR * TICK_TIME;
        let to = SState { vel: from.vel + Vec3A::new(boost, 0.0, GRAVITY_Z * TICK_TIME), pos: from.pos, rot, ang: Vec3A::ZERO };
        let r = solve_air(&from, &to, TICK_TIME, &SolverConfig::default());
        assert!(r.controls.boost);
    }

    #[test]
    fn air_solver_reads_a_forward_flip_impulse() {
        let rot = Mat3A::IDENTITY;
        let from = SState { pos: Vec3A::new(0.0, 0.0, 1000.0), rot, vel: Vec3A::new(0.0, 0.0, 0.0), ang: Vec3A::ZERO };
        // Forward flip at standstill: 500 UU/s forward.
        let to = SState { vel: Vec3A::new(500.0, 0.0, GRAVITY_Z * TICK_TIME), pos: from.pos, rot, ang: Vec3A::ZERO };
        let r = solve_air(&from, &to, TICK_TIME, &SolverConfig::default());
        assert!(r.flip_started);
        assert!((r.controls.pitch + 1.0).abs() < 1e-3 && r.controls.yaw.abs() < 1e-3, "{:?}", r.controls);
    }
}
