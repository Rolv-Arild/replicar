//! Compare replay orientation changes with angular-velocity integration on fresh airborne pairs.

use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use glam::{Quat, Vec3};
use replay_to_rocketsim::observations;

fn paths(root: &Path) -> Result<Vec<PathBuf>, Box<dyn Error>> {
    if root.is_file() {
        return Ok(vec![root.to_owned()]);
    }
    let mut out = Vec::new();
    for size in ["1v1", "2v2", "3v3"] {
        for entry in fs::read_dir(root.join(size))? {
            let path = entry?.path();
            if path.extension().is_some_and(|ext| ext == "replay") {
                out.push(path);
            }
        }
    }
    out.sort();
    Ok(out)
}

fn quaternion(raw: [f32; 4]) -> Option<Quat> {
    let q = Quat::from_xyzw(raw[0], raw[1], raw[2], raw[3]);
    (q.is_finite() && q.length_squared() > 0.5).then(|| q.normalize())
}

fn angle_degrees(a: Quat, b: Quat) -> f32 {
    2.0 * a.dot(b).abs().clamp(0.0, 1.0).acos().to_degrees()
}

fn projected_translation_scale(
    p0: [f32; 3],
    p1: [f32; 3],
    v0: [f32; 3],
    v1: [f32; 3],
    dt: f32,
) -> Option<f32> {
    let mean_vel = (Vec3::from_array(v0) + Vec3::from_array(v1)) * 0.5;
    if mean_vel.length_squared() <= 10000.0 {
        return None;
    }
    let delta = Vec3::from_array(p1) - Vec3::from_array(p0);
    let scale = delta.dot(mean_vel) / (dt * mean_vel.length_squared());
    scale.is_finite().then_some(scale)
}

fn summary(label: &str, mut samples: Vec<f32>) {
    samples.sort_by(f32::total_cmp);
    if samples.is_empty() {
        println!("{label}: no samples");
        return;
    }
    println!(
        "{label}: n={} p50={:.3} p90={:.3} mean={:.3}",
        samples.len(),
        samples[samples.len() / 2],
        samples[(samples.len() as f64 * 0.9) as usize],
        samples.iter().map(|&v| v as f64).sum::<f64>() / samples.len() as f64,
    );
}

fn main() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(
        env::args_os()
            .nth(1)
            .ok_or("usage: diagnose_air_rotation <split>")?,
    );
    let replay_paths = paths(&root)?;
    let mut hold = Vec::new();
    let mut midpoint = Vec::new();
    let mut endpoint = Vec::new();
    let mut midpoint_half = Vec::new();
    let mut midpoint_three_quarters = Vec::new();
    let mut midpoint_five_quarters = Vec::new();
    let mut projected_scale = Vec::new();
    let mut scale_by_gap: [Vec<f32>; 7] = std::array::from_fn(|_| Vec::new());
    let mut translation_scale_by_gap: [Vec<f32>; 7] = std::array::from_fn(|_| Vec::new());
    let mut adaptive_rotation = Vec::new();
    let mut full_rotation_on_adaptive_pairs = Vec::new();
    let mut angular_translation_scale_difference = Vec::new();
    let mut same_frame_car_scale_difference = Vec::new();
    let mut same_frame_ball_car_scale_difference = Vec::new();
    let mut ball_scale_by_gap: [Vec<f32>; 7] = std::array::from_fn(|_| Vec::new());
    let mut observed_mean_vs_start = Vec::new();
    let mut observed_mean_vs_midpoint = Vec::new();

    for path in &replay_paths {
        let rotation_start = scale_by_gap[4].len();
        let translation_start = translation_scale_by_gap[4].len();
        let bytes = fs::read(path)?;
        let parsed = replay_to_rocketsim::parse_replay(&bytes)?;
        let replay = observations::extract(&parsed).ok_or("network observations unavailable")?;
        let mut clock_gaps = Vec::new();
        let mut last_clock = None;
        for frame in &replay.frames {
            if !frame
                .game_state
                .as_ref()
                .is_some_and(|s| s.value == "Active")
            {
                last_clock = None;
                continue;
            }
            if let Some(clock) = frame
                .seconds_remaining
                .as_ref()
                .filter(|v| v.frame == frame.index)
            {
                if let Some((prior, prior_time)) = last_clock {
                    let gap = frame.time - prior_time;
                    if prior == clock.value + 1 && (0.5..=3.0).contains(&gap) {
                        clock_gaps.push(gap);
                    }
                }
                last_clock = Some((clock.value, frame.time));
            }
        }
        clock_gaps.sort_by(f32::total_cmp);
        for pair in replay.frames.windows(2) {
            let [f0, f1] = pair else { unreachable!() };
            if !f0.game_state.as_ref().is_some_and(|s| s.value == "Active")
                || !f1.game_state.as_ref().is_some_and(|s| s.value == "Active")
            {
                continue;
            }
            let dt = f1.time - f0.time;
            if !dt.is_finite() || dt <= 0.0 || dt > 0.05 {
                continue;
            }
            let mut frame_scales = Vec::new();
            for prior in observations::primary_linked_cars(f0) {
                let Some(next) = observations::primary_linked_cars(f1).into_iter().find(|c| {
                    c.actor_id == prior.actor_id
                        && c.actor_created_frame == prior.actor_created_frame
                        && c.player_key == prior.player_key
                }) else {
                    continue;
                };
                let (Some(p0), Some(p1), Some(v0), Some(v1)) = (
                    &prior.body.position,
                    &next.body.position,
                    &prior.body.linear_velocity,
                    &next.body.linear_velocity,
                ) else {
                    continue;
                };
                if p0.frame == f0.index
                    && p1.frame == f1.index
                    && v0.frame == f0.index
                    && v1.frame == f1.index
                {
                    if let Some(scale) =
                        projected_translation_scale(p0.value, p1.value, v0.value, v1.value, dt)
                    {
                        frame_scales.push(scale);
                    }
                }
            }
            for i in 0..frame_scales.len() {
                for j in i + 1..frame_scales.len() {
                    same_frame_car_scale_difference.push((frame_scales[i] - frame_scales[j]).abs());
                }
            }
            if let (Some(b0), Some(b1)) = (&f0.ball, &f1.ball) {
                if let (Some(p0), Some(p1), Some(v0), Some(v1)) = (
                    &b0.position,
                    &b1.position,
                    &b0.linear_velocity,
                    &b1.linear_velocity,
                ) {
                    if p0.frame == f0.index
                        && p1.frame == f1.index
                        && v0.frame == f0.index
                        && v1.frame == f1.index
                    {
                        if let Some(ball_scale) =
                            projected_translation_scale(p0.value, p1.value, v0.value, v1.value, dt)
                        {
                            let ticks = (dt * 120.0).round() as usize;
                            if ticks < ball_scale_by_gap.len() {
                                ball_scale_by_gap[ticks].push(ball_scale);
                            }
                            for &car_scale in &frame_scales {
                                same_frame_ball_car_scale_difference
                                    .push((ball_scale - car_scale).abs());
                            }
                        }
                    }
                }
            }
            for c0 in observations::primary_linked_cars(f0) {
                let Some(c1) = observations::primary_linked_cars(f1).into_iter().find(|c| {
                    c.actor_id == c0.actor_id
                        && c.actor_created_frame == c0.actor_created_frame
                        && c.player_key == c0.player_key
                        && c.player_link_active == c0.player_link_active
                }) else {
                    continue;
                };
                let (Some(p0), Some(p1), Some(r0), Some(r1), Some(w0), Some(w1)) = (
                    &c0.body.position,
                    &c1.body.position,
                    &c0.body.rotation_xyzw,
                    &c1.body.rotation_xyzw,
                    &c0.body.angular_velocity_replay_units,
                    &c1.body.angular_velocity_replay_units,
                ) else {
                    continue;
                };
                if p0.frame != f0.index
                    || p1.frame != f1.index
                    || r0.frame != f0.index
                    || r1.frame != f1.index
                    || w0.frame != f0.index
                    || w1.frame != f1.index
                    || p0.value[2] <= 100.0
                    || p1.value[2] <= 100.0
                    || c0
                        .inputs
                        .dodge_active_raw
                        .as_ref()
                        .is_some_and(|d| d.frame == f0.index && d.value % 2 == 1)
                    || c1
                        .inputs
                        .dodge_active_raw
                        .as_ref()
                        .is_some_and(|d| d.frame == f1.index && d.value % 2 == 1)
                {
                    continue;
                }
                let (Some(q0), Some(q1)) = (quaternion(r0.value), quaternion(r1.value)) else {
                    continue;
                };
                let omega0 = Vec3::from_array(w0.value) * 0.01;
                let omega1 = Vec3::from_array(w1.value) * 0.01;
                if !omega0.is_finite() || !omega1.is_finite() {
                    continue;
                }
                let omega_mid = (omega0 + omega1) * 0.5;
                let ticks = (dt * 120.0).round() as usize;
                let mut translation_scale = None;
                if let (Some(v0), Some(v1)) = (&c0.body.linear_velocity, &c1.body.linear_velocity) {
                    if v0.frame == f0.index
                        && v1.frame == f1.index
                        && ticks < translation_scale_by_gap.len()
                    {
                        if let Some(scale) =
                            projected_translation_scale(p0.value, p1.value, v0.value, v1.value, dt)
                        {
                            translation_scale_by_gap[ticks].push(scale);
                            translation_scale = Some(scale);
                        }
                    }
                }
                hold.push(angle_degrees(Quat::from_scaled_axis(omega0 * dt) * q0, q1));
                midpoint.push(angle_degrees(
                    Quat::from_scaled_axis(omega_mid * dt) * q0,
                    q1,
                ));
                if let Some(scale) = translation_scale.filter(|s| (0.25..=2.5).contains(s)) {
                    adaptive_rotation.push(angle_degrees(
                        Quat::from_scaled_axis(omega_mid * dt * scale) * q0,
                        q1,
                    ));
                    full_rotation_on_adaptive_pairs.push(angle_degrees(
                        Quat::from_scaled_axis(omega_mid * dt) * q0,
                        q1,
                    ));
                }
                midpoint_half.push(angle_degrees(
                    Quat::from_scaled_axis(omega_mid * dt * 0.5) * q0,
                    q1,
                ));
                midpoint_three_quarters.push(angle_degrees(
                    Quat::from_scaled_axis(omega_mid * dt * 0.75) * q0,
                    q1,
                ));
                midpoint_five_quarters.push(angle_degrees(
                    Quat::from_scaled_axis(omega_mid * dt * 1.25) * q0,
                    q1,
                ));
                endpoint.push(angle_degrees(Quat::from_scaled_axis(omega1 * dt) * q0, q1));
                let mut delta = q1 * q0.conjugate();
                if delta.w < 0.0 {
                    delta = -delta;
                }
                let omega_rot = delta.to_scaled_axis() / dt;
                if omega_mid.length_squared() > 0.25 {
                    let scale = omega_rot.dot(omega_mid) / omega_mid.length_squared();
                    projected_scale.push(scale);
                    if let Some(translation_scale) =
                        translation_scale.filter(|s| (0.25..=2.5).contains(s))
                    {
                        angular_translation_scale_difference
                            .push((scale - translation_scale).abs());
                    }
                    if ticks < scale_by_gap.len() {
                        scale_by_gap[ticks].push(scale);
                    }
                }
                observed_mean_vs_start.push((omega_rot - omega0).length());
                observed_mean_vs_midpoint.push((omega_rot - omega_mid).length());
            }
        }
        let mut rotation_new = scale_by_gap[4][rotation_start..].to_vec();
        let mut translation_new = translation_scale_by_gap[4][translation_start..].to_vec();
        if rotation_new.len() >= 30 && translation_new.len() >= 30 {
            rotation_new.sort_by(f32::total_cmp);
            translation_new.sort_by(f32::total_cmp);
            println!(
                "replay {} n={} angular_scale={:.3} translation_scale={:.3} clock_gap={:?} record_fps={:?} match_type={:?} game_version={:?}",
                path.file_name().unwrap().to_string_lossy(),
                rotation_new.len(),
                rotation_new[rotation_new.len() / 2],
                translation_new[translation_new.len() / 2],
                clock_gaps.get(clock_gaps.len() / 2),
                parsed
                    .properties
                    .iter()
                    .find(|(key, _)| key == "RecordFPS")
                    .map(|(_, value)| value),
                parsed
                    .properties
                    .iter()
                    .find(|(key, _)| key == "MatchType")
                    .map(|(_, value)| value),
                parsed
                    .properties
                    .iter()
                    .find(|(key, _)| key == "GameVersion")
                    .map(|(_, value)| value),
            );
        }
    }
    println!("{} replays", replay_paths.len());
    summary("rotation from start angular velocity (deg)", hold);
    summary("rotation from midpoint angular velocity (deg)", midpoint);
    summary(
        "rotation from half midpoint angular velocity (deg)",
        midpoint_half,
    );
    summary(
        "rotation from 0.75 midpoint angular velocity (deg)",
        midpoint_three_quarters,
    );
    summary(
        "rotation from 1.25 midpoint angular velocity (deg)",
        midpoint_five_quarters,
    );
    summary("rotation from end angular velocity (deg)", endpoint);
    summary(
        "rotation-derived angular velocity vs start (rad/s)",
        observed_mean_vs_start,
    );
    summary(
        "rotation-derived angular velocity vs midpoint (rad/s)",
        observed_mean_vs_midpoint,
    );
    summary(
        "projected orientation-to-angular-velocity scale",
        projected_scale,
    );
    summary(
        "full rotation on adaptive subset (deg)",
        full_rotation_on_adaptive_pairs,
    );
    summary(
        "translation-scaled rotation on same subset (deg)",
        adaptive_rotation,
    );
    summary(
        "absolute angular vs translation scale difference",
        angular_translation_scale_difference,
    );
    summary(
        "same-frame car-car scale difference",
        same_frame_car_scale_difference,
    );
    summary(
        "same-frame ball-car scale difference",
        same_frame_ball_car_scale_difference,
    );
    for (ticks, samples) in scale_by_gap.into_iter().enumerate().skip(1) {
        summary(&format!("projected scale at {ticks} ticks"), samples);
    }
    for (ticks, samples) in translation_scale_by_gap.into_iter().enumerate().skip(1) {
        summary(&format!("translation scale at {ticks} ticks"), samples);
    }
    for (ticks, samples) in ball_scale_by_gap.into_iter().enumerate().skip(1) {
        summary(&format!("ball scale at {ticks} ticks"), samples);
    }
    Ok(())
}
