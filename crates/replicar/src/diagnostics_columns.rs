//! The `diagnostics` group (docs/glossary.md, "Column group"): what the reconstruction measured about itself.
//! Per frame, the prediction error of every update before it corrected the simulation (v1's position
//! residuals, without the offline interval estimate), RocketSim's own events, and the simulated touches.

use glam::{Mat3A, Vec3A};
use replicar_format::{
    Columns, RecordBatch, child_bool, child_f32, child_i32, child_str, child_u8, child_u16,
    child_u64,
};
use rocketsim::ArenaEvent;

use crate::annotate::SimulatedTouch;
use crate::decode::{NetworkBody, NetworkFrame};
use crate::simulate::{Prediction, SimEvent};
use crate::update_ticks::quaternion;

const GROUP: &str = "diagnostics";

/// The error of a prediction against the update that corrected it.
#[derive(Debug, Clone, Copy)]
pub(crate) struct PredictionError {
    pub car: Option<i32>,
    /// Replay time since the body's previous position update.
    pub seconds_since_previous: f32,
    /// Predicted minus updated position.
    pub position_error: [f32; 3],
    pub velocity_error: Option<f32>,
    pub rotation_error_degrees: Option<f32>,
    pub angular_velocity_error: Option<f32>,
    /// The same errors for holding the previous update (a baseline), and for extrapolating it linearly.
    pub hold_position_error: f32,
    pub linear_position_error: Option<f32>,
    pub hold_velocity_error: Option<f32>,
    pub hold_rotation_error_degrees: Option<f32>,
    pub hold_angular_velocity_error: Option<f32>,
    pub is_on_ground: Option<bool>,
}

/// One frame's diagnostics.
#[derive(Debug, Clone, Default)]
pub(crate) struct DiagnosticsRow {
    pub errors: Vec<PredictionError>,
    pub events: Vec<SimEvent>,
    pub touches: Vec<SimulatedTouch>,
}

fn rotation_error_degrees(a: Mat3A, b: Mat3A) -> f32 {
    let trace = a.x_axis.dot(b.x_axis) + a.y_axis.dot(b.y_axis) + a.z_axis.dot(b.z_axis);
    ((trace - 1.0) * 0.5).clamp(-1.0, 1.0).acos().to_degrees()
}

/// The error of `prediction` against `body`'s update at frame `index`, with the body's state in the previous
/// frame as the baseline (v1's `position_residual`). `None` when the frame has no position update or the
/// previous one is missing or more than 0.5 s old.
pub(crate) fn prediction_error(
    frames: &[NetworkFrame],
    index: usize,
    prediction: &Prediction,
    body: &NetworkBody,
    previous: Option<&NetworkBody>,
) -> Option<PredictionError> {
    let at = |f: replicar_format::FrameIndex| f.get() == index;
    let actual = body.position.as_ref().filter(|v| at(v.frame))?;
    let previous_position = previous?.position.as_ref()?;
    if previous_position.frame.get() >= index {
        return None;
    }
    let dt = frames[index].time - frames[previous_position.frame.get()].time;
    if !dt.is_finite() || dt <= 0.0 || dt > 0.5 {
        return None;
    }
    let recent = |frame: replicar_format::FrameIndex| {
        frame.get() < index && {
            let gap = frames[index].time - frames[frame.get()].time;
            gap.is_finite() && gap > 0.0 && gap <= 0.5
        }
    };
    let predicted = &prediction.phys;
    let actual_position = Vec3A::from(actual.value);
    let previous_value = Vec3A::from(previous_position.value);
    let linear_position_error = previous
        .and_then(|b| b.linear_velocity.as_ref())
        .filter(|v| recent(v.frame))
        .map(|v| (previous_value + Vec3A::from(v.value) * dt - actual_position).length());
    let pair = |now: Option<Vec3A>, before: Option<(replicar_format::FrameIndex, Vec3A)>| match (
        now, before,
    ) {
        (Some(now), Some((frame, before))) if recent(frame) => Some((now, before)),
        _ => None,
    };
    let velocity = pair(
        body.linear_velocity
            .as_ref()
            .filter(|v| at(v.frame))
            .map(|v| Vec3A::from(v.value)),
        previous
            .and_then(|b| b.linear_velocity.as_ref())
            .map(|v| (v.frame, Vec3A::from(v.value))),
    );
    let angular = pair(
        body.angular_velocity_raw
            .as_ref()
            .filter(|v| at(v.frame))
            .map(|v| Vec3A::from(v.value) * 0.01),
        previous
            .and_then(|b| b.angular_velocity_raw.as_ref())
            .map(|v| (v.frame, Vec3A::from(v.value) * 0.01)),
    );
    let rotation = match (
        body.rotation.as_ref().filter(|v| at(v.frame)),
        previous.and_then(|b| b.rotation.as_ref()),
    ) {
        (Some(now), Some(before)) if recent(before.frame) => {
            match (quaternion(now.value), quaternion(before.value)) {
                (Some(now), Some(before)) => {
                    Some((Mat3A::from_quat(now), Mat3A::from_quat(before)))
                }
                _ => None,
            }
        }
        _ => None,
    };
    Some(PredictionError {
        car: prediction.car.map(|c| c.0),
        seconds_since_previous: dt,
        position_error: (predicted.pos - actual_position).to_array(),
        velocity_error: velocity.map(|(now, _)| (predicted.vel - now).length()),
        rotation_error_degrees: rotation
            .map(|(now, _)| rotation_error_degrees(predicted.rot_mat, now)),
        angular_velocity_error: angular.map(|(now, _)| (predicted.ang_vel - now).length()),
        hold_position_error: (previous_value - actual_position).length(),
        linear_position_error,
        hold_velocity_error: velocity.map(|(now, before)| (before - now).length()),
        hold_rotation_error_degrees: rotation
            .map(|(now, before)| rotation_error_degrees(before, now)),
        hold_angular_velocity_error: angular.map(|(now, before)| (before - now).length()),
        is_on_ground: prediction.is_on_ground,
    })
}

/// The prediction errors of a frame's predictions; the previous frame's bodies are the baseline (for a car,
/// the previous frame's car with the same actor id).
pub(crate) fn frame_errors(
    frames: &[NetworkFrame],
    index: usize,
    predictions: &[Prediction],
) -> Vec<PredictionError> {
    let frame = &frames[index];
    let previous = index.checked_sub(1).map(|i| &frames[i]);
    predictions
        .iter()
        .filter_map(|prediction| match prediction.car {
            None => prediction_error(
                frames,
                index,
                prediction,
                frame.ball.as_ref()?,
                previous.and_then(|p| p.ball.as_ref()),
            ),
            Some(actor) => {
                let car = frame.cars.iter().find(|c| c.life.actor == actor)?;
                let before = previous
                    .and_then(|p| p.cars.iter().find(|c| c.life.actor == actor))
                    .map(|c| &c.body);
                prediction_error(frames, index, prediction, &car.body, before)
            }
        })
        .collect()
}

fn xyz(v: Vec3A) -> [Option<f32>; 3] {
    v.to_array().map(Some)
}

/// The group's columns, one row per frame.
pub(crate) fn diagnostics_columns(
    rows: &[DiagnosticsRow],
) -> Result<RecordBatch, replicar_format::WriteError> {
    let mut columns = Columns::default();
    let errors: Vec<&PredictionError> = rows.iter().flat_map(|r| &r.errors).collect();
    let lengths: Vec<usize> = rows.iter().map(|r| r.errors.len()).collect();
    let floats = |f: fn(&PredictionError) -> Option<f32>| errors.iter().map(|e| f(e)).collect();
    columns.records(
        "prediction_errors",
        GROUP,
        &lengths,
        vec![
            child_i32("car_actor", errors.iter().map(|e| e.car).collect()),
            child_f32(
                "seconds_since_previous",
                floats(|e| Some(e.seconds_since_previous)),
            ),
            child_f32("position_error_x", floats(|e| Some(e.position_error[0]))),
            child_f32("position_error_y", floats(|e| Some(e.position_error[1]))),
            child_f32("position_error_z", floats(|e| Some(e.position_error[2]))),
            child_f32("velocity_error", floats(|e| e.velocity_error)),
            child_f32(
                "rotation_error_degrees",
                floats(|e| e.rotation_error_degrees),
            ),
            child_f32(
                "angular_velocity_error",
                floats(|e| e.angular_velocity_error),
            ),
            child_f32(
                "hold_position_error",
                floats(|e| Some(e.hold_position_error)),
            ),
            child_f32("linear_position_error", floats(|e| e.linear_position_error)),
            child_f32("hold_velocity_error", floats(|e| e.hold_velocity_error)),
            child_f32(
                "hold_rotation_error_degrees",
                floats(|e| e.hold_rotation_error_degrees),
            ),
            child_f32(
                "hold_angular_velocity_error",
                floats(|e| e.hold_angular_velocity_error),
            ),
            child_bool(
                "is_on_ground",
                errors.iter().map(|e| e.is_on_ground).collect(),
            ),
        ],
    );

    let events: Vec<&SimEvent> = rows.iter().flat_map(|r| &r.events).collect();
    let lengths: Vec<usize> = rows.iter().map(|r| r.events.len()).collect();
    // (kind, player, other player, pad, is_demo, point, normal, extra velocity)
    type Fields = (
        &'static str,
        Option<u8>,
        Option<u8>,
        Option<u16>,
        Option<bool>,
        [Option<f32>; 3],
        [Option<f32>; 3],
        [Option<f32>; 3],
    );
    let player = |i: usize| u8::try_from(i).ok();
    let none = [None; 3];
    let fields: Vec<Fields> = events
        .iter()
        .map(|e| match &e.event {
            ArenaEvent::BallHitWorld(h) => (
                "ball_hit_world",
                None,
                None,
                None,
                None,
                xyz(h.contact_point),
                xyz(h.contact_normal),
                none,
            ),
            ArenaEvent::CarHitBall(h) => (
                "car_hit_ball",
                player(h.car_idx),
                None,
                None,
                None,
                xyz(h.contact_point),
                none,
                xyz(h.extra_hit_vel),
            ),
            ArenaEvent::CarHitCar(h) => (
                "car_hit_car",
                player(h.bumper_car_idx),
                player(h.victim_car_idx),
                None,
                Some(h.is_demo),
                xyz(h.contact_point),
                none,
                none,
            ),
            ArenaEvent::CarHitWorld(h) => (
                "car_hit_world",
                player(h.car_idx),
                None,
                None,
                None,
                xyz(h.contact_point),
                xyz(h.contact_normal),
                none,
            ),
            ArenaEvent::CarPickupBoost(h) => (
                "car_pickup_boost",
                player(h.car_idx),
                None,
                u16::try_from(h.boost_pad_idx).ok(),
                None,
                none,
                none,
                none,
            ),
            ArenaEvent::CarLanded(h) => (
                "car_landed",
                player(h.car_idx),
                None,
                None,
                None,
                none,
                none,
                none,
            ),
        })
        .collect();
    let mut children = vec![
        child_u64(
            "sim_tick",
            events.iter().map(|e| Some(e.sim_tick)).collect(),
        ),
        child_str("kind", fields.iter().map(|f| Some(f.0)).collect()),
        child_u8("player", fields.iter().map(|f| f.1).collect()),
        child_u8("other_player", fields.iter().map(|f| f.2).collect()),
        child_u16("pad", fields.iter().map(|f| f.3).collect()),
        child_bool("is_demo", fields.iter().map(|f| f.4).collect()),
    ];
    for (name, get) in [
        (
            "point",
            (|f: &Fields| f.5) as fn(&Fields) -> [Option<f32>; 3],
        ),
        ("normal", |f: &Fields| f.6),
        ("extra_velocity", |f: &Fields| f.7),
    ] {
        for (i, axis) in ["x", "y", "z"].iter().enumerate() {
            children.push(child_f32(
                &format!("{name}_{axis}"),
                fields.iter().map(|f| get(f)[i]).collect(),
            ));
        }
    }
    columns.records("simulated_events", GROUP, &lengths, children);

    let touches: Vec<&SimulatedTouch> = rows.iter().flat_map(|r| &r.touches).collect();
    let lengths: Vec<usize> = rows.iter().map(|r| r.touches.len()).collect();
    let mut children = vec![
        child_u8("player", touches.iter().map(|t| Some(t.player.0)).collect()),
        child_u64(
            "replay_tick",
            touches.iter().map(|t| Some(t.replay_tick)).collect(),
        ),
    ];
    for (i, axis) in ["x", "y", "z"].iter().enumerate() {
        children.push(child_f32(
            &format!("point_{axis}"),
            touches.iter().map(|t| Some(t.contact_point[i])).collect(),
        ));
    }
    columns.records("simulated_touches", GROUP, &lengths, children);
    Ok(columns.finish()?)
}
