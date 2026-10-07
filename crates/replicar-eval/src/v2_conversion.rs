//! v2's conversion in v1's output types (`replicar_v1::conversion::ConversionOutput`), so that the evaluator's
//! metric code runs on v2 (docs/v2-plan.md, story 9.1). v1's options are mapped to v2's configuration as in the
//! parity rungs; v1's position residuals are computed from v2's predictions with v1's rule.

use std::error::Error;

use replicar::annotate::scoreboard::{Decider, reconstruct};
use replicar::annotate::{Annotator, ball_intervals};
use replicar::decode::{NetworkBody, NetworkReplay};
use replicar::infer::{FittedInference, InferenceOptions};
use replicar::rocketsim::{Mat3A, Vec3A};
use replicar::simulate::{
    FittedKind, HoldSource, Prediction, SimulatedFrame, SimulationOptions, TickSource, simulate,
};
use replicar::update_ticks::{Withheld, quaternion};
use replicar_v1::conversion::{
    AppliedPacketLag, BallContact, BoostPickup, CarSlot, ConversionOutput, ConvertOptions,
    ConvertedFrame, DeadShellHold, Diagnostics, FittedInput, PositionResidual, SimEvent,
    TouchEvent, estimate_car_packet_interval, rotation_error_degrees,
};

/// v1's position residual of `prediction` against `body`'s update at frame `index` (v1's
/// `conversion::position_residual`, private there, with the network feed's types).
fn residual(
    network: &NetworkReplay,
    index: usize,
    prediction: &Prediction,
    body: &NetworkBody,
    previous: Option<&NetworkBody>,
) -> Option<PositionResidual> {
    let frames = &network.frames;
    let at = |f: replicar::FrameIndex| f.get() == index;
    let actual = body.position.as_ref().filter(|v| at(v.frame))?;
    let previous_pos = previous?.position.as_ref()?;
    if previous_pos.frame.get() >= index {
        return None;
    }
    let dt = frames[index].time - frames[previous_pos.frame.get()].time;
    if !dt.is_finite() || dt <= 0.0 || dt > 0.5 {
        return None;
    }
    let recent = |previous_frame: replicar::FrameIndex| {
        previous_frame.get() < index && {
            let gap = frames[index].time - frames[previous_frame.get()].time;
            gap.is_finite() && gap > 0.0 && gap <= 0.5
        }
    };
    let predicted = &prediction.phys;
    let actual_pos = Vec3A::from(actual.value);
    let previous_pos_val = Vec3A::from(previous_pos.value);
    let linear_extrapolation_error_uu = previous
        .and_then(|b| b.linear_velocity.as_ref())
        .filter(|v| recent(v.frame))
        .map(|v| (previous_pos_val + Vec3A::from(v.value) * dt - actual_pos).length());
    let (simulated_velocity_error_uu_per_sec, hold_velocity_error_uu_per_sec) = match (
        body.linear_velocity.as_ref().filter(|v| at(v.frame)),
        previous.and_then(|b| b.linear_velocity.as_ref()),
    ) {
        (Some(now), Some(before)) if recent(before.frame) => {
            let now = Vec3A::from(now.value);
            (
                Some((predicted.vel - now).length()),
                Some((Vec3A::from(before.value) - now).length()),
            )
        }
        _ => (None, None),
    };
    let (simulated_rotation_error_degrees, hold_rotation_error_degrees) = match (
        body.rotation.as_ref().filter(|v| at(v.frame)),
        previous.and_then(|b| b.rotation.as_ref()),
    ) {
        (Some(now), Some(before)) if recent(before.frame) => {
            match (quaternion(now.value), quaternion(before.value)) {
                (Some(now), Some(before)) => {
                    let (now, before) = (Mat3A::from_quat(now), Mat3A::from_quat(before));
                    (
                        Some(rotation_error_degrees(predicted.rot_mat, now)),
                        Some(rotation_error_degrees(before, now)),
                    )
                }
                _ => (None, None),
            }
        }
        _ => (None, None),
    };
    let (simulated_angular_velocity_error_rad_per_sec, hold_angular_velocity_error_rad_per_sec) =
        match (
            body.angular_velocity_raw.as_ref().filter(|v| at(v.frame)),
            previous.and_then(|b| b.angular_velocity_raw.as_ref()),
        ) {
            (Some(now), Some(before)) if recent(before.frame) => {
                let now = Vec3A::from(now.value) * 0.01;
                let before = Vec3A::from(before.value) * 0.01;
                (
                    Some((predicted.ang_vel - now).length()),
                    Some((before - now).length()),
                )
            }
            _ => (None, None),
        };
    let (offline_interval, offline_projection_fit_error_uu) = match (
        prediction.car,
        body.linear_velocity.as_ref().filter(|v| at(v.frame)),
        previous.and_then(|b| b.linear_velocity.as_ref()),
    ) {
        (Some(_), Some(now), Some(before)) if before.frame == previous_pos.frame => {
            let estimate = estimate_car_packet_interval(
                previous_pos.value,
                actual.value,
                before.value,
                now.value,
                dt,
            );
            let error = estimate.map(|e| {
                (previous_pos_val + Vec3A::from(before.value) * e.effective_seconds - actual_pos)
                    .length()
            });
            (estimate, error)
        }
        _ => (None, None),
    };
    Some(PositionResidual {
        frame: index,
        actor_id: prediction.car.map(|c| c.0),
        seconds_since_previous_position: dt,
        simulated_error_uu: (predicted.pos - actual_pos).length(),
        simulated_error_vector_uu: (predicted.pos - actual_pos).to_array(),
        previous_linear_velocity_uu_per_second: previous
            .and_then(|b| b.linear_velocity.as_ref())
            .filter(|v| recent(v.frame))
            .map(|v| v.value),
        hold_error_uu: (previous_pos_val - actual_pos).length(),
        linear_extrapolation_error_uu,
        simulated_velocity_error_uu_per_sec,
        hold_velocity_error_uu_per_sec,
        simulated_rotation_error_degrees,
        hold_rotation_error_degrees,
        simulated_angular_velocity_error_rad_per_sec,
        hold_angular_velocity_error_rad_per_sec,
        altitude_z: Some(actual.value[2]),
        is_on_ground: prediction.is_on_ground,
        offline_interval,
        offline_projection_fit_error_uu,
    })
}

/// The frame's residuals in v1's order (the frame's predictions, ball first as the simulator makes them).
fn residuals(network: &NetworkReplay, frame: &SimulatedFrame) -> Vec<PositionResidual> {
    let index = frame.index.get();
    let this = &network.frames[index];
    let previous = index.checked_sub(1).map(|i| &network.frames[i]);
    frame
        .predictions
        .iter()
        .filter_map(|prediction| match prediction.car {
            None => residual(
                network,
                index,
                prediction,
                this.ball.as_ref()?,
                previous.and_then(|p| p.ball.as_ref()),
            ),
            Some(actor) => {
                let car = this.cars.iter().find(|c| c.life.actor == actor)?;
                let before = previous
                    .and_then(|p| p.cars.iter().find(|c| c.life.actor == actor))
                    .map(|c| &c.body);
                residual(network, index, prediction, &car.body, before)
            }
        })
        .collect()
}

fn source(source: TickSource) -> &'static str {
    match source {
        TickSource::Chain => "chain",
        TickSource::FrameMedian => "frame_median",
        TickSource::Default => "default",
        TickSource::DodgeFit => "dodge_fit",
    }
}

/// v1's frame from v2's simulated frame, its annotations and its scoreboard.
fn converted_frame(
    network: &NetworkReplay,
    frame: SimulatedFrame,
    annotations: replicar::annotate::Annotations,
    scoreboard: replicar::annotate::scoreboard::Scoreboard,
) -> ConvertedFrame {
    let index = frame.index.get();
    ConvertedFrame {
        replay_frame: index,
        replay_time: network.frames[index].time,
        timeline_tick: frame.replay_tick,
        simulated_events: frame
            .events
            .iter()
            .map(|e| SimEvent {
                arena_tick: e.sim_tick,
                event: e.event,
            })
            .collect(),
        touches: annotations
            .simulated_touches
            .iter()
            .map(|t| TouchEvent {
                car_slot: usize::from(t.player.0),
                tick: t.replay_tick,
                contact_point: t.contact_point,
            })
            .collect(),
        ball_contacts: annotations
            .ball_contacts
            .iter()
            .map(|c| BallContact {
                frame_a: c.frame_a,
                tick: c.replay_tick,
                tick_from: c.tick_from,
                tick_to: c.tick_to,
                car_slot: c.player.map(|p| usize::from(p.0)),
                gap_uu: c.gap,
                velocity_residual: c.velocity_residual,
                simulated_touch: c.simulated_touch,
            })
            .collect(),
        boost_pickups: annotations
            .boost_pickups
            .iter()
            .map(|b| BoostPickup {
                pad_index: b.pad_index,
                pad_actor_id: b.pad.0,
                is_big: b.is_big,
                car_slot: b.player.map(|p| usize::from(p.0)),
                verified: b.verified,
                distance_uu: b.distance,
                suggested_car_slot: b.suggested_player.map(|p| usize::from(p.0)),
                tick: b.replay_tick,
            })
            .collect(),
        scoreboard: Some(replicar_v1::scoreboard::ScoreboardFrame {
            period: scoreboard.period.name(),
            clock_state: scoreboard.clock_phase.name(),
            seconds_remaining: scoreboard.seconds_remaining,
            overtime_seconds: scoreboard.overtime_seconds,
        }),
        packet_lags: frame
            .applied_ticks
            .iter()
            .map(|t| AppliedPacketLag {
                actor_id: t.car.map(|a| a.0),
                ticks: t.ticks,
                source: source(t.source),
            })
            .collect(),
        fitted_inputs: frame
            .fitted
            .iter()
            .map(|f| {
                let slot = usize::from(f.player.0);
                match f.kind {
                    FittedKind::Air { span_ticks } => FittedInput {
                        slot,
                        activation_frame: 0,
                        kind: "air",
                        tick: f.replay_tick,
                        pitch: 0.0,
                        yaw: 0.0,
                        cancel: 0.0,
                        span_ticks: Some(span_ticks),
                    },
                    FittedKind::Jump => FittedInput {
                        slot,
                        activation_frame: 0,
                        kind: "jump",
                        tick: f.replay_tick,
                        pitch: 0.0,
                        yaw: 0.0,
                        cancel: 0.0,
                        span_ticks: None,
                    },
                    FittedKind::Dodge {
                        pitch,
                        yaw,
                        cancel,
                        activation_frame,
                    } => FittedInput {
                        slot,
                        activation_frame,
                        kind: "dodge",
                        tick: f.replay_tick,
                        pitch,
                        yaw,
                        cancel,
                        span_ticks: None,
                    },
                }
            })
            .collect(),
        car_actor_slots: frame
            .car_players
            .iter()
            .map(|(a, p)| (a.0, usize::from(p.0)))
            .collect(),
        sleeping_velocity_inferred: frame
            .sleeping_velocity_zeroed
            .iter()
            .map(|a| a.map(|a| a.0))
            .collect(),
        demolition_inferred: frame.wrecks_inferred.iter().map(|a| a.0).collect(),
        dead_shells_held: frame
            .wrecks_held
            .iter()
            .map(|(p, s)| DeadShellHold {
                slot: usize::from(p.0),
                source: match s {
                    HoldSource::Observed => "observed",
                    HoldSource::Inferred => "inferred",
                },
            })
            .collect(),
        spawn_pose_held: frame.spawning.iter().map(|p| usize::from(p.0)).collect(),
        ball_fresh: frame.ball_updated,
        fresh_car_slots: frame
            .updated_players
            .iter()
            .map(|p| usize::from(p.0))
            .collect(),
        state: frame.state,
    }
}

/// Converts `network` with v1's `options` through v2, in v1's output types.
pub fn convert_network(
    network: &NetworkReplay,
    options: &ConvertOptions,
    source_sha256: Option<String>,
) -> Result<ConversionOutput, Box<dyn Error>> {
    if options.external_packet_lags.is_some() {
        return Err("external packet lags are a v1 experiment".into());
    }
    let withheld_frames: Option<Vec<bool>> =
        options.withheld_frames.as_ref().map(|w| w.as_ref().clone());
    let withheld = Withheld(withheld_frames.as_deref());
    let inference_options = InferenceOptions {
        air_lookahead: options.infer_air_controls_from_lookahead,
        air_schedules: options.air_bvp,
        input_fits: options.input_fits,
        fit_on_next_update: options.fit_on_next_packet,
        flip_cancel_holdout: options.flip_cancel_holdout,
        dodge_first_update_tick: options.infer_dodge_first_packet_tick,
        seed: options.seed,
    };
    let simulation_options = SimulationOptions {
        seed: options.seed,
        loadout_hitboxes: options.use_loadout_hitboxes,
        simulated_pad_pickups: !options.block_sim_pad_pickups,
        simulated_demolitions: !options.disable_simulated_demolitions,
        withheld: withheld_frames.clone(),
    };
    let meshes = replicar::Meshes::load(&options.collision_meshes)?;
    let inferred = options.infer_packet_lag && !options.zero_packet_lag;
    let mut ticks = if !options.infer_packet_lag {
        None
    } else if options.zero_packet_lag {
        Some(replicar::update_ticks::lag_free(network))
    } else {
        Some(replicar::update_ticks::infer(
            network,
            options.use_loadout_hitboxes,
            withheld,
        ))
    };
    if options.align_contacts
        && inferred
        && let Some(found) = &ticks
    {
        ticks = Some(
            replicar::align::align_contacts(
                network,
                found,
                &meshes,
                inference_options,
                &simulation_options,
            )?
            .0,
        );
    }
    let mut inference = FittedInference::new(network, ticks.as_ref(), inference_options, withheld);
    let mut annotator = Annotator::new(match (&ticks, inferred) {
        (Some(ticks), true) => ball_intervals(&network.frames, ticks, withheld),
        _ => Vec::new(),
    });
    let scoreboard = reconstruct(&network.frames);
    let mut decider = Decider::default();
    let mut frames = Vec::with_capacity(network.frames.len());
    let mut position_residuals = Vec::new();
    let simulation = simulate(
        network,
        ticks.as_ref(),
        &mut inference,
        &meshes,
        simulation_options,
        |frame| {
            position_residuals.extend(residuals(network, &frame));
            let annotations = annotator.annotate(&frame);
            let board = decider.apply(scoreboard[frame.index.get()], &frame.events);
            frames.push(converted_frame(network, frame, annotations, board));
        },
    )?;
    let d = &simulation.diagnostics;
    let i = &inference.diagnostics;
    Ok(ConversionOutput {
        source_sha256,
        options: options.clone(),
        observations: crate::v1_shape::observed_replay(network),
        frames,
        position_residuals,
        car_slots: simulation
            .players
            .iter()
            .map(|p| CarSlot {
                slot: usize::from(p.index.0),
                player_key: p.key.0.clone(),
                team: p.team.number(),
                body_product_id: p.body_product_id,
                hitbox: p.hitbox.name().to_owned(),
            })
            .collect(),
        diagnostics: Diagnostics {
            skipped_timeline_ticks: d.skipped_replay_ticks,
            ball_interval_error: None,
            slot_loadout_changes: d.player_loadout_changes,
            unlinked_car_frames: d.unlinked_car_frames,
            default_hitbox_players: d.default_hitbox_players,
            active_pawn_demo_corrections: d.active_car_demolition_corrections,
            dodge_refreshes_observed: d.flip_resets_observed,
            dodge_refreshes_applied: d.flip_resets_applied,
            sleeping_car_packets: d.sleeping_car_updates,
            cars_started_from_spawn_trajectory: d.cars_started_from_spawn_pose,
            goal_explosion_demolitions: d.goal_explosion_demolitions,
            dead_shells_inferred: d.wrecks_inferred,
            dead_shells_after_demolition: d.wrecks_after_demolition,
            dead_shells_released: d.wrecks_released,
            sleeping_ball_packets: d.sleeping_ball_updates,
            shadowed_car_frames: d.shadowed_car_frames,
            ball_lag_frames: d.ball_tick_frames,
            car_lag_frames: d.car_tick_frames,
            dodge_activations: d.dodge_activations,
            dodge_starts_fitted: i.dodge_starts_fitted,
            air_bvp_planned: i.air_schedules_planned,
            air_bvp_refused: i.air_schedules_refused,
        },
    })
}

/// Converts a replay file's bytes with v1's `options` through v2.
pub fn convert_bytes(
    bytes: &[u8],
    options: &ConvertOptions,
) -> Result<ConversionOutput, Box<dyn Error>> {
    use sha2::{Digest, Sha256};
    let network = replicar::decode::decode(&replicar::parse(bytes)?)?;
    convert_network(
        &network,
        options,
        Some(format!("{:x}", Sha256::digest(bytes))),
    )
}
