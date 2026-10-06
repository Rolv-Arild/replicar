//! v2's decoded network feed in v1's observation types, so the two can be compared value for value
//! (`parity decode`). The mapping is explicit: v2's names and types differ, the values must not.

use replicar::decode::{
    DemolitionReport, NetworkBody, NetworkCar, NetworkEvent, NetworkFrame, NetworkInputs,
    NetworkPlayer, NetworkReplay, NetworkValue, PadRecord, PlayerStats, SpawnPose, ValueSource,
};
use replicar_v1::observations as v1;

fn value<T: Clone>(v: &NetworkValue<T>) -> v1::Value<T> {
    v1::Value {
        value: v.value.clone(),
        frame: v.frame.get(),
        source: match v.source {
            ValueSource::Replay => v1::Source::Replay,
            ValueSource::InferredMatchStart => v1::Source::InferredMatchStart,
        },
    }
}

fn opt<T: Clone>(v: &Option<NetworkValue<T>>) -> Option<v1::Value<T>> {
    v.as_ref().map(value)
}

fn body(b: &NetworkBody) -> v1::Body {
    v1::Body {
        position: opt(&b.position),
        rotation_xyzw: opt(&b.rotation),
        linear_velocity: opt(&b.linear_velocity),
        angular_velocity_replay_units: opt(&b.angular_velocity_raw),
        sleeping: opt(&b.sleeping),
    }
}

fn spawn(s: &SpawnPose) -> v1::SpawnPose {
    v1::SpawnPose {
        position: s.position,
        rotation_xyzw: s.rotation,
        frame: s.frame.get(),
    }
}

fn inputs(i: &NetworkInputs) -> v1::Inputs {
    v1::Inputs {
        throttle: opt(&i.throttle),
        steer: opt(&i.steer),
        handbrake: opt(&i.handbrake),
        boost_active_raw: opt(&i.boost_active_raw),
        jump_active_raw: opt(&i.jump_active_raw),
        double_jump_active_raw: opt(&i.double_jump_active_raw),
        dodge_active_raw: opt(&i.dodge_active_raw),
        dodge_torque_replay_units: opt(&i.dodge_torque_raw),
        flip_car_active_raw: opt(&i.flip_car_active_raw),
    }
}

fn car(c: &NetworkCar) -> v1::Car {
    v1::Car {
        actor_id: c.life.actor.0,
        actor_created_frame: c.life.created.get(),
        player_key: c.player.as_ref().map(|key| key.0.clone()),
        player_link_active: c.player_link_active,
        team: c.team.map(|team| team.number()),
        body_product_id: opt(&c.body_product_id),
        body: body(&c.body),
        boost: opt(&c.boost),
        boost_raw: opt(&c.boost_raw),
        inputs: inputs(&c.inputs),
        spawn_pose: c.spawn_pose.as_ref().map(spawn),
    }
}

fn stats(s: &PlayerStats) -> v1::PlayerStats {
    v1::PlayerStats {
        match_score: opt(&s.match_score),
        goals: opt(&s.goals),
        assists: opt(&s.assists),
        saves: opt(&s.saves),
        shots: opt(&s.shots),
        demolishes: opt(&s.demolitions),
    }
}

fn player(p: &NetworkPlayer) -> v1::Player {
    v1::Player {
        actor_id: p.actor.0,
        key: p.key.0.clone(),
        name: p.name.clone(),
        team: p.team.map(|team| team.number()),
        body_product_ids: [opt(&p.body_product_ids[0]), opt(&p.body_product_ids[1])],
        stats: stats(&p.stats),
        ping_raw: opt(&p.ping_raw),
    }
}

fn event(e: &NetworkEvent) -> v1::Event {
    match e {
        NetworkEvent::GoalScoredOn { team } => v1::Event::GoalScoredOn {
            team: team.number(),
        },
        NetworkEvent::Demolition {
            report,
            attacker_car,
            victim_car,
            attacker_player,
            self_demolition,
            attacker_velocity_raw,
            victim_velocity_raw,
            repeat,
        } => v1::Event::Demolish {
            source: match report {
                DemolitionReport::Extended => "extended",
                DemolitionReport::Plain => "plain",
                DemolitionReport::GoalExplosion => "goal_explosion",
            },
            attacker_car: attacker_car.map(|a| a.0),
            victim_car: victim_car.map(|a| a.0),
            attacker_pri: attacker_player.map(|a| a.0),
            self_demolish: *self_demolition,
            attacker_velocity: *attacker_velocity_raw,
            victim_velocity: *victim_velocity_raw,
            repeat: *repeat,
        },
        NetworkEvent::FlipReset { car, count } => v1::Event::DodgeRefreshed {
            car: car.0,
            count: *count,
        },
    }
}

fn pad(r: &PadRecord) -> v1::PadPickup {
    v1::PadPickup {
        pad_actor_id: r.pad.0,
        pad_actor_name: r.pad_name.clone(),
        instigator_car_id: r.instigator_car.map(|a| a.0),
        picked_up: r.picked_up_raw,
        repeat: r.repeat,
    }
}

fn frame(f: &NetworkFrame) -> v1::Frame {
    v1::Frame {
        index: f.index.get(),
        time: f.time,
        delta: f.delta,
        ball: f.ball.as_ref().map(body),
        cars: f.cars.iter().map(car).collect(),
        players: f.players.iter().map(player).collect(),
        team_scores: [opt(&f.team_scores[0]), opt(&f.team_scores[1])],
        seconds_remaining: opt(&f.seconds_remaining),
        overtime: opt(&f.overtime),
        game_state: f.game_state.as_ref().map(|state| v1::Value {
            value: state.value.name().to_owned(),
            frame: state.frame.get(),
            source: match state.source {
                ValueSource::Replay => v1::Source::Replay,
                ValueSource::InferredMatchStart => v1::Source::InferredMatchStart,
            },
        }),
        events: f.events.iter().map(event).collect(),
        pad_pickups: f.pad_records.iter().map(pad).collect(),
    }
}

/// v2's decoded replay as v1's observations.
pub fn observed_replay(replay: &NetworkReplay) -> v1::ObservedReplay {
    let d = &replay.diagnostics;
    v1::ObservedReplay {
        header: v1::Header {
            game_type: replay.header.game_type.clone(),
            levels: replay.header.levels.clone(),
            final_team_scores: replay.header.final_scores,
        },
        frames: replay.frames.iter().map(frame).collect(),
        diagnostics: v1::Diagnostics {
            repeated_actor_announcements: d.repeated_actor_announcements,
            actor_class_replacements: d.actor_class_replacements,
            unknown_actor_updates: d.unknown_actor_updates,
            unlinked_car_frames: d.unlinked_car_frames,
            map_name: d.map_name.clone(),
            game_settings: d.game_settings.clone(),
            nonstandard_notes: d.nonstandard_notes.clone(),
            player_key_changes: d.player_key_changes,
            players_deleted_with_cars: d.players_deleted_with_cars,
            replacement_cars_linked: d.replacement_cars_linked,
            replacement_cars_linked_after_creation: d.replacement_cars_linked_after_creation,
            non_monotonic_frame_times: d.non_monotonic_frame_times,
        },
    }
}

/// v1's packet lags in a canonical form: car runs sorted by (actor, creation frame, first frame), and each
/// car update's run named by that identity instead of v1's index (v1 builds its runs from a `HashMap`, so
/// their order varies between runs of the program).
pub fn canonical_lags_v1(lags: &replicar_v1::conversion::PacketLags) -> serde_json::Value {
    let run_id = |run: &replicar_v1::conversion::CarRun| {
        format!(
            "{}:{}:{}",
            run.actor,
            run.created,
            run.entries.first().map_or(0, |e| e.0)
        )
    };
    let mut runs: Vec<serde_json::Value> = lags
        .car_runs
        .iter()
        .map(|run| {
            serde_json::json!({
                "id": run_id(run),
                "entries": run.entries.iter().map(|&(f, k)| (f, k)).collect::<Vec<_>>(),
                "lo": run.lo, "hi": run.hi, "start": run.start,
            })
        })
        .collect();
    runs.sort_by(|a, b| a["id"].as_str().cmp(&b["id"].as_str()));
    let mut cars: Vec<(String, f32, String)> = lags
        .car_actor
        .iter()
        .map(|(&(actor, created, frame), &lag)| {
            let run = lags
                .car_run_of
                .get(&(actor, created, frame))
                .map_or_else(String::new, |&i| run_id(&lags.car_runs[i]));
            (format!("{actor}:{created}:{frame}"), lag, run)
        })
        .collect();
    cars.sort_by(|a, b| a.0.cmp(&b.0));
    serde_json::json!({
        "ball": lags.ball,
        "car_median": lags.cars,
        "cars": cars,
        "ball_car_offset": lags.ball_car_offset,
        "bridged_hits": lags.bridged_hits,
        "lag_free": lags.lag_free,
        "car_runs": runs,
    })
}

/// v2's update ticks in the same canonical form.
pub fn canonical_lags_v2(ticks: &replicar::update_ticks::UpdateTicks) -> serde_json::Value {
    let run_id = |run: &replicar::update_ticks::CarRun| {
        format!(
            "{}:{}:{}",
            run.life.actor.0,
            run.life.created.0,
            run.entries.first().map_or(0, |e| e.0.0)
        )
    };
    let mut runs: Vec<serde_json::Value> = ticks
        .car_runs
        .iter()
        .map(|run| {
            serde_json::json!({
                "id": run_id(run),
                "entries": run.entries.iter().map(|&(f, k)| (f.0, k)).collect::<Vec<_>>(),
                "lo": run.lo, "hi": run.hi, "start": run.start,
            })
        })
        .collect();
    runs.sort_by(|a, b| a["id"].as_str().cmp(&b["id"].as_str()));
    let as_f32 = |v: &[Option<u32>]| v.iter().map(|x| x.map(|x| x as f32)).collect::<Vec<_>>();
    let mut cars: Vec<(String, f32, String)> = ticks
        .cars
        .iter()
        .map(|(&(life, frame), &lag)| {
            let run = ticks
                .car_run_of
                .get(&(life, frame))
                .map_or_else(String::new, |&i| run_id(&ticks.car_runs[i]));
            (
                format!("{}:{}:{}", life.actor.0, life.created.0, frame.0),
                lag as f32,
                run,
            )
        })
        .collect();
    cars.sort_by(|a, b| a.0.cmp(&b.0));
    serde_json::json!({
        "ball": as_f32(&ticks.ball),
        "car_median": as_f32(&ticks.car_median),
        "cars": cars,
        "ball_car_offset": ticks.ball_car_offset,
        "bridged_hits": ticks.bridged_hits,
        "lag_free": ticks.lag_free,
        "car_runs": runs,
    })
}

/// One frame of v1's conversion, for the `simulate` stage: the state (as `Debug`, so equal text means equal
/// bits), RocketSim's events, the applied ticks, the holds and provenance lists.
pub fn simulated_frame_v1(frame: &replicar_v1::conversion::ConvertedFrame) -> serde_json::Value {
    serde_json::json!({
        "replay_tick": frame.timeline_tick,
        "state": format!("{:?}", frame.state),
        "events": frame.simulated_events.iter().map(|e| format!("{} {:?}", e.arena_tick, e.event)).collect::<Vec<_>>(),
        "applied": frame.packet_lags.iter().map(|l| format!("{:?} {} {}", l.actor_id, l.ticks, l.source)).collect::<Vec<_>>(),
        "sleeping": frame.sleeping_velocity_inferred,
        "wrecks_inferred": frame.demolition_inferred,
        "wrecks_held": frame.dead_shells_held.iter().map(|h| format!("{} {}", h.slot, h.source)).collect::<Vec<_>>(),
        "spawning": frame.spawn_pose_held,
        "car_players": frame.car_actor_slots,
        "ball_updated": frame.ball_fresh,
        "updated_players": frame.fresh_car_slots,
    })
}

/// The same for v2's simulated frame.
pub fn simulated_frame_v2(frame: &replicar::simulate::SimulatedFrame) -> serde_json::Value {
    use replicar::simulate::{HoldSource, TickSource};
    serde_json::json!({
        "replay_tick": frame.replay_tick,
        "state": format!("{:?}", frame.state),
        "events": frame.events.iter().map(|e| format!("{} {:?}", e.sim_tick, e.event)).collect::<Vec<_>>(),
        "applied": frame.applied_ticks.iter().map(|t| {
            let source = match t.source {
                TickSource::Chain => "chain",
                TickSource::FrameMedian => "frame_median",
                TickSource::Default => "default",
                TickSource::DodgeFit => "dodge_fit",
            };
            format!("{:?} {} {source}", t.car.map(|a| a.0), t.ticks)
        }).collect::<Vec<_>>(),
        "sleeping": frame.sleeping_velocity_zeroed.iter().map(|a| a.map(|a| a.0)).collect::<Vec<_>>(),
        "wrecks_inferred": frame.wrecks_inferred.iter().map(|a| a.0).collect::<Vec<_>>(),
        "wrecks_held": frame.wrecks_held.iter().map(|(p, s)| format!("{} {}", p.0, match s {
            HoldSource::Observed => "observed",
            HoldSource::Inferred => "inferred",
        })).collect::<Vec<_>>(),
        "spawning": frame.spawning.iter().map(|p| usize::from(p.0)).collect::<Vec<_>>(),
        "car_players": frame.car_players.iter().map(|(a, p)| (a.0, usize::from(p.0))).collect::<Vec<_>>(),
        "ball_updated": frame.ball_updated,
        "updated_players": frame.updated_players.iter().map(|p| usize::from(p.0)).collect::<Vec<_>>(),
    })
}

/// v1's car slots and simulation counters, for the `simulate` stage.
pub fn simulation_v1(summary: &replicar_v1::conversion::ConversionSummary) -> serde_json::Value {
    let d = &summary.diagnostics;
    serde_json::json!({
        "players": summary.car_slots.iter().map(|s| format!("{} {} {} {:?} {}", s.slot, s.player_key, s.team, s.body_product_id, s.hitbox)).collect::<Vec<_>>(),
        "counters": [d.skipped_timeline_ticks as usize, d.slot_loadout_changes, d.unlinked_car_frames, d.default_hitbox_players,
            d.active_pawn_demo_corrections, d.dodge_refreshes_observed, d.dodge_refreshes_applied, d.sleeping_car_packets,
            d.cars_started_from_spawn_trajectory, d.goal_explosion_demolitions, d.dead_shells_inferred,
            d.dead_shells_after_demolition, d.dead_shells_released, d.sleeping_ball_packets, d.shadowed_car_frames,
            d.ball_lag_frames, d.car_lag_frames, d.dodge_activations],
    })
}

/// The same for v2's simulation.
pub fn simulation_v2(simulation: &replicar::simulate::Simulation) -> serde_json::Value {
    let d = &simulation.diagnostics;
    serde_json::json!({
        "players": simulation.players.iter().map(|p| format!("{} {} {} {:?} {}", p.index.0, p.key.0, p.team.number(), p.body_product_id, p.hitbox.name())).collect::<Vec<_>>(),
        "counters": [d.skipped_replay_ticks as usize, d.player_loadout_changes, d.unlinked_car_frames, d.default_hitbox_players,
            d.active_car_demolition_corrections, d.flip_resets_observed, d.flip_resets_applied, d.sleeping_car_updates,
            d.cars_started_from_spawn_pose, d.goal_explosion_demolitions, d.wrecks_inferred,
            d.wrecks_after_demolition, d.wrecks_released, d.sleeping_ball_updates, d.shadowed_car_frames,
            d.ball_tick_frames, d.car_tick_frames, d.dodge_activations],
    })
}
