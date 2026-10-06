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
