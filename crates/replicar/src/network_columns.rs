//! The `network` group (docs/glossary.md, "Network group"): every value replicar reads from the replay, per
//! frame, in the replay's units, each with the frame of its last change (`<name>_frame`). Per player, the
//! values of the player's current car in that frame and of the player's own actor. Columns start with
//! `network_` so that they never share a name with the state's.

use replicar_format::{Columns, RecordBatch, child_bool, child_i32, child_u8};

use crate::decode::{NetworkBody, NetworkCar, NetworkPlayer, NetworkReplay, NetworkValue};
use crate::simulate::SimPlayer;

const GROUP: &str = "network";
const XYZ: [&str; 3] = ["x", "y", "z"];
const XYZW: [&str; 4] = ["x", "y", "z", "w"];

/// A named network value of a car or player.
type Getter<S, T> = (&'static str, fn(&S) -> Option<&NetworkValue<T>>);

/// The frame of a value's last change.
fn frame_of<T>(value: Option<&NetworkValue<T>>) -> Option<u32> {
    value.map(|v| v.frame.0)
}

/// A float vector value: one column per component, and its frame.
fn vector<const N: usize>(
    columns: &mut Columns,
    name: &str,
    components: [&str; N],
    values: &[Option<&NetworkValue<[f32; N]>>],
) {
    for (i, component) in components.iter().enumerate() {
        columns.f32(
            format!("{name}_{component}"),
            GROUP,
            None,
            values.iter().map(|v| v.map(|v| v.value[i])),
        );
    }
    columns.u32(
        format!("{name}_frame"),
        GROUP,
        None,
        values.iter().map(|v| frame_of(*v)),
    );
}

/// A scalar value and its frame; `column` writes the value column.
fn scalar<T: Copy>(
    columns: &mut Columns,
    name: &str,
    values: &[Option<&NetworkValue<T>>],
    column: impl FnOnce(&mut Columns, String, Vec<Option<T>>),
) {
    column(
        columns,
        name.to_owned(),
        values.iter().map(|v| v.map(|v| v.value)).collect(),
    );
    columns.u32(
        format!("{name}_frame"),
        GROUP,
        None,
        values.iter().map(|v| frame_of(*v)),
    );
}

fn f32s(columns: &mut Columns, name: String, values: Vec<Option<f32>>) {
    columns.f32(name, GROUP, None, values);
}

fn u8s(columns: &mut Columns, name: String, values: Vec<Option<u8>>) {
    columns.u8(name, GROUP, None, values);
}

fn i32s(columns: &mut Columns, name: String, values: Vec<Option<i32>>) {
    columns.i32(name, GROUP, values);
}

fn u32s(columns: &mut Columns, name: String, values: Vec<Option<u32>>) {
    columns.u32(name, GROUP, None, values);
}

fn bools(columns: &mut Columns, name: String, values: Vec<Option<bool>>) {
    columns.bool(name, GROUP, values);
}

/// A body's values; `body` gives each frame's body.
fn body_columns(columns: &mut Columns, name: &str, bodies: &[Option<&NetworkBody>]) {
    let field = |f: fn(&NetworkBody) -> Option<&NetworkValue<[f32; 3]>>| {
        bodies.iter().map(|b| b.and_then(f)).collect::<Vec<_>>()
    };
    vector(
        columns,
        &format!("{name}_position"),
        XYZ,
        &field(|b| b.position.as_ref()),
    );
    let rotation: Vec<_> = bodies
        .iter()
        .map(|b| b.and_then(|b| b.rotation.as_ref()))
        .collect();
    vector(columns, &format!("{name}_rotation"), XYZW, &rotation);
    vector(
        columns,
        &format!("{name}_velocity"),
        XYZ,
        &field(|b| b.linear_velocity.as_ref()),
    );
    vector(
        columns,
        &format!("{name}_angular_velocity_raw"),
        XYZ,
        &field(|b| b.angular_velocity_raw.as_ref()),
    );
    let sleeping: Vec<_> = bodies
        .iter()
        .map(|b| b.and_then(|b| b.sleeping.as_ref()))
        .collect();
    scalar(columns, &format!("{name}_sleeping"), &sleeping, bools);
}

fn car_columns(columns: &mut Columns, name: &str, cars: &[Option<&NetworkCar>]) {
    columns.i32(
        format!("{name}_actor"),
        GROUP,
        cars.iter().map(|c| c.map(|c| c.life.actor.0)),
    );
    columns.u32(
        format!("{name}_created"),
        GROUP,
        None,
        cars.iter().map(|c| c.map(|c| c.life.created.0)),
    );
    columns.bool(
        format!("{name}_player_link_active"),
        GROUP,
        cars.iter().map(|c| c.map(|c| c.player_link_active)),
    );
    let bodies: Vec<_> = cars.iter().map(|c| c.map(|c| &c.body)).collect();
    body_columns(columns, name, &bodies);
    let value = |f: fn(&NetworkCar) -> Option<&NetworkValue<f32>>| {
        cars.iter().map(|c| c.and_then(f)).collect::<Vec<_>>()
    };
    scalar(
        columns,
        &format!("{name}_boost"),
        &value(|c| c.boost.as_ref()),
        f32s,
    );
    scalar(
        columns,
        &format!("{name}_throttle"),
        &value(|c| c.inputs.throttle.as_ref()),
        f32s,
    );
    scalar(
        columns,
        &format!("{name}_steer"),
        &value(|c| c.inputs.steer.as_ref()),
        f32s,
    );
    let byte = |f: fn(&NetworkCar) -> Option<&NetworkValue<u8>>| {
        cars.iter().map(|c| c.and_then(f)).collect::<Vec<_>>()
    };
    let counters: [Getter<NetworkCar, u8>; 6] = [
        ("boost_raw", |c| c.boost_raw.as_ref()),
        ("boost_active_raw", |c| c.inputs.boost_active_raw.as_ref()),
        ("jump_active_raw", |c| c.inputs.jump_active_raw.as_ref()),
        ("double_jump_active_raw", |c| {
            c.inputs.double_jump_active_raw.as_ref()
        }),
        ("dodge_active_raw", |c| c.inputs.dodge_active_raw.as_ref()),
        ("flip_car_active_raw", |c| {
            c.inputs.flip_car_active_raw.as_ref()
        }),
    ];
    for (field, get) in counters {
        scalar(columns, &format!("{name}_{field}"), &byte(get), u8s);
    }
    let handbrake: Vec<_> = cars
        .iter()
        .map(|c| c.and_then(|c| c.inputs.handbrake.as_ref()))
        .collect();
    scalar(columns, &format!("{name}_handbrake"), &handbrake, bools);
    let torque: Vec<_> = cars
        .iter()
        .map(|c| c.and_then(|c| c.inputs.dodge_torque_raw.as_ref()))
        .collect();
    vector(columns, &format!("{name}_dodge_torque_raw"), XYZ, &torque);
    let body: Vec<_> = cars
        .iter()
        .map(|c| c.and_then(|c| c.body_product_id.as_ref()))
        .collect();
    scalar(columns, &format!("{name}_body_product_id"), &body, u32s);
}

fn player_columns(columns: &mut Columns, name: &str, players: &[Option<&NetworkPlayer>]) {
    let stats: [Getter<NetworkPlayer, i32>; 16] = [
        ("match_score", |p| p.stats.match_score.as_ref()),
        ("goals", |p| p.stats.goals.as_ref()),
        ("assists", |p| p.stats.assists.as_ref()),
        ("saves", |p| p.stats.saves.as_ref()),
        ("shots", |p| p.stats.shots.as_ref()),
        ("demolitions", |p| p.stats.demolitions.as_ref()),
        ("epic_saves", |p| p.stats.epic_saves.as_ref()),
        ("clears", |p| p.stats.clears.as_ref()),
        ("centers", |p| p.stats.centers.as_ref()),
        ("aerial_hits", |p| p.stats.aerial_hits.as_ref()),
        ("first_touches", |p| p.stats.first_touches.as_ref()),
        ("crossbar_hits", |p| p.stats.crossbar_hits.as_ref()),
        ("bicycle_hits", |p| p.stats.bicycle_hits.as_ref()),
        ("juggle_hits", |p| p.stats.juggle_hits.as_ref()),
        ("flip_resets", |p| p.stats.flip_resets.as_ref()),
        ("times_demolished", |p| p.stats.times_demolished.as_ref()),
    ];
    for (field, get) in stats {
        let values: Vec<_> = players.iter().map(|p| p.and_then(get)).collect();
        scalar(columns, &format!("{name}_{field}"), &values, i32s);
    }
    let ping: Vec<_> = players
        .iter()
        .map(|p| p.and_then(|p| p.ping_raw.as_ref()))
        .collect();
    scalar(columns, &format!("{name}_ping_raw"), &ping, u8s);
}

/// The group's columns, one row per frame of `network`; `players` are the simulation's, in player order.
pub(crate) fn network_columns(
    network: &NetworkReplay,
    players: &[SimPlayer],
) -> Result<RecordBatch, replicar_format::WriteError> {
    let frames = &network.frames;
    let mut columns = Columns::default();
    let clock: Vec<_> = frames
        .iter()
        .map(|f| f.seconds_remaining.as_ref())
        .collect();
    scalar(&mut columns, "network_seconds_remaining", &clock, i32s);
    let overtime: Vec<_> = frames.iter().map(|f| f.overtime.as_ref()).collect();
    scalar(&mut columns, "network_overtime", &overtime, bools);
    for (team, name) in ["blue", "orange"].iter().enumerate() {
        let scores: Vec<_> = frames
            .iter()
            .map(|f| f.team_scores[team].as_ref())
            .collect();
        scalar(
            &mut columns,
            &format!("network_{name}_score"),
            &scores,
            i32s,
        );
    }
    columns.strings(
        "network_game_state",
        GROUP,
        frames.iter().map(|f| {
            f.game_state.as_ref().map(|s| match &s.value {
                crate::decode::GameState::Other(name) => name.clone(),
                state => format!("{state:?}"),
            })
        }),
    );
    columns.u32(
        "network_game_state_frame",
        GROUP,
        None,
        frames.iter().map(|f| frame_of(f.game_state.as_ref())),
    );
    let balls: Vec<_> = frames.iter().map(|f| f.ball.as_ref()).collect();
    body_columns(&mut columns, "network_ball", &balls);
    for player in players {
        let p = player.index.0;
        let cars: Vec<_> = frames
            .iter()
            .map(|f| {
                f.current_cars()
                    .into_iter()
                    .find(|c| c.player.as_ref() == Some(&player.key))
            })
            .collect();
        car_columns(&mut columns, &format!("network_car_{p}"), &cars);
        let actors: Vec<_> = frames
            .iter()
            .map(|f| f.players.iter().find(|q| q.key == player.key))
            .collect();
        player_columns(&mut columns, &format!("network_player_{p}"), &actors);
    }
    let records: Vec<_> = frames.iter().flat_map(|f| &f.pad_records).collect();
    let lengths: Vec<usize> = frames.iter().map(|f| f.pad_records.len()).collect();
    columns.records(
        "network_pad_records",
        GROUP,
        &lengths,
        vec![
            child_i32("pad", records.iter().map(|r| Some(r.pad.0)).collect()),
            child_i32(
                "instigator_car",
                records
                    .iter()
                    .map(|r| r.instigator_car.map(|c| c.0))
                    .collect(),
            ),
            child_u8(
                "picked_up_raw",
                records.iter().map(|r| Some(r.picked_up_raw)).collect(),
            ),
            child_bool("repeat", records.iter().map(|r| Some(r.repeat)).collect()),
        ],
    );
    Ok(columns.finish()?)
}
