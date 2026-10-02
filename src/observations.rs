//! Typed replay observations with actor identity and per-field freshness.

use std::collections::{BTreeMap, HashMap};

use boxcars::{ActorId, Attribute, HeaderProp, Replay, Vector3f};
use serde::Serialize;

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Source {
    Replay,
    InferredMatchStart,
}

#[derive(Debug, Clone, Serialize)]
pub struct Value<T> {
    pub value: T,
    pub frame: usize,
    pub source: Source,
}

impl<T> Value<T> {
    fn replay(value: T, frame: usize) -> Self {
        Self {
            value,
            frame,
            source: Source::Replay,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct Body {
    pub position: Option<Value<[f32; 3]>>,
    pub rotation_xyzw: Option<Value<[f32; 4]>>,
    pub linear_velocity: Option<Value<[f32; 3]>>,
    /// Multiply these boxcars values by 0.01 for RocketSim radians per second.
    pub angular_velocity_replay_units: Option<Value<[f32; 3]>>,
    pub sleeping: Option<Value<bool>>,
}

impl Body {
    fn update(&mut self, body: &boxcars::RigidBody, frame: usize) {
        self.position = Some(Value::replay(vector(body.location), frame));
        self.rotation_xyzw = Some(Value::replay(
            [
                body.rotation.x,
                body.rotation.y,
                body.rotation.z,
                body.rotation.w,
            ],
            frame,
        ));
        self.sleeping = Some(Value::replay(body.sleeping, frame));
        if let Some(velocity) = body.linear_velocity {
            self.linear_velocity = Some(Value::replay(vector(velocity), frame));
        }
        if let Some(velocity) = body.angular_velocity {
            self.angular_velocity_replay_units = Some(Value::replay(vector(velocity), frame));
        }
    }
}

fn vector(v: Vector3f) -> [f32; 3] {
    [v.x, v.y, v.z]
}

/// Boxcars' spawn trajectory of a car actor: where the replay creates it. Carried by a car only until its
/// first rigid-body packet (the first packet follows within 0-3 frames; against it the spawn location is
/// within 3 UU at p90 and 10 UU at most on train), so a car that is simulated before that does not sit at
/// RocketSim's default pose. Inferred, not observed.
#[derive(Debug, Clone, Serialize)]
pub struct SpawnPose {
    pub position: [f32; 3],
    /// The heading about the vertical axis from the trajectory's compressed angle (the field boxcars
    /// calls `pitch`, in 1/256 turns); `None` when the trajectory also tilts the car (not decoded).
    pub rotation_xyzw: Option<[f32; 4]>,
    pub frame: usize,
}

impl SpawnPose {
    fn from_trajectory(trajectory: &boxcars::Trajectory, frame: usize) -> Option<Self> {
        let location = trajectory.location?;
        let rotation = trajectory.rotation.as_ref().and_then(|r| {
            let tilt = |v: Option<i8>| v.is_none_or(|v| v.abs() <= 1);
            (tilt(r.yaw) && tilt(r.roll)).then(|| {
                let angle = f32::from(r.pitch.unwrap_or(0)) * std::f32::consts::PI / 128.0;
                [0.0, 0.0, (angle / 2.0).sin(), (angle / 2.0).cos()]
            })
        });
        Some(Self {
            position: [location.x as f32, location.y as f32, location.z as f32],
            rotation_xyzw: rotation,
            frame,
        })
    }
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct Inputs {
    pub throttle: Option<Value<f32>>,
    pub steer: Option<Value<f32>>,
    pub handbrake: Option<Value<bool>>,
    /// Boost component activation counter. Odd values appear active in train replay calibration.
    pub boost_active_raw: Option<Value<u8>>,
    pub jump_active_raw: Option<Value<u8>>,
    pub double_jump_active_raw: Option<Value<u8>>,
    pub dodge_active_raw: Option<Value<u8>>,
    /// Raw replay dodge-torque vector; its relation to a controller direction is uncalibrated.
    pub dodge_torque_replay_units: Option<Value<[f32; 3]>>,
    pub flip_car_active_raw: Option<Value<u8>>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Car {
    pub actor_id: i32,
    /// Creation frame for this actor lifetime; repeat keyframe announcements do not change it.
    pub actor_created_frame: usize,
    pub player_key: Option<String>,
    /// Whether the current pawn-to-player link is active. A known owner is retained when it goes inactive.
    pub player_link_active: bool,
    pub team: Option<u8>,
    /// Car-body product ID from this player's loadout for the current team.
    pub body_product_id: Option<Value<u32>>,
    pub body: Body,
    pub boost: Option<Value<f32>>,
    pub boost_raw: Option<Value<u8>>,
    pub inputs: Inputs,
    /// The spawn trajectory, present only while the car has no rigid-body packet yet (`SpawnPose`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub spawn_pose: Option<SpawnPose>,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct PlayerStats {
    pub match_score: Option<Value<i32>>,
    pub goals: Option<Value<i32>>,
    pub assists: Option<Value<i32>>,
    pub saves: Option<Value<i32>>,
    pub shots: Option<Value<i32>>,
    pub demolishes: Option<Value<i32>>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Player {
    pub actor_id: i32,
    pub key: String,
    pub name: Option<String>,
    pub team: Option<u8>,
    /// Blue and orange car-body product IDs; the selected body can differ by team.
    pub body_product_ids: [Option<Value<u32>>; 2],
    pub stats: PlayerStats,
}

/// Seconds within which a second report of one victim is a repeat.
pub const DEMOLITION_REPEAT_WINDOW: f32 = 5.0;

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Event {
    GoalScoredOn { team: u8 },
    /// A demolition replicated on the victim car (`ReplicatedDemolish*`). Car fields are replay car
    /// actor ids (the player a car belongs to comes from `Frame::cars`); velocities are in replay
    /// units. The same demolition can be replicated in more than one update.
    Demolish {
        /// `extended`, `plain`, or `goal_explosion` (the celebration demolition after a goal).
        source: &'static str,
        attacker_car: Option<i32>,
        victim_car: Option<i32>,
        /// Extended only: the attacker's player (PRI) actor and whether the victim demolished itself.
        attacker_pri: Option<i32>,
        self_demolish: bool,
        attacker_velocity: [f32; 3],
        victim_velocity: [f32; 3],
        /// The same victim car actor was reported as demolished less than 5 s before (a car cannot
        /// be demolished during its 3 s respawn time, and the replay sends a demolition again 200-530
        /// ticks after: 4 of 16 events on the remote-client games). Count only events with `repeat`
        /// false.
        repeat: bool,
    },
    /// The car's `DodgesRefreshedCounter` went up: the car regained its flip in the air (the replay's own
    /// flip-reset indicator, builds from March 2026). `car` is the replay car actor id and `count` the new
    /// total for that car actor. Reported only for an increase over a value already seen for the actor (a
    /// first value above zero, or a re-sent value, is not an event), and seen with the replication delay of
    /// the update, not at the tick of the contact. The counter does not count every reset: the flags also
    /// clear on wheel contact with a car or a wall (RESULTS.md, 'Flip resets').
    DodgeRefreshed { car: i32, count: i32 },
}

#[derive(Debug, Clone, Serialize)]
pub struct PadPickup {
    pub pad_actor_id: i32,
    pub pad_actor_name: Option<String>,
    pub instigator_car_id: Option<i32>,
    /// Non-255 odd values count pickups; 255 marks available/inactive in the train corpus.
    pub picked_up: u8,
    /// The pad's counter value was already reported with an instigator: the replay re-announces
    /// earlier pickups at resets (about a third of the records on the remote-client games), so only
    /// records with `repeat` false are new pickups.
    pub repeat: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct Frame {
    pub index: usize,
    pub time: f32,
    pub delta: f32,
    pub ball: Option<Body>,
    pub cars: Vec<Car>,
    pub players: Vec<Player>,
    pub team_scores: [Option<Value<i32>>; 2],
    pub seconds_remaining: Option<Value<i32>>,
    pub overtime: Option<Value<bool>>,
    pub game_state: Option<Value<String>>,
    pub events: Vec<Event>,
    pub pad_pickups: Vec<PadPickup>,
}

/// Choose the replay car that currently represents each known player. A demolished
/// car actor can remain in the network after a replacement car has appeared.
pub fn primary_linked_cars(frame: &Frame) -> Vec<&Car> {
    let mut by_player: HashMap<&str, &Car> = HashMap::new();
    for car in &frame.cars {
        let Some(key) = car.player_key.as_deref() else {
            continue;
        };
        let priority = |car: &Car| {
            (
                car.player_link_active,
                car.actor_created_frame,
                car.body
                    .position
                    .as_ref()
                    .is_some_and(|v| v.frame == frame.index),
            )
        };
        match by_player.get_mut(key) {
            Some(current) if priority(car) > priority(current) => *current = car,
            None => {
                by_player.insert(key, car);
            }
            _ => {}
        }
    }
    let mut result: Vec<_> = by_player.into_values().collect();
    result.sort_by_key(|car| car.actor_id);
    result
}

#[derive(Debug, Clone, Serialize)]
pub struct Header {
    pub game_type: String,
    pub levels: Vec<String>,
    /// Header scores are final totals and are never copied into earlier frames.
    pub final_team_scores: [Option<i32>; 2],
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct Diagnostics {
    pub repeated_actor_announcements: usize,
    pub actor_class_replacements: usize,
    pub unknown_actor_updates: usize,
    pub unlinked_car_frames: usize,
    /// The replay header's `MapName`.
    pub map_name: Option<String>,
    /// The distinct values the replay announced for the game settings the converter does not model
    /// (`ReplicatedGameMutatorIndex`, `ReplicatedBallGravityScale`, `ReplicatedBallMaxLinearSpeedScale`).
    pub game_settings: BTreeMap<String, Vec<String>>,
    /// Anything that makes the replay non-standard for the soccar arena the converter simulates: a map that
    /// is not in `KNOWN_MAPS`, a game mutator index other than -1, a ball gravity or speed scale other than 1.
    /// A report, not a refusal: the conversion still runs and is simulated as standard soccar.
    pub nonstandard_notes: Vec<String>,
    /// A player's key (unique id, player id, name) changed between frames.
    pub player_key_changes: usize,
    /// A player replication actor was deleted while cars still pointed at it (the cars' link is cleared;
    /// they would otherwise keep a stale owner and simulate for a player who has left).
    pub players_deleted_with_cars: usize,
    /// A car was linked to a player that another car actor lifetime already owned (a replacement car), and
    /// how many of those links came after the car's creation frame.
    pub replacement_cars_linked: usize,
    pub replacement_cars_linked_after_creation: usize,
    /// Frames whose time is earlier than the previous frame's (the conversion clamps them).
    pub non_monotonic_frame_times: usize,
}

/// Map names (lower case) of the train replays, all converted with normal residuals: soccar arenas of the
/// standard dimensions (`Stadium_10A`, Throwback, included: one train replay converts normally). A map
/// outside the list is reported in `Diagnostics::nonstandard_notes`.
const KNOWN_MAPS: [&str; 18] = [
    "chn_stadium_p",
    "eurostadium_dusk_p",
    "eurostadium_night_p",
    "eurostadium_p",
    "ff_dusk_p",
    "farm_grs_p",
    "neotokyo_standard_p",
    "park_rainy_p",
    "stadium_10a_p",
    "stadium_p",
    "trainstation_dawn_p",
    "trainstation_night_p",
    "underwater_grs_p",
    "utopiastadium_dusk_p",
    "beach_night_p",
    "cs_day_p",
    "cs_p",
    "woods_p",
];

fn nonstandard_notes(map: Option<&str>, settings: &BTreeMap<String, Vec<String>>) -> Vec<String> {
    let mut notes = Vec::new();
    match map {
        None => notes.push("the replay header has no MapName".to_owned()),
        Some(map) if !KNOWN_MAPS.contains(&map.to_lowercase().as_str()) => {
            notes.push(format!("map {map} is not one of the maps measured on train"));
        }
        _ => {}
    }
    for (name, values) in settings {
        let standard = match name.as_str() {
            "ProjectX.GRI_X:ReplicatedGameMutatorIndex" => "-1",
            _ => "1",
        };
        for value in values {
            let is_standard = value == standard || value.parse::<f32>().is_ok_and(|v| (v - standard.parse::<f32>().unwrap_or(0.0)).abs() < 1e-6);
            if !is_standard {
                notes.push(format!("{name} = {value} (standard {standard})"));
            }
        }
    }
    notes
}

#[derive(Debug, Clone, Serialize)]
pub struct ObservedReplay {
    pub header: Header,
    pub frames: Vec<Frame>,
    pub diagnostics: Diagnostics,
}

#[derive(Clone)]
enum ActorKind {
    Car,
    Ball,
    Player,
    Team(u8),
    Component(ComponentKind),
    GameEvent,
    Pad(String),
    Other,
}

#[derive(Clone, Copy)]
enum ComponentKind {
    Boost,
    Jump,
    DoubleJump,
    Dodge,
    FlipCar,
}

#[derive(Clone)]
struct Actor {
    class: String,
    kind: ActorKind,
}

#[derive(Clone, Default)]
struct TrackedCar {
    created_frame: usize,
    spawn: Option<SpawnPose>,
    body: Body,
    player_actor: Option<ActorId>,
    player_link_active: bool,
    boost: Option<Value<f32>>,
    boost_raw: Option<Value<u8>>,
    inputs: Inputs,
}

#[derive(Clone, Default)]
struct TrackedPlayer {
    unique_id: Option<String>,
    player_id: Option<i32>,
    name: Option<String>,
    team_actor: Option<ActorId>,
    body_product_ids: [Option<Value<u32>>; 2],
    stats: PlayerStats,
}

#[derive(Default)]
struct Tracker {
    actors: HashMap<ActorId, Actor>,
    cars: HashMap<ActorId, TrackedCar>,
    players: HashMap<ActorId, TrackedPlayer>,
    components: HashMap<ActorId, ActorId>,
    ball: Option<(ActorId, Body)>,
    team_scores: [Option<Value<i32>>; 2],
    seconds_remaining: Option<Value<i32>>,
    overtime: Option<Value<bool>>,
    game_state: Option<Value<String>>,
    /// Teams of the goal events already reported in the current post-goal phase: the attribute is
    /// sometimes sent again 100-150 ticks later (two goals in one phase cannot happen).
    goal_events_this_phase: Vec<u8>,
    /// Last counter value reported with an instigator, per pad actor.
    pad_reported: HashMap<ActorId, u8>,
    /// Time of the last demolition reported for each victim car actor.
    demolished_at: HashMap<i32, f32>,
    /// Last `DodgesRefreshedCounter` value seen per car actor.
    dodges_refreshed: HashMap<i32, i32>,
    /// The car actor lifetime that last linked to each player actor, and the last key seen per player actor.
    player_owner: HashMap<ActorId, (ActorId, usize)>,
    last_player_keys: HashMap<i32, String>,
    diagnostics: Diagnostics,
}

fn classify(class: &str) -> ActorKind {
    if class.contains("Archetypes.Car.Car") || class.ends_with(".Car_Default") {
        ActorKind::Car
    } else if class.contains("Archetypes.Ball.Ball") || class.ends_with(".Ball_Default") {
        ActorKind::Ball
    } else if class.ends_with("Team0") {
        ActorKind::Team(0)
    } else if class.ends_with("Team1") {
        ActorKind::Team(1)
    } else if class.contains("__PRI_TA") {
        ActorKind::Player
    } else if class.contains("CarComponent_Boost") {
        ActorKind::Component(ComponentKind::Boost)
    } else if class.contains("CarComponent_DoubleJump") {
        ActorKind::Component(ComponentKind::DoubleJump)
    } else if class.contains("CarComponent_Jump") {
        ActorKind::Component(ComponentKind::Jump)
    } else if class.contains("CarComponent_Dodge") {
        ActorKind::Component(ComponentKind::Dodge)
    } else if class.contains("CarComponent_FlipCar") {
        ActorKind::Component(ComponentKind::FlipCar)
    } else if class.contains("GameEvent_Soccar") {
        ActorKind::GameEvent
    } else if class.contains("VehiclePickup_Boost") {
        let name = class
            .find("VehiclePickup_Boost")
            .map(|idx| class[idx..].to_owned())
            .unwrap_or_else(|| class.to_owned());
        ActorKind::Pad(name)
    } else {
        ActorKind::Other
    }
}

fn prop_name(replay: &Replay, id: i32) -> Option<&str> {
    usize::try_from(id)
        .ok()
        .and_then(|idx| replay.objects.get(idx))
        .map(String::as_str)
}

fn linked_actor(attribute: &Attribute) -> Option<ActorId> {
    match attribute {
        Attribute::ActiveActor(value) if value.active => Some(value.actor),
        _ => None,
    }
}

fn normalized_axis(byte: u8) -> f32 {
    if byte >= 128 {
        f32::from(byte - 128) / 127.0
    } else {
        (f32::from(byte) - 128.0) / 128.0
    }
}

fn boost_amount(raw: u8) -> f32 {
    f32::from(raw) * (100.0 / 255.0)
}

impl Tracker {
    fn delete(&mut self, id: ActorId) {
        if let Some(actor) = self.actors.remove(&id) {
            match actor.kind {
                ActorKind::Car => {
                    self.cars.remove(&id);
                    self.components.retain(|_, car| *car != id);
                    // A recycled actor id must not inherit the previous car's counter.
                    self.dodges_refreshed.remove(&id.0);
                    // Nor a demolition report of the previous car, which would mark a new one as a repeat.
                    self.demolished_at.remove(&id.0);
                }
                ActorKind::Ball => {
                    if self
                        .ball
                        .as_ref()
                        .is_some_and(|(ball_id, _)| *ball_id == id)
                    {
                        self.ball = None;
                    }
                }
                ActorKind::Player => {
                    self.players.remove(&id);
                    // Cars must not keep pointing at a player actor id that can be reused.
                    for car in self.cars.values_mut() {
                        if car.player_actor == Some(id) {
                            car.player_actor = None;
                            car.player_link_active = false;
                            self.diagnostics.players_deleted_with_cars += 1;
                        }
                    }
                    self.player_owner.remove(&id);
                }
                ActorKind::Component(_) => {
                    self.components.remove(&id);
                }
                ActorKind::Pad(_) => {
                    // A recycled actor id must not inherit the previous pad's counter.
                    self.pad_reported.remove(&id);
                }
                _ => {}
            }
        }
    }

    fn announce(&mut self, id: ActorId, class: &str, frame: usize, trajectory: &boxcars::Trajectory) {
        if let Some(existing) = self.actors.get(&id) {
            if existing.class == class {
                self.diagnostics.repeated_actor_announcements += 1;
                return;
            }
            self.diagnostics.actor_class_replacements += 1;
            self.delete(id);
        }
        let kind = classify(class);
        match kind {
            ActorKind::Car => {
                self.cars.insert(
                    id,
                    TrackedCar {
                        created_frame: frame,
                        spawn: SpawnPose::from_trajectory(trajectory, frame),
                        ..TrackedCar::default()
                    },
                );
            }
            ActorKind::Ball => {
                self.ball = Some((id, Body::default()));
            }
            ActorKind::Player => {
                self.players.insert(id, TrackedPlayer::default());
            }
            _ => {}
        }
        self.actors.insert(
            id,
            Actor {
                class: class.to_owned(),
                kind,
            },
        );
    }

    fn link(&mut self, actor: ActorId, property: &str, attribute: &Attribute, frame: usize) {
        match property {
            "Engine.Pawn:PlayerReplicationInfo" => {
                if let Some(car) = self.cars.get_mut(&actor) {
                    if let Attribute::ActiveActor(value) = attribute {
                        car.player_link_active = value.active;
                        if value.active {
                            car.player_actor = Some(value.actor);
                            let lifetime = (actor, car.created_frame);
                            let created = car.created_frame;
                            // A car lifetime that takes over a player another lifetime owned is a replacement.
                            if let Some(previous) = self.player_owner.insert(value.actor, lifetime) {
                                if previous != lifetime {
                                    self.diagnostics.replacement_cars_linked += 1;
                                    if created != frame {
                                        self.diagnostics.replacement_cars_linked_after_creation += 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            "Engine.PlayerReplicationInfo:Team" => {
                if let Some(player) = self.players.get_mut(&actor) {
                    player.team_actor = linked_actor(attribute);
                }
            }
            "TAGame.CarComponent_TA:Vehicle" => {
                if let Some(car) = linked_actor(attribute) {
                    self.components.insert(actor, car);
                } else {
                    self.components.remove(&actor);
                }
            }
            "Engine.PlayerReplicationInfo:UniqueId" => {
                if let (Some(player), Attribute::UniqueId(uid)) =
                    (self.players.get_mut(&actor), attribute)
                {
                    player.unique_id = serde_json::to_string(uid).ok();
                }
            }
            "Engine.PlayerReplicationInfo:PlayerID" => {
                if let (Some(player), Attribute::Int(id)) =
                    (self.players.get_mut(&actor), attribute)
                {
                    player.player_id = Some(*id);
                }
            }
            "Engine.PlayerReplicationInfo:PlayerName" => {
                if let (Some(player), Attribute::String(name)) =
                    (self.players.get_mut(&actor), attribute)
                {
                    player.name = Some(name.clone());
                }
            }
            _ => {}
        }
    }

    fn observe(
        &mut self,
        actor: ActorId,
        property: &str,
        attribute: &Attribute,
        names: &[String],
        frame: usize,
        events: &mut Vec<Event>,
        pad_pickups: &mut Vec<PadPickup>,
    ) {
        match property {
            "TAGame.RBActor_TA:ReplicatedRBState" => {
                if let Attribute::RigidBody(body) = attribute {
                    match self.actors.get(&actor).map(|actor| &actor.kind) {
                        Some(ActorKind::Car) => {
                            if let Some(car) = self.cars.get_mut(&actor) {
                                car.body.update(body, frame);
                            }
                        }
                        Some(ActorKind::Ball) => {
                            if let Some((id, ball)) = self.ball.as_mut() {
                                if *id == actor {
                                    ball.update(body, frame);
                                }
                            }
                        }
                        _ => self.diagnostics.unknown_actor_updates += 1,
                    }
                }
            }
            "TAGame.Vehicle_TA:ReplicatedThrottle" => {
                if let (Some(car), Attribute::Byte(raw)) = (self.cars.get_mut(&actor), attribute) {
                    car.inputs.throttle = Some(Value::replay(normalized_axis(*raw), frame));
                }
            }
            "TAGame.Vehicle_TA:ReplicatedSteer" => {
                if let (Some(car), Attribute::Byte(raw)) = (self.cars.get_mut(&actor), attribute) {
                    car.inputs.steer = Some(Value::replay(normalized_axis(*raw), frame));
                }
            }
            "TAGame.Vehicle_TA:bReplicatedHandbrake" => {
                if let (Some(car), Attribute::Boolean(value)) =
                    (self.cars.get_mut(&actor), attribute)
                {
                    car.inputs.handbrake = Some(Value::replay(*value, frame));
                }
            }
            "TAGame.CarComponent_Boost_TA:ReplicatedBoostAmount" => {
                if let (Some(car_id), Attribute::Byte(raw)) =
                    (self.components.get(&actor), attribute)
                {
                    if let Some(car) = self.cars.get_mut(car_id) {
                        car.boost_raw = Some(Value::replay(*raw, frame));
                        car.boost = Some(Value::replay(boost_amount(*raw), frame));
                    }
                }
            }
            "TAGame.CarComponent_Boost_TA:ReplicatedBoost" => {
                if let (Some(car_id), Attribute::ReplicatedBoost(value)) =
                    (self.components.get(&actor), attribute)
                {
                    if let Some(car) = self.cars.get_mut(car_id) {
                        car.boost_raw = Some(Value::replay(value.boost_amount, frame));
                        car.boost = Some(Value::replay(boost_amount(value.boost_amount), frame));
                    }
                }
            }
            "TAGame.CarComponent_TA:ReplicatedActive" => {
                if let (
                    Some(Actor {
                        kind: ActorKind::Component(component),
                        ..
                    }),
                    Some(car_id),
                    Attribute::Byte(raw),
                ) = (
                    self.actors.get(&actor),
                    self.components.get(&actor),
                    attribute,
                ) {
                    if let Some(car) = self.cars.get_mut(car_id) {
                        let field = match component {
                            ComponentKind::Boost => &mut car.inputs.boost_active_raw,
                            ComponentKind::Jump => &mut car.inputs.jump_active_raw,
                            ComponentKind::DoubleJump => &mut car.inputs.double_jump_active_raw,
                            ComponentKind::Dodge => &mut car.inputs.dodge_active_raw,
                            ComponentKind::FlipCar => &mut car.inputs.flip_car_active_raw,
                        };
                        *field = Some(Value::replay(*raw, frame));
                    }
                }
            }
            "TAGame.CarComponent_Dodge_TA:DodgeTorque" => {
                if let (Some(car_id), Attribute::Location(torque)) =
                    (self.components.get(&actor), attribute)
                {
                    if let Some(car) = self.cars.get_mut(car_id) {
                        car.inputs.dodge_torque_replay_units =
                            Some(Value::replay(vector(*torque), frame));
                    }
                }
            }
            "Engine.TeamInfo:Score" => {
                if let (
                    Some(Actor {
                        kind: ActorKind::Team(team),
                        ..
                    }),
                    Attribute::Int(score),
                ) = (self.actors.get(&actor), attribute)
                {
                    self.team_scores[usize::from(*team)] = Some(Value::replay(*score, frame));
                }
            }
            "TAGame.GameEvent_Soccar_TA:SecondsRemaining" => {
                if let Attribute::Int(value) = attribute {
                    self.seconds_remaining = Some(Value::replay(*value, frame));
                }
            }
            "TAGame.GameEvent_Soccar_TA:bOverTime" => {
                if let Attribute::Boolean(value) = attribute {
                    self.overtime = Some(Value::replay(*value, frame));
                }
            }
            "TAGame.GameEvent_TA:ReplicatedStateName" => {
                if let Attribute::Int(index) = attribute {
                    if let Some(name) = usize::try_from(*index)
                        .ok()
                        .and_then(|index| names.get(index))
                    {
                        if !matches!(name.as_str(), "PostGoalScored" | "ReplayPlayback") {
                            self.goal_events_this_phase.clear();
                        }
                        self.game_state = Some(Value::replay(name.clone(), frame));
                    }
                }
            }
            "TAGame.GameEvent_Soccar_TA:ReplicatedScoredOnTeam" => {
                if let Attribute::Byte(team @ 0..=1) = attribute {
                    if !self.goal_events_this_phase.contains(team) {
                        self.goal_events_this_phase.push(*team);
                        events.push(Event::GoalScoredOn { team: *team });
                    }
                }
            }
            "ProjectX.GRI_X:ReplicatedGameMutatorIndex"
            | "TAGame.Ball_TA:ReplicatedBallGravityScale"
            | "TAGame.Ball_TA:ReplicatedBallMaxLinearSpeedScale" => {
                let value = match attribute {
                    Attribute::Int(v) => Some(v.to_string()),
                    Attribute::Float(v) => Some(v.to_string()),
                    _ => None,
                };
                if let Some(value) = value {
                    let seen = self.diagnostics.game_settings.entry(property.to_owned()).or_default();
                    if !seen.contains(&value) {
                        seen.push(value);
                    }
                }
            }
            "TAGame.Car_TA:DodgesRefreshedCounter" => {
                if let Attribute::Int(count) = attribute {
                    let before = self.dodges_refreshed.insert(actor.0, *count);
                    if before.is_some_and(|before| *count > before) {
                        events.push(Event::DodgeRefreshed { car: actor.0, count: *count });
                    }
                }
            }
            "TAGame.Car_TA:ReplicatedDemolishExtended" => {
                if let Attribute::DemolishExtended(d) = attribute {
                    let active = |a: &boxcars::ActiveActor| a.active.then_some(a.actor.0);
                    events.push(Event::Demolish {
                        source: "extended",
                        attacker_car: active(&d.attacker),
                        victim_car: active(&d.victim),
                        attacker_pri: active(&d.attacker_pri),
                        self_demolish: d.self_demolish,
                        repeat: false,
                        attacker_velocity: [d.attacker_velocity.x, d.attacker_velocity.y, d.attacker_velocity.z],
                        victim_velocity: [d.victim_velocity.x, d.victim_velocity.y, d.victim_velocity.z],
                    });
                }
            }
            "TAGame.Car_TA:ReplicatedDemolish" => {
                if let Attribute::Demolish(d) = attribute {
                    events.push(Event::Demolish {
                        source: "plain",
                        attacker_car: d.attacker_flag.then_some(d.attacker.0),
                        victim_car: d.victim_flag.then_some(d.victim.0),
                        attacker_pri: None,
                        self_demolish: false,
                        repeat: false,
                        attacker_velocity: [d.attack_velocity.x, d.attack_velocity.y, d.attack_velocity.z],
                        victim_velocity: [d.victim_velocity.x, d.victim_velocity.y, d.victim_velocity.z],
                    });
                }
            }
            "TAGame.Car_TA:ReplicatedDemolishGoalExplosion" => {
                if let Attribute::DemolishFx(d) = attribute {
                    events.push(Event::Demolish {
                        source: "goal_explosion",
                        attacker_car: d.attacker_flag.then_some(d.attacker.0),
                        victim_car: d.victim_flag.then_some(d.victim.0),
                        attacker_pri: None,
                        self_demolish: false,
                        repeat: false,
                        attacker_velocity: [d.attack_velocity.x, d.attack_velocity.y, d.attack_velocity.z],
                        victim_velocity: [d.victim_velocity.x, d.victim_velocity.y, d.victim_velocity.z],
                    });
                }
            }
            "TAGame.VehiclePickup_TA:NewReplicatedPickupData" => {
                if let Attribute::PickupNew(pickup) = attribute {
                    let pad_actor_name = match self.actors.get(&actor).map(|a| &a.kind) {
                        Some(ActorKind::Pad(name)) => Some(name.clone()),
                        _ => None,
                    };
                    let repeat = pickup.instigator.is_some()
                        && pickup.picked_up != 255
                        && self.pad_reported.get(&actor) == Some(&pickup.picked_up);
                    if pickup.instigator.is_some() && pickup.picked_up != 255 {
                        self.pad_reported.insert(actor, pickup.picked_up);
                    }
                    pad_pickups.push(PadPickup {
                        pad_actor_id: actor.0,
                        pad_actor_name,
                        instigator_car_id: pickup.instigator.map(|id| id.0),
                        picked_up: pickup.picked_up,
                        repeat,
                    });
                }
            }
            "TAGame.PRI_TA:MatchScore" => {
                self.player_stat(actor, attribute, frame, |s| &mut s.match_score)
            }
            "TAGame.PRI_TA:ClientLoadouts" => {
                if let (Some(player), Attribute::TeamLoadout(loadouts)) =
                    (self.players.get_mut(&actor), attribute)
                {
                    player.body_product_ids = [
                        Some(Value::replay(loadouts.blue.body, frame)),
                        Some(Value::replay(loadouts.orange.body, frame)),
                    ];
                }
            }
            "TAGame.PRI_TA:MatchGoals" => {
                self.player_stat(actor, attribute, frame, |s| &mut s.goals)
            }
            "TAGame.PRI_TA:MatchAssists" => {
                self.player_stat(actor, attribute, frame, |s| &mut s.assists)
            }
            "TAGame.PRI_TA:MatchSaves" => {
                self.player_stat(actor, attribute, frame, |s| &mut s.saves)
            }
            "TAGame.PRI_TA:MatchShots" => {
                self.player_stat(actor, attribute, frame, |s| &mut s.shots)
            }
            "TAGame.PRI_TA:MatchDemolishes" => {
                self.player_stat(actor, attribute, frame, |s| &mut s.demolishes)
            }
            _ => {}
        }
    }

    fn player_stat(
        &mut self,
        actor: ActorId,
        attribute: &Attribute,
        frame: usize,
        field: impl FnOnce(&mut PlayerStats) -> &mut Option<Value<i32>>,
    ) {
        if let (Some(player), Attribute::Int(value)) = (self.players.get_mut(&actor), attribute) {
            *field(&mut player.stats) = Some(Value::replay(*value, frame));
        }
    }

    fn snapshot(
        &mut self,
        index: usize,
        time: f32,
        delta: f32,
        events: Vec<Event>,
        pad_pickups: Vec<PadPickup>,
    ) -> Frame {
        let mut events = events;
        for event in &mut events {
            if let Event::Demolish {
                source,
                victim_car: Some(victim),
                repeat,
                ..
            } = event
            {
                if *source == "goal_explosion" {
                    continue;
                }
                *repeat = self
                    .demolished_at
                    .get(victim)
                    .is_some_and(|&t| time - t < DEMOLITION_REPEAT_WINDOW);
                if !*repeat {
                    self.demolished_at.insert(*victim, time);
                }
            }
        }
        if self
            .seconds_remaining
            .as_ref()
            .is_some_and(|x| x.value == 300)
        {
            for score in &mut self.team_scores {
                if score.is_none() {
                    *score = Some(Value {
                        value: 0,
                        frame: index,
                        source: Source::InferredMatchStart,
                    });
                }
            }
        }
        let mut players: Vec<_> = self
            .players
            .iter()
            .map(|(actor, tracked)| {
                let team = tracked
                    .team_actor
                    .and_then(|id| self.actors.get(&id))
                    .and_then(|actor| match actor.kind {
                        ActorKind::Team(team) => Some(team),
                        _ => None,
                    });
                Player {
                    actor_id: actor.0,
                    key: tracked
                        .unique_id
                        .clone()
                        .or_else(|| tracked.player_id.map(|id| format!("player_id:{id}")))
                        .unwrap_or_else(|| format!("actor:{}", actor.0)),
                    name: tracked.name.clone(),
                    team,
                    body_product_ids: tracked.body_product_ids.clone(),
                    stats: tracked.stats.clone(),
                }
            })
            .collect();
        players.sort_by_key(|player| player.actor_id);
        for player in &players {
            if let Some(previous) = self.last_player_keys.insert(player.actor_id, player.key.clone()) {
                if previous != player.key {
                    self.diagnostics.player_key_changes += 1;
                }
            }
        }
        let player_lookup: HashMap<_, _> = players
            .iter()
            .map(|player| {
                (
                    ActorId(player.actor_id),
                    (
                        player.key.clone(),
                        player.team,
                        player.body_product_ids.clone(),
                    ),
                )
            })
            .collect();
        let mut cars: Vec<_> = self
            .cars
            .iter()
            .map(|(actor, tracked)| {
                let (player_key, team, body_product_id) = tracked
                    .player_actor
                    .and_then(|id| player_lookup.get(&id))
                    .map_or((None, None, None), |(key, team, ids)| {
                        (
                            Some(key.clone()),
                            *team,
                            team.and_then(|side| ids[usize::from(side)].clone()),
                        )
                    });
                Car {
                    actor_id: actor.0,
                    actor_created_frame: tracked.created_frame,
                    player_key,
                    player_link_active: tracked.player_link_active,
                    team,
                    body_product_id,
                    body: tracked.body.clone(),
                    boost: tracked.boost.clone(),
                    boost_raw: tracked.boost_raw.clone(),
                    inputs: tracked.inputs.clone(),
                    spawn_pose: tracked
                        .spawn
                        .clone()
                        .filter(|_| tracked.body.position.is_none()),
                }
            })
            .collect();
        cars.sort_by_key(|car| car.actor_id);
        self.diagnostics.unlinked_car_frames +=
            cars.iter().filter(|car| car.player_key.is_none()).count();
        Frame {
            index,
            time,
            delta,
            ball: self.ball.as_ref().map(|(_, body)| body.clone()),
            cars,
            players,
            team_scores: self.team_scores.clone(),
            seconds_remaining: self.seconds_remaining.clone(),
            overtime: self.overtime.clone(),
            game_state: self.game_state.clone(),
            events,
            pad_pickups,
        }
    }
}

fn final_score(replay: &Replay, key: &str) -> Option<i32> {
    replay.properties.iter().find_map(|(name, prop)| {
        if name == key {
            match prop {
                HeaderProp::Int(value) => Some(*value),
                _ => None,
            }
        } else {
            None
        }
    })
}

/// Extract frame-aligned observations. The input must have network frames.
pub fn extract(replay: &Replay) -> Option<ObservedReplay> {
    let frames = &replay.network_frames.as_ref()?.frames;
    let mut tracker = Tracker::default();
    let mut output = Vec::with_capacity(frames.len());
    let mut previous_time: Option<f32> = None;
    for (index, frame) in frames.iter().enumerate() {
        if previous_time.is_some_and(|previous| frame.time < previous) {
            tracker.diagnostics.non_monotonic_frame_times += 1;
        }
        previous_time = Some(frame.time);
        for actor in &frame.deleted_actors {
            tracker.delete(*actor);
        }
        for actor in &frame.new_actors {
            if let Some(class) = prop_name(replay, actor.object_id.0) {
                tracker.announce(actor.actor_id, class, index, &actor.initial_trajectory);
            }
        }
        for update in &frame.updated_actors {
            if let Some(property) = prop_name(replay, update.object_id.0) {
                tracker.link(update.actor_id, property, &update.attribute, index);
            }
        }
        let mut events = Vec::new();
        let mut pad_pickups = Vec::new();
        for update in &frame.updated_actors {
            if let Some(property) = prop_name(replay, update.object_id.0) {
                tracker.observe(
                    update.actor_id,
                    property,
                    &update.attribute,
                    &replay.names,
                    index,
                    &mut events,
                    &mut pad_pickups,
                );
            }
        }
        output.push(tracker.snapshot(index, frame.time, frame.delta, events, pad_pickups));
    }
    tracker.diagnostics.map_name = replay.properties.iter().find_map(|(name, prop)| match (name.as_str(), prop) {
        ("MapName", HeaderProp::Name(map) | HeaderProp::Str(map)) => Some(map.clone()),
        _ => None,
    });
    tracker.diagnostics.nonstandard_notes =
        nonstandard_notes(tracker.diagnostics.map_name.as_deref(), &tracker.diagnostics.game_settings);
    Some(ObservedReplay {
        header: Header {
            game_type: replay.game_type.clone(),
            levels: replay.levels.clone(),
            final_team_scores: [
                final_score(replay, "Team0Score"),
                final_score(replay, "Team1Score"),
            ],
        },
        frames: output,
        diagnostics: tracker.diagnostics,
    })
}

#[cfg(test)]
mod event_tests {
    use super::*;

    fn demolish(victim: i32) -> Event {
        Event::Demolish {
            source: "extended",
            attacker_car: Some(1),
            victim_car: Some(victim),
            attacker_pri: None,
            self_demolish: false,
            attacker_velocity: [0.0; 3],
            victim_velocity: [0.0; 3],
            repeat: false,
        }
    }

    /// Only an increase over a value already seen is a flip reset: the first value, a value re-sent
    /// unchanged every ten seconds, and a recycled actor id are not.
    #[test]
    fn dodge_refresh_counter_increases_are_events() {
        let mut tracker = Tracker::default();
        let observe = |tracker: &mut Tracker, actor: i32, value: i32| {
            let mut events = Vec::new();
            tracker.observe(
                ActorId(actor),
                "TAGame.Car_TA:DodgesRefreshedCounter",
                &Attribute::Int(value),
                &[],
                0,
                &mut events,
                &mut Vec::new(),
            );
            events
        };
        assert!(observe(&mut tracker, 5, 0).is_empty());
        assert!(observe(&mut tracker, 5, 0).is_empty());
        let events = observe(&mut tracker, 5, 1);
        assert!(matches!(events[..], [Event::DodgeRefreshed { car: 5, count: 1 }]));
        assert!(observe(&mut tracker, 5, 1).is_empty());
        let events = observe(&mut tracker, 5, 2);
        assert!(matches!(events[..], [Event::DodgeRefreshed { car: 5, count: 2 }]));
        // A car first seen with a nonzero total has no event for it (when it happened is unknown).
        assert!(observe(&mut tracker, 6, 3).is_empty());
        // After the actor is deleted its id starts over.
        tracker.actors.insert(
            ActorId(5),
            Actor { class: "Car".to_owned(), kind: ActorKind::Car },
        );
        tracker.delete(ActorId(5));
        assert!(observe(&mut tracker, 5, 1).is_empty());
    }

    /// The spawn trajectory gives the car's location and, when it only turns the car about the vertical axis
    /// (the field boxcars calls `pitch`, 1/256 turns), a heading: 64 is a quarter turn (kickoff data of
    /// train replays); a trajectory that also tilts the car keeps its location only.
    #[test]
    fn a_spawn_trajectory_gives_a_location_and_a_heading() {
        let trajectory = |yaw, pitch, roll| boxcars::Trajectory {
            location: Some(boxcars::Vector3i { x: 256, y: -3840, z: 36 }),
            rotation: Some(boxcars::Rotation { yaw, pitch, roll }),
        };
        let spawn = SpawnPose::from_trajectory(&trajectory(Some(-1), Some(64), None), 7).unwrap();
        assert_eq!(spawn.position, [256.0, -3840.0, 36.0]);
        let [x, y, z, w] = spawn.rotation_xyzw.unwrap();
        assert!(x == 0.0 && y == 0.0 && (z - 0.707_106_8).abs() < 1e-5 && (w - 0.707_106_8).abs() < 1e-5);
        assert!(SpawnPose::from_trajectory(&trajectory(Some(13), Some(90), Some(75)), 7).unwrap().rotation_xyzw.is_none());
        let no_location = boxcars::Trajectory { location: None, rotation: None };
        assert!(SpawnPose::from_trajectory(&no_location, 7).is_none());
    }

    /// A car taking over a player another car lifetime owned is a replacement (late when linked after its
    /// creation); deleting a player actor clears the link of the cars that still point at it; non-standard
    /// settings and unknown maps are reported.
    #[test]
    fn link_changes_and_settings_are_reported() {
        let announce = |tracker: &mut Tracker, id: i32, class: &str, frame: usize| {
            tracker.announce(ActorId(id), class, frame, &boxcars::Trajectory { location: None, rotation: None });
        };
        let link = |tracker: &mut Tracker, car: i32, player: i32, frame: usize| {
            tracker.link(
                ActorId(car),
                "Engine.Pawn:PlayerReplicationInfo",
                &Attribute::ActiveActor(boxcars::ActiveActor { active: true, actor: ActorId(player) }),
                frame,
            );
        };
        let mut tracker = Tracker::default();
        announce(&mut tracker, 5, "TAGame.Default__PRI_TA", 0);
        announce(&mut tracker, 1, "Archetypes.Car.Car_Default", 0);
        link(&mut tracker, 1, 5, 0);
        assert_eq!(tracker.diagnostics.replacement_cars_linked, 0);
        announce(&mut tracker, 2, "Archetypes.Car.Car_Default", 10);
        link(&mut tracker, 2, 5, 12);
        assert_eq!(tracker.diagnostics.replacement_cars_linked, 1);
        assert_eq!(tracker.diagnostics.replacement_cars_linked_after_creation, 1);
        tracker.delete(ActorId(5));
        assert_eq!(tracker.diagnostics.players_deleted_with_cars, 2);
        assert!(tracker.cars.values().all(|car| car.player_actor.is_none() && !car.player_link_active));

        let mut settings = BTreeMap::new();
        settings.insert("ProjectX.GRI_X:ReplicatedGameMutatorIndex".to_owned(), vec!["-1".to_owned()]);
        settings.insert("TAGame.Ball_TA:ReplicatedBallGravityScale".to_owned(), vec!["1".to_owned(), "0.5".to_owned()]);
        let notes = nonstandard_notes(Some("Stadium_P"), &settings);
        assert_eq!(notes.len(), 1, "{notes:?}");
        assert!(notes[0].contains("BallGravityScale = 0.5"));
        assert!(nonstandard_notes(Some("SomeNewMap_P"), &BTreeMap::new())[0].contains("SomeNewMap_P"));
        assert_eq!(nonstandard_notes(Some("CS_P"), &BTreeMap::new()), Vec::<String>::new());
        settings.clear();
        settings.insert("ProjectX.GRI_X:ReplicatedGameMutatorIndex".to_owned(), vec!["3".to_owned()]);
        assert_eq!(nonstandard_notes(Some("Stadium_P"), &settings).len(), 1);
    }

    fn repeat_of(frame: &Frame) -> bool {
        match &frame.events[0] {
            Event::Demolish { repeat, .. } => *repeat,
            _ => unreachable!(),
        }
    }

    /// A car cannot be demolished again within the 3 s it spends demolished: a report of the same victim
    /// 1.7 s later is a repeat, one 4.4 s later a new demolition, and another victim is unaffected.
    #[test]
    fn a_demolition_reported_again_within_the_respawn_time_is_a_repeat() {
        let mut tracker = Tracker::default();
        let first = tracker.snapshot(0, 10.0, 0.03, vec![demolish(7)], Vec::new());
        assert!(!repeat_of(&first));
        let again = tracker.snapshot(1, 11.7, 0.03, vec![demolish(7)], Vec::new());
        assert!(repeat_of(&again));
        let other = tracker.snapshot(2, 11.8, 0.03, vec![demolish(8)], Vec::new());
        assert!(!repeat_of(&other));
        let later = tracker.snapshot(3, 16.1, 0.03, vec![demolish(7)], Vec::new());
        assert!(!repeat_of(&later));
    }

    /// A new car on a recycled actor id is not the victim of the previous car's demolition: a report
    /// 1.7 s after the old car's, with the actor deleted in between, is a new demolition.
    #[test]
    fn a_recycled_car_actor_id_does_not_inherit_the_demolition_window() {
        let mut tracker = Tracker::default();
        let first = tracker.snapshot(0, 10.0, 0.03, vec![demolish(7)], Vec::new());
        assert!(!repeat_of(&first));
        tracker.actors.insert(
            ActorId(7),
            Actor { class: "Car".to_owned(), kind: ActorKind::Car },
        );
        tracker.delete(ActorId(7));
        let recycled = tracker.snapshot(1, 11.7, 0.03, vec![demolish(7)], Vec::new());
        assert!(!repeat_of(&recycled));
    }

    /// A pad actor id that is deleted and recycled starts without the previous pad's pickup counter.
    #[test]
    fn a_recycled_pad_actor_id_does_not_inherit_the_pickup_counter() {
        let mut tracker = Tracker::default();
        let pickup = |tracker: &mut Tracker| {
            let mut pickups = Vec::new();
            tracker.observe(
                ActorId(9),
                "TAGame.VehiclePickup_TA:NewReplicatedPickupData",
                &Attribute::PickupNew(boxcars::PickupNew { instigator: Some(ActorId(3)), picked_up: 1 }),
                &[],
                0,
                &mut Vec::new(),
                &mut pickups,
            );
            pickups[0].repeat
        };
        assert!(!pickup(&mut tracker));
        assert!(pickup(&mut tracker));
        tracker.actors.insert(
            ActorId(9),
            Actor { class: "Pad".to_owned(), kind: ActorKind::Pad("Pad".to_owned()) },
        );
        tracker.delete(ActorId(9));
        assert!(!pickup(&mut tracker));
    }
}
