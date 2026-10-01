//! Typed replay observations with actor identity and per-field freshness.

use std::collections::HashMap;

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
    },
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
                }
                ActorKind::Component(_) => {
                    self.components.remove(&id);
                }
                _ => {}
            }
        }
    }

    fn announce(&mut self, id: ActorId, class: &str, frame: usize) {
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

    fn link(&mut self, actor: ActorId, property: &str, attribute: &Attribute) {
        match property {
            "Engine.Pawn:PlayerReplicationInfo" => {
                if let Some(car) = self.cars.get_mut(&actor) {
                    if let Attribute::ActiveActor(value) = attribute {
                        car.player_link_active = value.active;
                        if value.active {
                            car.player_actor = Some(value.actor);
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
            "TAGame.Car_TA:ReplicatedDemolishExtended" => {
                if let Attribute::DemolishExtended(d) = attribute {
                    let active = |a: &boxcars::ActiveActor| a.active.then_some(a.actor.0);
                    events.push(Event::Demolish {
                        source: "extended",
                        attacker_car: active(&d.attacker),
                        victim_car: active(&d.victim),
                        attacker_pri: active(&d.attacker_pri),
                        self_demolish: d.self_demolish,
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
    for (index, frame) in frames.iter().enumerate() {
        for actor in &frame.deleted_actors {
            tracker.delete(*actor);
        }
        for actor in &frame.new_actors {
            if let Some(class) = prop_name(replay, actor.object_id.0) {
                tracker.announce(actor.actor_id, class, index);
            }
        }
        for update in &frame.updated_actors {
            if let Some(property) = prop_name(replay, update.object_id.0) {
                tracker.link(update.actor_id, property, &update.attribute);
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
