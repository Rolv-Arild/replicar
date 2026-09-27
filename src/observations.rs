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
    /// Replay values are degrees per second; RocketSim uses radians per second.
    pub angular_velocity_deg: Option<Value<[f32; 3]>>,
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
            self.angular_velocity_deg = Some(Value::replay(vector(velocity), frame));
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
    /// Sequence/counter from boost component; its bit semantics remain to be measured.
    pub boost_active_raw: Option<Value<u8>>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Car {
    pub actor_id: i32,
    pub player_key: Option<String>,
    pub team: Option<u8>,
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
    pub stats: PlayerStats,
}

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Event {
    GoalScoredOn { team: u8 },
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
    BoostComponent,
    GameEvent,
    Other,
}

#[derive(Clone)]
struct Actor {
    class: String,
    kind: ActorKind,
}

#[derive(Clone, Default)]
struct TrackedCar {
    body: Body,
    player_actor: Option<ActorId>,
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
        ActorKind::BoostComponent
    } else if class.contains("GameEvent_Soccar") {
        ActorKind::GameEvent
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
                ActorKind::BoostComponent => {
                    self.components.remove(&id);
                }
                _ => {}
            }
        }
    }

    fn announce(&mut self, id: ActorId, class: &str) {
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
                self.cars.insert(id, TrackedCar::default());
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
                    car.player_actor = linked_actor(attribute);
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
        frame: usize,
        events: &mut Vec<Event>,
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
                        kind: ActorKind::BoostComponent,
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
                        car.inputs.boost_active_raw = Some(Value::replay(*raw, frame));
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
                if let Attribute::String(value) = attribute {
                    self.game_state = Some(Value::replay(value.clone(), frame));
                }
            }
            "TAGame.GameEvent_Soccar_TA:ReplicatedScoredOnTeam" => {
                if let Attribute::Byte(team @ 0..=1) = attribute {
                    events.push(Event::GoalScoredOn { team: *team });
                }
            }
            "TAGame.PRI_TA:MatchScore" => {
                self.player_stat(actor, attribute, frame, |s| &mut s.match_score)
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

    fn snapshot(&mut self, index: usize, time: f32, delta: f32, events: Vec<Event>) -> Frame {
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
                    stats: tracked.stats.clone(),
                }
            })
            .collect();
        players.sort_by_key(|player| player.actor_id);
        let player_lookup: HashMap<_, _> = players
            .iter()
            .map(|player| (ActorId(player.actor_id), (player.key.clone(), player.team)))
            .collect();
        let mut cars: Vec<_> = self
            .cars
            .iter()
            .map(|(actor, tracked)| {
                let (player_key, team) = tracked
                    .player_actor
                    .and_then(|id| player_lookup.get(&id))
                    .map_or((None, None), |(key, team)| (Some(key.clone()), *team));
                Car {
                    actor_id: actor.0,
                    player_key,
                    team,
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
                tracker.announce(actor.actor_id, class);
            }
        }
        for update in &frame.updated_actors {
            if let Some(property) = prop_name(replay, update.object_id.0) {
                tracker.link(update.actor_id, property, &update.attribute);
            }
        }
        let mut events = Vec::new();
        for update in &frame.updated_actors {
            if let Some(property) = prop_name(replay, update.object_id.0) {
                tracker.observe(
                    update.actor_id,
                    property,
                    &update.attribute,
                    index,
                    &mut events,
                );
            }
        }
        output.push(tracker.snapshot(index, frame.time, frame.delta, events));
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
