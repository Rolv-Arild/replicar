//! The actor tracker: applies a frame's deletions, creations and updates to the actors it knows and takes
//! a snapshot of what they say.

use std::collections::{BTreeMap, HashMap};

use boxcars::{Attribute, RigidBody, Trajectory, Vector3f};
use replicar_format::{FrameIndex, Team};

use super::attributes::*;
use super::network::*;

/// Seconds within which a second report of one victim is a repeat: a car cannot be demolished again while
/// it is demolished (3 s), and the replay re-sends some demolitions 200-530 ticks later.
pub const DEMOLITION_REPEAT_WINDOW: f32 = 5.0;

#[derive(Clone)]
enum ActorKind {
    Car,
    Ball,
    Player,
    Team(Team),
    Component(Component),
    GameEvent,
    Pad(String),
    Other,
}

#[derive(Clone, Copy)]
enum Component {
    Boost,
    Jump,
    DoubleJump,
    Dodge,
    FlipCar,
}

struct Actor {
    class: String,
    kind: ActorKind,
}

#[derive(Default)]
struct TrackedCar {
    created: FrameIndex,
    spawn: Option<SpawnPose>,
    body: NetworkBody,
    player_actor: Option<ActorId>,
    player_link_active: bool,
    boost: Option<NetworkValue<f32>>,
    boost_raw: Option<NetworkValue<u8>>,
    inputs: NetworkInputs,
}

#[derive(Default)]
struct TrackedPlayer {
    unique_id: Option<String>,
    player_id: Option<i32>,
    name: Option<String>,
    team_actor: Option<ActorId>,
    body_product_ids: [Option<NetworkValue<u32>>; 2],
    stats: PlayerStats,
    ping_raw: Option<NetworkValue<u8>>,
}

/// What a frame's updates report besides values: events and pad records.
#[derive(Default)]
pub(super) struct FrameReports {
    pub(super) events: Vec<NetworkEvent>,
    pub(super) pad_records: Vec<PadRecord>,
}

#[derive(Default)]
pub(super) struct Tracker {
    actors: HashMap<ActorId, Actor>,
    cars: HashMap<ActorId, TrackedCar>,
    players: HashMap<ActorId, TrackedPlayer>,
    /// Component actor to the car it belongs to.
    components: HashMap<ActorId, ActorId>,
    ball: Option<(ActorId, NetworkBody)>,
    team_scores: [Option<NetworkValue<i32>>; 2],
    seconds_remaining: Option<NetworkValue<i32>>,
    overtime: Option<NetworkValue<bool>>,
    game_state: Option<NetworkValue<GameState>>,
    /// Teams already reported scored on in the current goal pause: the replay sometimes sends the report
    /// again 100-150 ticks later, and two goals in one pause cannot happen.
    scored_on_this_pause: Vec<Team>,
    /// The last counter value reported with an instigator, per pad actor.
    pad_reported: HashMap<ActorId, u8>,
    /// The time of the last demolition reported for each victim car actor.
    demolished_at: HashMap<ActorId, f32>,
    /// The last `DodgesRefreshedCounter` value per car actor.
    dodges_refreshed: HashMap<ActorId, i32>,
    /// The car life that last linked to each player actor, and the last key per player actor.
    player_owner: HashMap<ActorId, CarLife>,
    last_player_keys: HashMap<ActorId, PlayerKey>,
    pub(super) diagnostics: DecodeDiagnostics,
}

fn classify(class: &str) -> ActorKind {
    if class.contains("Archetypes.Car.Car") || class.ends_with(".Car_Default") {
        ActorKind::Car
    } else if class.contains("Archetypes.Ball.Ball") || class.ends_with(".Ball_Default") {
        ActorKind::Ball
    } else if class.ends_with("Team0") {
        ActorKind::Team(Team::Blue)
    } else if class.ends_with("Team1") {
        ActorKind::Team(Team::Orange)
    } else if class.contains("__PRI_TA") {
        ActorKind::Player
    } else if class.contains("CarComponent_Boost") {
        ActorKind::Component(Component::Boost)
    } else if class.contains("CarComponent_DoubleJump") {
        ActorKind::Component(Component::DoubleJump)
    } else if class.contains("CarComponent_Jump") {
        ActorKind::Component(Component::Jump)
    } else if class.contains("CarComponent_Dodge") {
        ActorKind::Component(Component::Dodge)
    } else if class.contains("CarComponent_FlipCar") {
        ActorKind::Component(Component::FlipCar)
    } else if class.contains("GameEvent_Soccar") {
        ActorKind::GameEvent
    } else if let Some(at) = class.find("VehiclePickup_Boost") {
        ActorKind::Pad(class[at..].to_owned())
    } else {
        ActorKind::Other
    }
}

fn vector(v: Vector3f) -> [f32; 3] {
    [v.x, v.y, v.z]
}

/// A replicated control byte as -1..1: 128 is 0, 255 is 1, 0 is -1.
fn control_axis(byte: u8) -> f32 {
    if byte >= 128 {
        f32::from(byte - 128) / 127.0
    } else {
        (f32::from(byte) - 128.0) / 128.0
    }
}

/// A replicated boost byte as 0-100.
fn boost_amount(raw: u8) -> f32 {
    f32::from(raw) * (100.0 / 255.0)
}

/// The actor an attribute links to, when the link is active.
fn linked_actor(attribute: &Attribute) -> Option<ActorId> {
    match attribute {
        Attribute::ActiveActor(link) if link.active => Some(ActorId(link.actor.0)),
        _ => None,
    }
}

fn actor_id(id: boxcars::ActorId) -> ActorId {
    ActorId(id.0)
}

impl NetworkBody {
    fn update(&mut self, body: &RigidBody, frame: FrameIndex) {
        self.position = Some(NetworkValue::replay(vector(body.location), frame));
        let r = body.rotation;
        self.rotation = Some(NetworkValue::replay([r.x, r.y, r.z, r.w], frame));
        self.sleeping = Some(NetworkValue::replay(body.sleeping, frame));
        if let Some(velocity) = body.linear_velocity {
            self.linear_velocity = Some(NetworkValue::replay(vector(velocity), frame));
        }
        if let Some(velocity) = body.angular_velocity {
            self.angular_velocity_raw = Some(NetworkValue::replay(vector(velocity), frame));
        }
    }
}

impl SpawnPose {
    /// The spawn of a car actor from its creation trajectory. The heading is boxcars' `pitch` field (in
    /// 1/256 turns); a trajectory that also yaws or rolls by more than one step keeps its location only.
    fn from_trajectory(trajectory: &Trajectory, frame: FrameIndex) -> Option<Self> {
        let location = trajectory.location?;
        let rotation = trajectory.rotation.as_ref().and_then(|r| {
            let level = |v: Option<i8>| v.is_none_or(|v| v.abs() <= 1);
            (level(r.yaw) && level(r.roll)).then(|| {
                let angle = f32::from(r.pitch.unwrap_or(0)) * std::f32::consts::PI / 128.0;
                [0.0, 0.0, (angle / 2.0).sin(), (angle / 2.0).cos()]
            })
        });
        Some(Self {
            position: [location.x as f32, location.y as f32, location.z as f32],
            rotation,
            frame,
        })
    }
}

impl Tracker {
    pub(super) fn delete(&mut self, id: ActorId) {
        let Some(actor) = self.actors.remove(&id) else {
            return;
        };
        match actor.kind {
            ActorKind::Car => {
                self.cars.remove(&id);
                self.components.retain(|_, car| *car != id);
                // A recycled actor id must inherit neither the old car's counter nor its demolition.
                self.dodges_refreshed.remove(&id);
                self.demolished_at.remove(&id);
            }
            ActorKind::Ball => {
                if self.ball.as_ref().is_some_and(|(ball, _)| *ball == id) {
                    self.ball = None;
                }
            }
            ActorKind::Player => {
                self.players.remove(&id);
                // Cars must not keep linking to a player actor id that can be reused.
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
                // A recycled actor id must not inherit the old pad's counter.
                self.pad_reported.remove(&id);
            }
            ActorKind::Team(_) | ActorKind::GameEvent | ActorKind::Other => {}
        }
    }

    pub(super) fn announce(
        &mut self,
        id: ActorId,
        class: &str,
        frame: FrameIndex,
        trajectory: &Trajectory,
    ) {
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
                        created: frame,
                        spawn: SpawnPose::from_trajectory(trajectory, frame),
                        ..TrackedCar::default()
                    },
                );
            }
            ActorKind::Ball => self.ball = Some((id, NetworkBody::default())),
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

    /// The first pass over a frame's updates: links between actors and player identity, so that the second
    /// pass finds every car's player and every component's car.
    pub(super) fn link(
        &mut self,
        actor: ActorId,
        property: &str,
        attribute: &Attribute,
        frame: FrameIndex,
    ) {
        match property {
            PAWN_PLAYER => {
                if let (Some(car), Attribute::ActiveActor(link)) =
                    (self.cars.get_mut(&actor), attribute)
                {
                    car.player_link_active = link.active;
                    if link.active {
                        let player = actor_id(link.actor);
                        car.player_actor = Some(player);
                        let life = CarLife {
                            actor,
                            created: car.created,
                        };
                        // A car life that takes over a player another car life had is a replacement.
                        if let Some(previous) = self.player_owner.insert(player, life)
                            && previous != life
                        {
                            self.diagnostics.replacement_cars_linked += 1;
                            if car.created != frame {
                                self.diagnostics.replacement_cars_linked_after_creation += 1;
                            }
                        }
                    }
                }
            }
            PLAYER_TEAM => {
                if let Some(player) = self.players.get_mut(&actor) {
                    player.team_actor = linked_actor(attribute);
                }
            }
            COMPONENT_VEHICLE => match linked_actor(attribute) {
                Some(car) => {
                    self.components.insert(actor, car);
                }
                None => {
                    self.components.remove(&actor);
                }
            },
            PLAYER_UNIQUE_ID => {
                if let (Some(player), Attribute::UniqueId(id)) =
                    (self.players.get_mut(&actor), attribute)
                {
                    player.unique_id = serde_json::to_string(id).ok();
                }
            }
            PLAYER_ID => {
                if let (Some(player), Attribute::Int(id)) =
                    (self.players.get_mut(&actor), attribute)
                {
                    player.player_id = Some(*id);
                }
            }
            PLAYER_NAME => {
                if let (Some(player), Attribute::String(name)) =
                    (self.players.get_mut(&actor), attribute)
                {
                    player.name = Some(name.clone());
                }
            }
            _ => {}
        }
    }

    /// The second pass over a frame's updates: the values themselves.
    pub(super) fn observe(
        &mut self,
        actor: ActorId,
        property: &str,
        attribute: &Attribute,
        state_names: &[String],
        frame: FrameIndex,
        reports: &mut FrameReports,
    ) {
        let FrameReports {
            events,
            pad_records,
        } = reports;
        match property {
            RIGID_BODY => {
                if let Attribute::RigidBody(body) = attribute {
                    match self.actors.get(&actor).map(|a| &a.kind) {
                        Some(ActorKind::Car) => {
                            if let Some(car) = self.cars.get_mut(&actor) {
                                car.body.update(body, frame);
                            }
                        }
                        Some(ActorKind::Ball) => {
                            if let Some((ball, state)) = self.ball.as_mut()
                                && *ball == actor
                            {
                                state.update(body, frame);
                            }
                        }
                        _ => self.diagnostics.unknown_actor_updates += 1,
                    }
                }
            }
            THROTTLE => {
                if let (Some(car), Attribute::Byte(raw)) = (self.cars.get_mut(&actor), attribute) {
                    car.inputs.throttle = Some(NetworkValue::replay(control_axis(*raw), frame));
                }
            }
            STEER => {
                if let (Some(car), Attribute::Byte(raw)) = (self.cars.get_mut(&actor), attribute) {
                    car.inputs.steer = Some(NetworkValue::replay(control_axis(*raw), frame));
                }
            }
            HANDBRAKE => {
                if let (Some(car), Attribute::Boolean(on)) = (self.cars.get_mut(&actor), attribute)
                {
                    car.inputs.handbrake = Some(NetworkValue::replay(*on, frame));
                }
            }
            BOOST_AMOUNT => {
                if let (Some(car), Attribute::Byte(raw)) = (self.component_car(actor), attribute) {
                    car.boost_raw = Some(NetworkValue::replay(*raw, frame));
                    car.boost = Some(NetworkValue::replay(boost_amount(*raw), frame));
                }
            }
            BOOST => {
                if let (Some(car), Attribute::ReplicatedBoost(boost)) =
                    (self.component_car(actor), attribute)
                {
                    car.boost_raw = Some(NetworkValue::replay(boost.boost_amount, frame));
                    car.boost = Some(NetworkValue::replay(
                        boost_amount(boost.boost_amount),
                        frame,
                    ));
                }
            }
            COMPONENT_ACTIVE => {
                let component = match self.actors.get(&actor).map(|a| &a.kind) {
                    Some(ActorKind::Component(component)) => *component,
                    _ => return,
                };
                if let (Some(car), Attribute::Byte(raw)) = (self.component_car(actor), attribute) {
                    let counter = match component {
                        Component::Boost => &mut car.inputs.boost_active_raw,
                        Component::Jump => &mut car.inputs.jump_active_raw,
                        Component::DoubleJump => &mut car.inputs.double_jump_active_raw,
                        Component::Dodge => &mut car.inputs.dodge_active_raw,
                        Component::FlipCar => &mut car.inputs.flip_car_active_raw,
                    };
                    *counter = Some(NetworkValue::replay(*raw, frame));
                }
            }
            DODGE_TORQUE => {
                if let (Some(car), Attribute::Location(torque)) =
                    (self.component_car(actor), attribute)
                {
                    car.inputs.dodge_torque_raw =
                        Some(NetworkValue::replay(vector(*torque), frame));
                }
            }
            TEAM_SCORE => {
                if let (
                    Some(Actor {
                        kind: ActorKind::Team(team),
                        ..
                    }),
                    Attribute::Int(score),
                ) = (self.actors.get(&actor), attribute)
                {
                    self.team_scores[usize::from(team.number())] =
                        Some(NetworkValue::replay(*score, frame));
                }
            }
            SECONDS_REMAINING => {
                if let Attribute::Int(seconds) = attribute {
                    self.seconds_remaining = Some(NetworkValue::replay(*seconds, frame));
                }
            }
            OVERTIME => {
                if let Attribute::Boolean(overtime) = attribute {
                    self.overtime = Some(NetworkValue::replay(*overtime, frame));
                }
            }
            GAME_STATE => {
                if let Attribute::Int(index) = attribute
                    && let Some(name) = usize::try_from(*index)
                        .ok()
                        .and_then(|i| state_names.get(i))
                {
                    let state = GameState::from_name(name);
                    if !matches!(state, GameState::PostGoalScored | GameState::ReplayPlayback) {
                        self.scored_on_this_pause.clear();
                    }
                    self.game_state = Some(NetworkValue::replay(state, frame));
                }
            }
            SCORED_ON_TEAM => {
                if let Attribute::Byte(number) = attribute
                    && let Some(team) = Team::from_number(*number)
                    && !self.scored_on_this_pause.contains(&team)
                {
                    self.scored_on_this_pause.push(team);
                    events.push(NetworkEvent::GoalScoredOn { team });
                }
            }
            MUTATOR_INDEX | BALL_GRAVITY_SCALE | BALL_MAX_SPEED_SCALE => {
                let value = match attribute {
                    Attribute::Int(v) => Some(v.to_string()),
                    Attribute::Float(v) => Some(v.to_string()),
                    _ => None,
                };
                if let Some(value) = value {
                    let seen = self
                        .diagnostics
                        .game_settings
                        .entry(property.to_owned())
                        .or_default();
                    if !seen.contains(&value) {
                        seen.push(value);
                    }
                }
            }
            DODGES_REFRESHED => {
                if let Attribute::Int(count) = attribute {
                    let before = self.dodges_refreshed.insert(actor, *count);
                    if before.is_some_and(|before| *count > before) {
                        events.push(NetworkEvent::FlipReset {
                            car: actor,
                            count: *count,
                        });
                    }
                }
            }
            DEMOLISH_EXTENDED => {
                if let Attribute::DemolishExtended(d) = attribute {
                    let active = |a: &boxcars::ActiveActor| a.active.then_some(actor_id(a.actor));
                    events.push(NetworkEvent::Demolition {
                        report: DemolitionReport::Extended,
                        attacker_car: active(&d.attacker),
                        victim_car: active(&d.victim),
                        attacker_player: active(&d.attacker_pri),
                        self_demolition: d.self_demolish,
                        attacker_velocity_raw: vector(d.attacker_velocity),
                        victim_velocity_raw: vector(d.victim_velocity),
                        repeat: false,
                    });
                }
            }
            DEMOLISH => {
                if let Attribute::Demolish(d) = attribute {
                    events.push(NetworkEvent::Demolition {
                        report: DemolitionReport::Plain,
                        attacker_car: d.attacker_flag.then_some(actor_id(d.attacker)),
                        victim_car: d.victim_flag.then_some(actor_id(d.victim)),
                        attacker_player: None,
                        self_demolition: false,
                        attacker_velocity_raw: vector(d.attack_velocity),
                        victim_velocity_raw: vector(d.victim_velocity),
                        repeat: false,
                    });
                }
            }
            DEMOLISH_GOAL_EXPLOSION => {
                if let Attribute::DemolishFx(d) = attribute {
                    events.push(NetworkEvent::Demolition {
                        report: DemolitionReport::GoalExplosion,
                        attacker_car: d.attacker_flag.then_some(actor_id(d.attacker)),
                        victim_car: d.victim_flag.then_some(actor_id(d.victim)),
                        attacker_player: None,
                        self_demolition: false,
                        attacker_velocity_raw: vector(d.attack_velocity),
                        victim_velocity_raw: vector(d.victim_velocity),
                        repeat: false,
                    });
                }
            }
            PAD_PICKUP => {
                if let Attribute::PickupNew(pickup) = attribute {
                    let pad_name = match self.actors.get(&actor).map(|a| &a.kind) {
                        Some(ActorKind::Pad(name)) => Some(name.clone()),
                        _ => None,
                    };
                    let counts = pickup.instigator.is_some() && pickup.picked_up != 255;
                    let repeat = counts && self.pad_reported.get(&actor) == Some(&pickup.picked_up);
                    if counts {
                        self.pad_reported.insert(actor, pickup.picked_up);
                    }
                    pad_records.push(PadRecord {
                        pad: actor,
                        pad_name,
                        instigator_car: pickup.instigator.map(actor_id),
                        picked_up_raw: pickup.picked_up,
                        repeat,
                    });
                }
            }
            PLAYER_PING => {
                if let (Some(player), Attribute::Byte(ping)) =
                    (self.players.get_mut(&actor), attribute)
                {
                    player.ping_raw = Some(NetworkValue::replay(*ping, frame));
                }
            }
            PLAYER_LOADOUTS => {
                if let (Some(player), Attribute::TeamLoadout(loadouts)) =
                    (self.players.get_mut(&actor), attribute)
                {
                    player.body_product_ids = [
                        Some(NetworkValue::replay(loadouts.blue.body, frame)),
                        Some(NetworkValue::replay(loadouts.orange.body, frame)),
                    ];
                }
            }
            PLAYER_MATCH_SCORE => self.player_stat(actor, attribute, frame, |s| &mut s.match_score),
            PLAYER_GOALS => self.player_stat(actor, attribute, frame, |s| &mut s.goals),
            PLAYER_ASSISTS => self.player_stat(actor, attribute, frame, |s| &mut s.assists),
            PLAYER_SAVES => self.player_stat(actor, attribute, frame, |s| &mut s.saves),
            PLAYER_SHOTS => self.player_stat(actor, attribute, frame, |s| &mut s.shots),
            PLAYER_DEMOLITIONS => self.player_stat(actor, attribute, frame, |s| &mut s.demolitions),
            _ => {}
        }
    }

    /// The car a component actor belongs to.
    fn component_car(&mut self, component: ActorId) -> Option<&mut TrackedCar> {
        let car = *self.components.get(&component)?;
        self.cars.get_mut(&car)
    }

    fn player_stat(
        &mut self,
        actor: ActorId,
        attribute: &Attribute,
        frame: FrameIndex,
        field: impl FnOnce(&mut PlayerStats) -> &mut Option<NetworkValue<i32>>,
    ) {
        if let (Some(player), Attribute::Int(value)) = (self.players.get_mut(&actor), attribute) {
            *field(&mut player.stats) = Some(NetworkValue::replay(*value, frame));
        }
    }

    /// What the actors say at the end of a frame.
    pub(super) fn snapshot(
        &mut self,
        index: FrameIndex,
        time: f32,
        delta: f32,
        reports: FrameReports,
    ) -> NetworkFrame {
        let FrameReports {
            mut events,
            pad_records,
        } = reports;
        for event in &mut events {
            if let NetworkEvent::Demolition {
                report,
                victim_car: Some(victim),
                repeat,
                ..
            } = event
            {
                if *report == DemolitionReport::GoalExplosion {
                    continue;
                }
                *repeat = self
                    .demolished_at
                    .get(victim)
                    .is_some_and(|&at| time - at < DEMOLITION_REPEAT_WINDOW);
                if !*repeat {
                    self.demolished_at.insert(*victim, time);
                }
            }
        }
        // A full clock means the match has not started: a score the replay has not sent yet is 0.
        if self
            .seconds_remaining
            .as_ref()
            .is_some_and(|clock| clock.value == 300)
        {
            for score in &mut self.team_scores {
                if score.is_none() {
                    *score = Some(NetworkValue {
                        value: 0,
                        frame: index,
                        source: ValueSource::InferredMatchStart,
                    });
                }
            }
        }
        let mut players: Vec<NetworkPlayer> = self
            .players
            .iter()
            .map(|(&actor, tracked)| NetworkPlayer {
                actor,
                key: PlayerKey(
                    tracked
                        .unique_id
                        .clone()
                        .or_else(|| tracked.player_id.map(|id| format!("player_id:{id}")))
                        .unwrap_or_else(|| format!("actor:{}", actor.0)),
                ),
                name: tracked.name.clone(),
                team: tracked
                    .team_actor
                    .and_then(|team| self.actors.get(&team))
                    .and_then(|team| match team.kind {
                        ActorKind::Team(team) => Some(team),
                        _ => None,
                    }),
                body_product_ids: tracked.body_product_ids.clone(),
                stats: tracked.stats.clone(),
                ping_raw: tracked.ping_raw.clone(),
            })
            .collect();
        players.sort_by_key(|player| player.actor);
        for player in &players {
            if let Some(previous) = self
                .last_player_keys
                .insert(player.actor, player.key.clone())
                && previous != player.key
            {
                self.diagnostics.player_key_changes += 1;
            }
        }
        let by_actor: HashMap<ActorId, &NetworkPlayer> =
            players.iter().map(|p| (p.actor, p)).collect();
        let mut cars: Vec<NetworkCar> = self
            .cars
            .iter()
            .map(|(&actor, tracked)| {
                let player = tracked.player_actor.and_then(|id| by_actor.get(&id));
                let team = player.and_then(|p| p.team);
                NetworkCar {
                    life: CarLife {
                        actor,
                        created: tracked.created,
                    },
                    player: player.map(|p| p.key.clone()),
                    player_link_active: tracked.player_link_active,
                    team,
                    body_product_id: player.zip(team).and_then(|(p, team)| {
                        p.body_product_ids[usize::from(team.number())].clone()
                    }),
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
        cars.sort_by_key(|car| car.life.actor);
        self.diagnostics.unlinked_car_frames +=
            cars.iter().filter(|car| car.player.is_none()).count();
        NetworkFrame {
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
            pad_records,
        }
    }
}

/// Map names (lower case) measured on the train split: standard soccar arenas (Throwback, `Stadium_10A`,
/// included). A map outside the list is reported as non-standard.
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

/// What makes a replay non-standard for the soccar arena replicar simulates: an unknown map, a game
/// mutator index other than -1, a ball gravity or speed scale other than 1.
pub(super) fn nonstandard_notes(
    map: Option<&str>,
    settings: &BTreeMap<String, Vec<String>>,
) -> Vec<String> {
    let mut notes = Vec::new();
    match map {
        None => notes.push("the replay header has no MapName".to_owned()),
        Some(map) if !KNOWN_MAPS.contains(&map.to_lowercase().as_str()) => {
            notes.push(format!(
                "map {map} is not one of the maps measured on train"
            ));
        }
        Some(_) => {}
    }
    for (name, values) in settings {
        let standard = if name == MUTATOR_INDEX { "-1" } else { "1" };
        let standard_value: f32 = standard.parse().unwrap_or(0.0);
        for value in values {
            let is_standard = value == standard
                || value
                    .parse::<f32>()
                    .is_ok_and(|v| (v - standard_value).abs() < 1e-6);
            if !is_standard {
                notes.push(format!("{name} = {value} (standard {standard})"));
            }
        }
    }
    notes
}

#[cfg(test)]
mod tests {
    use super::*;

    fn no_trajectory() -> Trajectory {
        Trajectory {
            location: None,
            rotation: None,
        }
    }

    fn demolition(victim: i32) -> NetworkEvent {
        NetworkEvent::Demolition {
            report: DemolitionReport::Extended,
            attacker_car: Some(ActorId(1)),
            victim_car: Some(ActorId(victim)),
            attacker_player: None,
            self_demolition: false,
            attacker_velocity_raw: [0.0; 3],
            victim_velocity_raw: [0.0; 3],
            repeat: false,
        }
    }

    fn repeat_of(frame: &NetworkFrame) -> bool {
        match &frame.events[0] {
            NetworkEvent::Demolition { repeat, .. } => *repeat,
            _ => unreachable!(),
        }
    }

    fn observe(
        tracker: &mut Tracker,
        actor: i32,
        property: &str,
        attribute: Attribute,
    ) -> Vec<NetworkEvent> {
        let mut reports = FrameReports::default();
        tracker.observe(
            ActorId(actor),
            property,
            &attribute,
            &[],
            FrameIndex(0),
            &mut reports,
        );
        reports.events
    }

    /// Only an increase over a value already seen is a flip reset: the first value, a value re-sent
    /// unchanged, and a recycled actor id are not.
    #[test]
    fn flip_resets_are_increases_of_the_refresh_counter() {
        let mut tracker = Tracker::default();
        assert!(observe(&mut tracker, 5, DODGES_REFRESHED, Attribute::Int(0)).is_empty());
        assert!(observe(&mut tracker, 5, DODGES_REFRESHED, Attribute::Int(0)).is_empty());
        assert_eq!(
            observe(&mut tracker, 5, DODGES_REFRESHED, Attribute::Int(1)),
            [NetworkEvent::FlipReset {
                car: ActorId(5),
                count: 1
            }]
        );
        assert!(observe(&mut tracker, 5, DODGES_REFRESHED, Attribute::Int(1)).is_empty());
        // A car first seen with a nonzero total has no event for it: when it happened is unknown.
        assert!(observe(&mut tracker, 6, DODGES_REFRESHED, Attribute::Int(3)).is_empty());
        tracker.announce(
            ActorId(5),
            "Archetypes.Car.Car_Default",
            FrameIndex(0),
            &no_trajectory(),
        );
        tracker.delete(ActorId(5));
        assert!(observe(&mut tracker, 5, DODGES_REFRESHED, Attribute::Int(1)).is_empty());
    }

    /// A spawn trajectory gives the car's location and, when it only turns the car about the vertical axis,
    /// a heading: 64 is a quarter turn. A trajectory that also tilts the car keeps its location only.
    #[test]
    fn a_spawn_trajectory_gives_a_location_and_a_heading() {
        let trajectory = |yaw, pitch, roll| Trajectory {
            location: Some(boxcars::Vector3i {
                x: 256,
                y: -3840,
                z: 36,
            }),
            rotation: Some(boxcars::Rotation { yaw, pitch, roll }),
        };
        let spawn =
            SpawnPose::from_trajectory(&trajectory(Some(-1), Some(64), None), FrameIndex(7))
                .unwrap();
        assert_eq!(spawn.position, [256.0, -3840.0, 36.0]);
        let [x, y, z, w] = spawn.rotation.unwrap();
        let half = std::f32::consts::FRAC_1_SQRT_2;
        assert!(x == 0.0 && y == 0.0 && (z - half).abs() < 1e-5 && (w - half).abs() < 1e-5);
        let tilted =
            SpawnPose::from_trajectory(&trajectory(Some(13), Some(90), Some(75)), FrameIndex(7));
        assert!(tilted.unwrap().rotation.is_none());
        assert!(SpawnPose::from_trajectory(&no_trajectory(), FrameIndex(7)).is_none());
    }

    /// A car taking over a player another car life had is a replacement (late when linked after its
    /// creation); deleting a player actor clears the link of the cars that still point at it.
    #[test]
    fn replacement_cars_and_deleted_players_are_counted() {
        let link = |tracker: &mut Tracker, car: i32, player: i32, frame: u32| {
            tracker.link(
                ActorId(car),
                PAWN_PLAYER,
                &Attribute::ActiveActor(boxcars::ActiveActor {
                    active: true,
                    actor: boxcars::ActorId(player),
                }),
                FrameIndex(frame),
            );
        };
        let mut tracker = Tracker::default();
        tracker.announce(
            ActorId(5),
            "TAGame.Default__PRI_TA",
            FrameIndex(0),
            &no_trajectory(),
        );
        tracker.announce(
            ActorId(1),
            "Archetypes.Car.Car_Default",
            FrameIndex(0),
            &no_trajectory(),
        );
        link(&mut tracker, 1, 5, 0);
        assert_eq!(tracker.diagnostics.replacement_cars_linked, 0);
        tracker.announce(
            ActorId(2),
            "Archetypes.Car.Car_Default",
            FrameIndex(10),
            &no_trajectory(),
        );
        link(&mut tracker, 2, 5, 12);
        assert_eq!(tracker.diagnostics.replacement_cars_linked, 1);
        assert_eq!(
            tracker.diagnostics.replacement_cars_linked_after_creation,
            1
        );
        tracker.delete(ActorId(5));
        assert_eq!(tracker.diagnostics.players_deleted_with_cars, 2);
        assert!(
            tracker
                .cars
                .values()
                .all(|car| car.player_actor.is_none() && !car.player_link_active)
        );
    }

    #[test]
    fn nonstandard_settings_and_unknown_maps_are_reported() {
        let mut settings = BTreeMap::new();
        settings.insert(MUTATOR_INDEX.to_owned(), vec!["-1".to_owned()]);
        settings.insert(
            BALL_GRAVITY_SCALE.to_owned(),
            vec!["1".to_owned(), "0.5".to_owned()],
        );
        let notes = nonstandard_notes(Some("Stadium_P"), &settings);
        assert_eq!(notes.len(), 1, "{notes:?}");
        assert!(notes[0].contains("BallGravityScale = 0.5"));
        assert!(
            nonstandard_notes(Some("SomeNewMap_P"), &BTreeMap::new())[0].contains("SomeNewMap_P")
        );
        assert!(nonstandard_notes(Some("CS_P"), &BTreeMap::new()).is_empty());
        let mutated = BTreeMap::from([(MUTATOR_INDEX.to_owned(), vec!["3".to_owned()])]);
        assert_eq!(nonstandard_notes(Some("Stadium_P"), &mutated).len(), 1);
    }

    /// A report of the same victim 1.7 s later is a repeat, 4.4 s later a new demolition; another victim
    /// is unaffected, and a recycled car actor does not inherit the window.
    #[test]
    fn a_demolition_reported_again_within_the_window_is_a_repeat() {
        let mut tracker = Tracker::default();
        let snapshot = |tracker: &mut Tracker, frame: u32, time: f32, victim: i32| {
            let reports = FrameReports {
                events: vec![demolition(victim)],
                pad_records: Vec::new(),
            };
            tracker.snapshot(FrameIndex(frame), time, 0.03, reports)
        };
        assert!(!repeat_of(&snapshot(&mut tracker, 0, 10.0, 7)));
        assert!(repeat_of(&snapshot(&mut tracker, 1, 11.7, 7)));
        assert!(!repeat_of(&snapshot(&mut tracker, 2, 11.8, 8)));
        assert!(!repeat_of(&snapshot(&mut tracker, 3, 16.1, 7)));
        tracker.announce(
            ActorId(8),
            "Archetypes.Car.Car_Default",
            FrameIndex(3),
            &no_trajectory(),
        );
        tracker.delete(ActorId(8));
        assert!(!repeat_of(&snapshot(&mut tracker, 4, 12.0, 8)));
    }

    /// A pad actor id that is deleted and recycled starts without the previous pad's counter.
    #[test]
    fn a_recycled_pad_does_not_inherit_the_pickup_counter() {
        let mut tracker = Tracker::default();
        let repeat = |tracker: &mut Tracker| {
            let mut reports = FrameReports::default();
            tracker.observe(
                ActorId(9),
                PAD_PICKUP,
                &Attribute::PickupNew(boxcars::PickupNew {
                    instigator: Some(boxcars::ActorId(3)),
                    picked_up: 1,
                }),
                &[],
                FrameIndex(0),
                &mut reports,
            );
            reports.pad_records[0].repeat
        };
        assert!(!repeat(&mut tracker));
        assert!(repeat(&mut tracker));
        tracker.announce(
            ActorId(9),
            "TAGame.VehiclePickup_Boost_TA_1",
            FrameIndex(0),
            &no_trajectory(),
        );
        tracker.delete(ActorId(9));
        assert!(!repeat(&mut tracker));
    }

    /// The ping byte is kept with the frame of its last update; a player without one stays `None`, and a
    /// ping for a non-player actor or of another type is ignored.
    #[test]
    fn the_raw_ping_is_kept_with_its_frame() {
        let mut tracker = Tracker::default();
        for actor in [6, 15] {
            tracker
                .players
                .insert(ActorId(actor), TrackedPlayer::default());
        }
        let ping = |tracker: &mut Tracker, actor: i32, attribute: Attribute, frame: u32| {
            tracker.observe(
                ActorId(actor),
                PLAYER_PING,
                &attribute,
                &[],
                FrameIndex(frame),
                &mut FrameReports::default(),
            );
        };
        let ping_of = |frame: &NetworkFrame, actor: i32| {
            let player = frame
                .players
                .iter()
                .find(|p| p.actor == ActorId(actor))
                .unwrap();
            player.ping_raw.as_ref().map(|p| (p.value, p.frame))
        };
        ping(&mut tracker, 6, Attribute::Byte(9), 1);
        ping(&mut tracker, 99, Attribute::Byte(77), 1);
        ping(&mut tracker, 15, Attribute::Int(5), 1);
        let first = tracker.snapshot(FrameIndex(1), 0.0, 0.0, FrameReports::default());
        assert_eq!(ping_of(&first, 6), Some((9, FrameIndex(1))));
        assert_eq!(ping_of(&first, 15), None);
        ping(&mut tracker, 15, Attribute::Byte(0), 5);
        let later = tracker.snapshot(FrameIndex(6), 0.0, 0.0, FrameReports::default());
        assert_eq!(ping_of(&later, 15), Some((0, FrameIndex(5))));
    }
}
