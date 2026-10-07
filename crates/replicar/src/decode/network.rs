//! The decoded network feed: per frame, every value replicar reads, each with the frame of its last change
//! (docs/glossary.md, "Network value"). Values are in the replay's own units where they end in `_raw`.

use std::collections::BTreeMap;

use replicar_format::{FrameIndex, Team};

/// A replay actor id. Actor ids identify network objects, not players, and the replay reuses them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ActorId(pub i32);

/// One car actor from its creation to its deletion: the actor id and the frame it was created in. A
/// recycled actor id is another car life.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CarLife {
    pub actor: ActorId,
    pub created: FrameIndex,
}

/// A player's identity across the replay: the platform id when the replay has one, else the player id,
/// else the player's actor (`unique:...`, `player_id:N`, `actor:N`).
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PlayerKey(pub String);

/// Where a network value comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValueSource {
    /// The replay sent it.
    Replay,
    /// Not sent: a team score taken as 0 at the start of the match, before the replay's first score update.
    InferredMatchStart,
}

/// A value with the frame of its last change.
#[derive(Debug, Clone, PartialEq)]
pub struct NetworkValue<T> {
    pub value: T,
    pub frame: FrameIndex,
    pub source: ValueSource,
}

impl<T> NetworkValue<T> {
    pub(super) fn replay(value: T, frame: FrameIndex) -> Self {
        Self {
            value,
            frame,
            source: ValueSource::Replay,
        }
    }
}

/// The last rigid-body update of the ball or a car. An update always carries position, rotation and the
/// sleeping flag, and usually the velocities; a missing velocity keeps its earlier value and frame.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct NetworkBody {
    pub position: Option<NetworkValue<[f32; 3]>>,
    /// Unit quaternion, x, y, z, w.
    pub rotation: Option<NetworkValue<[f32; 4]>>,
    pub linear_velocity: Option<NetworkValue<[f32; 3]>>,
    /// The replay's units: multiply by 0.01 for rad/s.
    pub angular_velocity_raw: Option<NetworkValue<[f32; 3]>>,
    pub sleeping: Option<NetworkValue<bool>>,
}

/// Where the replay creates a car: carried only until the car's first rigid-body update. An inference
/// target, not a body: the first update follows within 0-3 frames and is within 3 UU of it at p90 on train.
#[derive(Debug, Clone, PartialEq)]
pub struct SpawnPose {
    pub position: [f32; 3],
    /// The heading about the vertical axis as a quaternion (x, y, z, w); `None` when the spawn also tilts
    /// the car, which is not decoded.
    pub rotation: Option<[f32; 4]>,
    pub frame: FrameIndex,
}

/// A car's controls and action counters as the replay sends them.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct NetworkInputs {
    /// -1..1, from the replicated byte.
    pub throttle: Option<NetworkValue<f32>>,
    /// -1..1, from the replicated byte.
    pub steer: Option<NetworkValue<f32>>,
    pub handbrake: Option<NetworkValue<bool>>,
    /// The boost component's activation counter (odd while active).
    pub boost_active_raw: Option<NetworkValue<u8>>,
    pub jump_active_raw: Option<NetworkValue<u8>>,
    pub double_jump_active_raw: Option<NetworkValue<u8>>,
    pub dodge_active_raw: Option<NetworkValue<u8>>,
    /// The dodge torque vector in the replay's units; its relation to a stick direction is uncalibrated.
    pub dodge_torque_raw: Option<NetworkValue<[f32; 3]>>,
    pub flip_car_active_raw: Option<NetworkValue<u8>>,
}

/// A car actor in a frame.
#[derive(Debug, Clone, PartialEq)]
pub struct NetworkCar {
    pub life: CarLife,
    /// The player the car's pawn links to; kept when the link goes inactive.
    pub player: Option<PlayerKey>,
    /// Whether the pawn's link to its player is active.
    pub player_link_active: bool,
    pub team: Option<Team>,
    /// The car body product id of the player's loadout for the car's team.
    pub body_product_id: Option<NetworkValue<u32>>,
    pub body: NetworkBody,
    /// 0-100.
    pub boost: Option<NetworkValue<f32>>,
    /// 0-255, as the replay sends it.
    pub boost_raw: Option<NetworkValue<u8>>,
    pub inputs: NetworkInputs,
    /// Present only while the car has no rigid-body update yet.
    pub spawn_pose: Option<SpawnPose>,
}

/// A player's match statistics as the replay sends them (they can arrive late on a client replay).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PlayerStats {
    pub match_score: Option<NetworkValue<i32>>,
    pub goals: Option<NetworkValue<i32>>,
    pub assists: Option<NetworkValue<i32>>,
    pub saves: Option<NetworkValue<i32>>,
    pub shots: Option<NetworkValue<i32>>,
    pub demolitions: Option<NetworkValue<i32>>,
    /// The counters of the builds since September 2026; unset in older replays.
    pub epic_saves: Option<NetworkValue<i32>>,
    pub clears: Option<NetworkValue<i32>>,
    pub centers: Option<NetworkValue<i32>>,
    pub aerial_hits: Option<NetworkValue<i32>>,
    pub first_touches: Option<NetworkValue<i32>>,
    pub crossbar_hits: Option<NetworkValue<i32>>,
    pub bicycle_hits: Option<NetworkValue<i32>>,
    pub juggle_hits: Option<NetworkValue<i32>>,
    pub flip_resets: Option<NetworkValue<i32>>,
    pub times_demolished: Option<NetworkValue<i32>>,
}

/// A player in a frame.
#[derive(Debug, Clone, PartialEq)]
pub struct NetworkPlayer {
    pub actor: ActorId,
    pub key: PlayerKey,
    pub name: Option<String>,
    pub team: Option<Team>,
    /// Car body product ids of the blue and the orange loadout.
    pub body_product_ids: [Option<NetworkValue<u32>>; 2],
    pub stats: PlayerStats,
    /// The ping byte as the replay sends it (probably milliseconds / 4; uncalibrated).
    pub ping_raw: Option<NetworkValue<u8>>,
}

/// The game event's state name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GameState {
    WaitingForPlayers,
    PreGame,
    Countdown,
    /// In play: the cars can move.
    Active,
    PostGoalScored,
    ReplayPlayback,
    /// A state name replicar does not interpret, kept as the replay names it.
    Other(String),
}

impl GameState {
    pub(super) fn from_name(name: &str) -> Self {
        match name {
            "WaitingForPlayers" => Self::WaitingForPlayers,
            "PreGame" => Self::PreGame,
            "Countdown" => Self::Countdown,
            "Active" => Self::Active,
            "PostGoalScored" => Self::PostGoalScored,
            "ReplayPlayback" => Self::ReplayPlayback,
            other => Self::Other(other.to_owned()),
        }
    }

    /// The name the replay uses.
    #[must_use]
    pub fn name(&self) -> &str {
        match self {
            Self::WaitingForPlayers => "WaitingForPlayers",
            Self::PreGame => "PreGame",
            Self::Countdown => "Countdown",
            Self::Active => "Active",
            Self::PostGoalScored => "PostGoalScored",
            Self::ReplayPlayback => "ReplayPlayback",
            Self::Other(name) => name,
        }
    }
}

/// How the replay reported a demolition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DemolitionReport {
    /// `ReplicatedDemolishExtended`: with the attacker's player and whether the victim demolished itself.
    Extended,
    /// `ReplicatedDemolish`.
    Plain,
    /// `ReplicatedDemolishGoalExplosion`: the celebration explosion after a goal.
    GoalExplosion,
}

/// Something the replay reports in a frame.
#[derive(Debug, Clone, PartialEq)]
pub enum NetworkEvent {
    /// The replay's goal report: the team that was scored **on**. Reported once per team per goal pause.
    GoalScoredOn { team: Team },
    /// A demolition, replicated on the victim car. Cars are car actors; velocities in the replay's units.
    Demolition {
        report: DemolitionReport,
        attacker_car: Option<ActorId>,
        victim_car: Option<ActorId>,
        /// Extended reports only.
        attacker_player: Option<ActorId>,
        self_demolition: bool,
        attacker_velocity_raw: [f32; 3],
        victim_velocity_raw: [f32; 3],
        /// The same victim car was reported demolished less than 5 s before: the replay sends some
        /// demolitions twice. Count only reports with `repeat` false. Never set for goal explosions.
        repeat: bool,
    },
    /// The car's `DodgesRefreshedCounter` went up over a value already seen: it regained its flip in the
    /// air. `count` is the car actor's new total.
    FlipReset { car: ActorId, count: i32 },
}

/// A boost pad record as the replay sends it.
#[derive(Debug, Clone, PartialEq)]
pub struct PadRecord {
    pub pad: ActorId,
    /// The pad's name (`VehiclePickup_Boost_TA_14`), which survives the pad actor being created again.
    pub pad_name: Option<String>,
    pub instigator_car: Option<ActorId>,
    /// Odd values count pickups; 255 marks the pad available.
    pub picked_up_raw: u8,
    /// The counter value was already reported with an instigator: the replay re-announces old pickups.
    pub repeat: bool,
}

/// One replay frame of the network feed.
#[derive(Debug, Clone, PartialEq)]
pub struct NetworkFrame {
    pub index: FrameIndex,
    /// Replay time, seconds.
    pub time: f32,
    /// The replay's own frame delta, seconds.
    pub delta: f32,
    pub ball: Option<NetworkBody>,
    /// Every car actor, ordered by actor id.
    pub cars: Vec<NetworkCar>,
    /// Every player, ordered by actor id.
    pub players: Vec<NetworkPlayer>,
    /// Blue, orange.
    pub team_scores: [Option<NetworkValue<i32>>; 2],
    /// The integer clock (the ceiling of the true one).
    pub seconds_remaining: Option<NetworkValue<i32>>,
    pub overtime: Option<NetworkValue<bool>>,
    pub game_state: Option<NetworkValue<GameState>>,
    pub events: Vec<NetworkEvent>,
    pub pad_records: Vec<PadRecord>,
}

impl NetworkFrame {
    /// The car that currently stands for each player, ordered by actor id. A demolished car actor can stay
    /// in the network after its replacement appears; the current one is the one with an active link, then
    /// the newest, then the one updated in this frame.
    #[must_use]
    pub fn current_cars(&self) -> Vec<&NetworkCar> {
        let mut by_player: BTreeMap<&PlayerKey, &NetworkCar> = BTreeMap::new();
        let priority = |car: &NetworkCar| {
            (
                car.player_link_active,
                car.life.created,
                car.body
                    .position
                    .as_ref()
                    .is_some_and(|position| position.frame == self.index),
            )
        };
        for car in &self.cars {
            let Some(key) = car.player.as_ref() else {
                continue;
            };
            match by_player.get_mut(key) {
                Some(current) if priority(car) > priority(current) => *current = car,
                None => {
                    by_player.insert(key, car);
                }
                _ => {}
            }
        }
        let mut cars: Vec<_> = by_player.into_values().collect();
        cars.sort_by_key(|car| car.life.actor);
        cars
    }
}

/// The replay header's facts the reconstruction uses. The final scores are totals and never reach a frame.
#[derive(Debug, Clone, PartialEq)]
pub struct ReplayHeader {
    /// `TAGame.Replay_Soccar_TA` for soccar.
    pub game_type: String,
    pub levels: Vec<String>,
    /// Blue, orange.
    pub final_scores: [Option<i32>; 2],
}

/// What the decoder noticed about the replay.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize)]
pub struct DecodeDiagnostics {
    pub repeated_actor_announcements: usize,
    pub actor_class_replacements: usize,
    pub unknown_actor_updates: usize,
    pub unlinked_car_frames: usize,
    /// The header's `MapName`.
    pub map_name: Option<String>,
    /// The distinct values the replay announced for game settings the simulation does not model.
    pub game_settings: BTreeMap<String, Vec<String>>,
    /// What makes the replay non-standard for the soccar arena replicar simulates. A report, not a refusal.
    pub nonstandard_notes: Vec<String>,
    /// A player's key changed between frames.
    pub player_key_changes: usize,
    /// A player actor was deleted while cars still linked to it (their link is cleared).
    pub players_deleted_with_cars: usize,
    /// A car linked to a player another car life already had (a replacement car), and how many of those
    /// links came after the car's creation frame.
    pub replacement_cars_linked: usize,
    pub replacement_cars_linked_after_creation: usize,
    /// Frames whose time is earlier than the previous frame's.
    pub non_monotonic_frame_times: usize,
}

/// A replay's decoded network feed.
#[derive(Debug, Clone, PartialEq)]
pub struct NetworkReplay {
    pub header: ReplayHeader,
    pub frames: Vec<NetworkFrame>,
    pub diagnostics: DecodeDiagnostics,
}
