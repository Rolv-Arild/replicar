//! The network property names the decoder reads: every name in one place, so a reader can see what
//! replicar takes from a replay and grep for where.

// Rigid bodies.
pub(super) const RIGID_BODY: &str = "TAGame.RBActor_TA:ReplicatedRBState";

// Links between actors.
pub(super) const PAWN_PLAYER: &str = "Engine.Pawn:PlayerReplicationInfo";
pub(super) const PLAYER_TEAM: &str = "Engine.PlayerReplicationInfo:Team";
pub(super) const COMPONENT_VEHICLE: &str = "TAGame.CarComponent_TA:Vehicle";

// Player identity and state.
pub(super) const PLAYER_UNIQUE_ID: &str = "Engine.PlayerReplicationInfo:UniqueId";
pub(super) const PLAYER_ID: &str = "Engine.PlayerReplicationInfo:PlayerID";
pub(super) const PLAYER_NAME: &str = "Engine.PlayerReplicationInfo:PlayerName";
pub(super) const PLAYER_PING: &str = "Engine.PlayerReplicationInfo:Ping";
pub(super) const PLAYER_LOADOUTS: &str = "TAGame.PRI_TA:ClientLoadouts";
pub(super) const PLAYER_MATCH_SCORE: &str = "TAGame.PRI_TA:MatchScore";
pub(super) const PLAYER_GOALS: &str = "TAGame.PRI_TA:MatchGoals";
pub(super) const PLAYER_ASSISTS: &str = "TAGame.PRI_TA:MatchAssists";
pub(super) const PLAYER_SAVES: &str = "TAGame.PRI_TA:MatchSaves";
pub(super) const PLAYER_SHOTS: &str = "TAGame.PRI_TA:MatchShots";
pub(super) const PLAYER_DEMOLITIONS: &str = "TAGame.PRI_TA:MatchDemolishes";
// Counted since the builds of September 2026 (older replays leave them unset).
pub(super) const PLAYER_EPIC_SAVES: &str = "TAGame.PRI_TA:MatchEpicSaves";
pub(super) const PLAYER_CLEARS: &str = "TAGame.PRI_TA:MatchClears";
pub(super) const PLAYER_CENTERS: &str = "TAGame.PRI_TA:MatchCenters";
pub(super) const PLAYER_AERIAL_HITS: &str = "TAGame.PRI_TA:MatchAerialHits";
pub(super) const PLAYER_FIRST_TOUCHES: &str = "TAGame.PRI_TA:MatchFirstTouches";
pub(super) const PLAYER_CROSSBAR_HITS: &str = "TAGame.PRI_TA:MatchCrossbarHits";
pub(super) const PLAYER_BICYCLE_HITS: &str = "TAGame.PRI_TA:MatchBicycleHits";
pub(super) const PLAYER_JUGGLE_HITS: &str = "TAGame.PRI_TA:MatchJuggleHits";
pub(super) const PLAYER_FLIP_RESETS: &str = "TAGame.PRI_TA:MatchFlipResets";
pub(super) const PLAYER_TIMES_DEMOLISHED: &str = "TAGame.PRI_TA:MatchTimesDemolished";

// Car controls and components.
pub(super) const THROTTLE: &str = "TAGame.Vehicle_TA:ReplicatedThrottle";
pub(super) const STEER: &str = "TAGame.Vehicle_TA:ReplicatedSteer";
pub(super) const HANDBRAKE: &str = "TAGame.Vehicle_TA:bReplicatedHandbrake";
pub(super) const BOOST_AMOUNT: &str = "TAGame.CarComponent_Boost_TA:ReplicatedBoostAmount";
pub(super) const BOOST: &str = "TAGame.CarComponent_Boost_TA:ReplicatedBoost";
pub(super) const COMPONENT_ACTIVE: &str = "TAGame.CarComponent_TA:ReplicatedActive";
pub(super) const DODGE_TORQUE: &str = "TAGame.CarComponent_Dodge_TA:DodgeTorque";
pub(super) const DODGES_REFRESHED: &str = "TAGame.Car_TA:DodgesRefreshedCounter";

// Demolitions.
pub(super) const DEMOLISH_EXTENDED: &str = "TAGame.Car_TA:ReplicatedDemolishExtended";
pub(super) const DEMOLISH: &str = "TAGame.Car_TA:ReplicatedDemolish";
pub(super) const DEMOLISH_GOAL_EXPLOSION: &str = "TAGame.Car_TA:ReplicatedDemolishGoalExplosion";

// Boost pads.
pub(super) const PAD_PICKUP: &str = "TAGame.VehiclePickup_TA:NewReplicatedPickupData";

// The match.
pub(super) const TEAM_SCORE: &str = "Engine.TeamInfo:Score";
pub(super) const SECONDS_REMAINING: &str = "TAGame.GameEvent_Soccar_TA:SecondsRemaining";
pub(super) const OVERTIME: &str = "TAGame.GameEvent_Soccar_TA:bOverTime";
pub(super) const GAME_STATE: &str = "TAGame.GameEvent_TA:ReplicatedStateName";
pub(super) const SCORED_ON_TEAM: &str = "TAGame.GameEvent_Soccar_TA:ReplicatedScoredOnTeam";

/// Game settings the simulation does not model: reported in the diagnostics when they are not standard.
pub(super) const MUTATOR_INDEX: &str = "ProjectX.GRI_X:ReplicatedGameMutatorIndex";
pub(super) const BALL_GRAVITY_SCALE: &str = "TAGame.Ball_TA:ReplicatedBallGravityScale";
pub(super) const BALL_MAX_SPEED_SCALE: &str = "TAGame.Ball_TA:ReplicatedBallMaxLinearSpeedScale";

// Replay header properties.
pub(super) const HEADER_MAP_NAME: &str = "MapName";
pub(super) const HEADER_BLUE_SCORE: &str = "Team0Score";
pub(super) const HEADER_ORANGE_SCORE: &str = "Team1Score";
