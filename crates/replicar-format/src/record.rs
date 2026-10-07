//! One row of a replicar file, as plain data (docs/glossary.md). The converter fills every group; the writer
//! keeps the ones asked for. Per-player values are indexed by the player index; `None` is unknown or not
//! applicable, never zero.

use crate::{CarStatus, ClockPhase, FrameIndex, Period, SegmentEnd};

/// A rigid body: position (UU), velocity (UU/s), angular velocity (rad/s) and rotation (unit quaternion x, y,
/// z, w with w >= 0).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Body {
    pub position: [f32; 3],
    pub velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
    pub rotation: [f32; 4],
}

/// The ball.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Ball {
    pub body: Body,
    /// RocketSim's ticks since the kickoff.
    pub ticks_since_kickoff: u64,
}

/// A car's controls, in RocketSim's ranges.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Controls {
    pub throttle: f32,
    pub steer: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub jump: bool,
    pub boost: bool,
    pub handbrake: bool,
}

/// The rest of RocketSim's car state: what restoring the state needs besides the body and controls. The
/// wheels keep only whether each touches.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CarInternals {
    pub is_on_ground: bool,
    pub wheels_with_contact: [bool; 4],
    pub has_jumped: bool,
    pub has_double_jumped: bool,
    pub has_flipped: bool,
    pub flip_relative_torque: [f32; 3],
    pub jump_ticks: u32,
    pub flip_time: f32,
    pub is_flipping: bool,
    pub is_jumping: bool,
    pub air_time: f32,
    pub air_time_since_jump: f32,
    pub time_since_boosted: f32,
    pub is_boosting: bool,
    pub boosting_time: f32,
    pub is_supersonic: bool,
    pub supersonic_grace_timer: f32,
    pub handbrake_value: f32,
    pub is_auto_flipping: bool,
    pub auto_flip_timer: f32,
    pub auto_flip_torque_scale: f32,
    pub bump_cooldown_timer: f32,
    /// The sim tick of the car's last extra ball-hit impulse.
    pub last_extra_hit_tick: Option<u64>,
    pub world_contact_normal: Option<[f32; 3]>,
    pub is_demoed: bool,
    pub demo_respawn_timer: f32,
}

/// A player's car.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Car {
    pub body: Body,
    pub boost: f32,
    pub controls: Controls,
    pub previous_controls: Controls,
    pub internals: CarInternals,
}

/// The `state` group of a frame.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct State {
    pub ball: Ball,
    /// Per player: the car's status, whether replicar inferred it, and the car unless absent.
    pub car_status: Vec<CarStatus>,
    pub car_status_inferred: Vec<bool>,
    pub cars: Vec<Option<Car>>,
    /// Per boost pad of the header's pad layout: seconds until it is available again (0: available).
    pub pad_cooldowns: Vec<f32>,
}

/// Something the replay reports (docs/glossary.md, "Event").
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    Goal {
        scoring_team: u8,
    },
    Demolition {
        attacker: Option<u8>,
        victim: Option<u8>,
        /// The report repeats one already seen (count only the others).
        repeat: bool,
        /// The demolition of a goal explosion.
        goal_explosion: bool,
    },
    FlipReset {
        player: Option<u8>,
    },
}

/// The ball's motion between two updates that no free flight explains (docs/glossary.md, "Ball contact").
#[derive(Debug, Clone, PartialEq)]
pub struct BallContact {
    /// The estimated replay tick of the contact, and the replay ticks of the two updates around it.
    pub replay_tick: u64,
    pub from_tick: u64,
    pub to_tick: u64,
    /// The nearest car's player, when one was close.
    pub player: Option<u8>,
    /// The gap (UU) from that car's hitbox to the ball.
    pub gap: Option<f32>,
    /// How far (UU/s) the second update is from the ball's free flight.
    pub velocity_residual: f32,
}

/// A boost pad the replay reports taken (docs/glossary.md, "Boost pickup").
#[derive(Debug, Clone, PartialEq)]
pub struct BoostPickup {
    /// The pad in the header's pad layout, when matched.
    pub pad: Option<u16>,
    pub is_big: Option<bool>,
    pub player: Option<u8>,
    /// The named car's path reaches the pad.
    pub verified: bool,
    /// Another player whose path reaches it, when the named one's does not.
    pub suggested_player: Option<u8>,
    pub replay_tick: u64,
}

/// The `game` group of a frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Game {
    pub period: Period,
    pub clock_phase: ClockPhase,
    pub seconds_remaining: Option<f32>,
    pub overtime_seconds: Option<f32>,
    /// Blue, orange; as the replay showed them at this frame.
    pub scores: [Option<i32>; 2],
    pub events: Vec<Event>,
    pub ball_contacts: Vec<BallContact>,
    pub boost_pickups: Vec<BoostPickup>,
}

/// The `updates` group of a frame (docs/glossary.md, "Updated", "Update tick").
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Updates {
    pub ball_updated: bool,
    pub ball_update_tick: Option<u32>,
    pub ball_ticks_since_update: Option<u32>,
    pub ball_seconds_since_update: Option<f32>,
    /// Per player.
    pub car_updated: Vec<Option<bool>>,
    pub car_update_tick: Vec<Option<u32>>,
    pub car_ticks_since_update: Vec<Option<u32>>,
    pub car_seconds_since_update: Vec<Option<f32>>,
    /// Per player: the ping byte as the replay sends it.
    pub ping_raw: Vec<Option<u8>>,
}

/// The `future` group of a frame: future-derived, read from later frames of the same play segment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Future {
    pub segment_end: SegmentEnd,
    pub seconds_until_segment_end: f32,
}

/// One frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    pub frame: FrameIndex,
    /// The play segment, `None` outside play.
    pub segment: Option<u32>,
    pub replay_time: f32,
    pub replay_tick: u32,
    pub sim_tick: u64,
    pub state: State,
    pub game: Game,
    pub updates: Updates,
    /// `None` outside play segments.
    pub future: Option<Future>,
}
