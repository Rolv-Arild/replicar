//! Annotate: what the simulated frames show beyond the state (docs/v2-plan.md, section 4.1). Simulated touches,
//! ball contacts found from the ball's updates and placed against the cars' exported poses, and boost pickups
//! checked against the cars' paths. Reads only the simulator's frames, in order.

mod ball_evidence;
pub mod scoreboard;
pub mod segments;
pub mod updates;

use std::collections::{HashMap, VecDeque};

use glam::{Mat3A, Quat, Vec2, Vec3A};
use replicar_format::{FrameIndex, PlayerIndex};
use rocketsim::ArenaEvent;

pub use ball_evidence::{BallInterval, CONTACT_VELOCITY_THRESHOLD, ball_intervals};

use crate::decode::ActorId;
use crate::simulate::SimulatedFrame;

/// Frames of car poses kept for placing contacts and pickups.
const RECENT_FRAMES: usize = 8;
/// Simulated touches kept for matching contacts.
const RECENT_TOUCHES: usize = 24;
/// A contact's car is the nearest one only within this gap (UU, hitbox to ball surface).
const CONTACT_MAX_GAP: f32 = 150.0;

/// The first tick of a car-ball contact in the simulation. Simulated: overlaps `BallContact`; never add them.
#[derive(Debug, Clone, PartialEq)]
pub struct SimulatedTouch {
    pub player: PlayerIndex,
    pub replay_tick: u64,
    pub contact_point: [f32; 3],
}

/// A contact found from the ball's updates, ending at this frame (docs/glossary.md, "Ball contact"). The
/// preferred touch evidence.
#[derive(Debug, Clone, PartialEq)]
pub struct BallContact {
    /// The frame of the interval's first update; the contact is between it and this frame.
    pub frame_a: usize,
    /// The estimated replay tick of the contact.
    pub replay_tick: u64,
    /// The replay ticks of the interval's two updates.
    pub tick_from: u64,
    pub tick_to: u64,
    /// The car nearest the ball when it reached it, and that gap (UU; negative: overlapping); `None` when no
    /// car was within 150 UU (a post, a wreck).
    pub player: Option<PlayerIndex>,
    pub gap: Option<f32>,
    /// The velocity difference to the contact-free rollout (UU/s).
    pub velocity_residual: f32,
    /// A simulated touch falls in the same interval.
    pub simulated_touch: bool,
}

/// A boost pad the replay reports taken, checked against the cars' paths.
#[derive(Debug, Clone, PartialEq)]
pub struct BoostPickup {
    /// The pad (RocketSim's index) and its size, when the record could be matched to one.
    pub pad_index: Option<usize>,
    pub pad: ActorId,
    pub is_big: Option<bool>,
    /// The player of the car the replay names.
    pub player: Option<PlayerIndex>,
    /// That car's path (straight lines between the exported poses of the last frames) enters the pad's
    /// trigger cylinder (radius 144 small, 208 big, plus 60 for the car's size).
    pub verified: bool,
    /// The named car's closest horizontal distance to the pad (UU).
    pub distance: Option<f32>,
    /// When the named car is not verified: another car whose path reaches the pad.
    pub suggested_player: Option<PlayerIndex>,
    /// The replay tick of the closest approach (of the verified or suggested car), else the frame's.
    pub replay_tick: u64,
}

/// A frame's annotations.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Annotations {
    pub simulated_touches: Vec<SimulatedTouch>,
    pub ball_contacts: Vec<BallContact>,
    pub boost_pickups: Vec<BoostPickup>,
}

/// A car's pose at a frame: player index, position, rotation, demolished, the car life's creation frame.
type Pose = (usize, Vec3A, Mat3A, bool, Option<FrameIndex>);

/// Annotates the simulated frames in order.
pub struct Annotator {
    /// The contact intervals by the frame that ends them.
    contacts: HashMap<usize, BallInterval>,
    recent_poses: VecDeque<(u64, Vec<Pose>)>,
    recent_touch_ticks: VecDeque<u64>,
    /// The last sim tick of a car-ball contact event per player.
    last_contact_tick: HashMap<usize, u64>,
}

impl Annotator {
    /// `intervals` from `ball_intervals` (only those over the contact threshold are used); without them the
    /// frames get no ball contacts.
    #[must_use]
    pub fn new(intervals: Vec<BallInterval>) -> Self {
        Self {
            contacts: intervals
                .into_iter()
                .filter(|i| i.velocity_residual > CONTACT_VELOCITY_THRESHOLD)
                .map(|i| (i.frame_b, i))
                .collect(),
            recent_poses: VecDeque::new(),
            recent_touch_ticks: VecDeque::new(),
            last_contact_tick: HashMap::new(),
        }
    }

    #[must_use]
    pub fn annotate(&mut self, frame: &SimulatedFrame) -> Annotations {
        // Sim ticks to the replay timeline: the offset at the end of this frame.
        let offset = frame.replay_tick as i64 - frame.state.tick_count as i64;
        let simulated_touches = self.touches(frame, offset);
        let lives: HashMap<usize, FrameIndex> = frame
            .lives
            .iter()
            .map(|&(p, created)| (p.get(), created))
            .collect();
        // The simulation holds a car on its spawn pose demolished (out of collisions) and exports it as not
        // demolished: for placing contacts and pickups it is out of play, as in the simulation.
        self.recent_poses.push_back((
            frame.replay_tick,
            frame
                .state
                .cars
                .iter()
                .enumerate()
                .map(|(i, (_, car))| {
                    let spawning = frame.spawning.iter().any(|p| p.get() == i);
                    (
                        i,
                        car.phys.pos,
                        car.phys.rot_mat,
                        car.is_demoed || spawning,
                        lives.get(&i).copied(),
                    )
                })
                .collect(),
        ));
        while self.recent_poses.len() > RECENT_FRAMES {
            self.recent_poses.pop_front();
        }
        for touch in &simulated_touches {
            self.recent_touch_ticks.push_back(touch.replay_tick);
        }
        while self.recent_touch_ticks.len() > RECENT_TOUCHES {
            self.recent_touch_ticks.pop_front();
        }
        let ball_contacts = self.contact(frame).into_iter().collect();
        let boost_pickups = frame
            .pickups
            .iter()
            .map(|pickup| self.pickup(frame, pickup))
            .collect();
        Annotations {
            simulated_touches,
            ball_contacts,
            boost_pickups,
        }
    }

    /// The first tick of each car-ball contact: RocketSim reports a hit every tick of a contact (and the extra
    /// impulse again within one), so a hit more than two ticks after the car's last is a new one.
    fn touches(&mut self, frame: &SimulatedFrame, offset: i64) -> Vec<SimulatedTouch> {
        let mut touches = Vec::new();
        for e in &frame.events {
            if let ArenaEvent::CarHitBall(hit) = &e.event {
                let new_contact = self
                    .last_contact_tick
                    .get(&hit.car_idx)
                    .is_none_or(|&last| e.sim_tick > last + 2);
                self.last_contact_tick.insert(hit.car_idx, e.sim_tick);
                if new_contact && let Ok(player) = u8::try_from(hit.car_idx) {
                    touches.push(SimulatedTouch {
                        player: PlayerIndex(player),
                        replay_tick: (e.sim_tick as i64 + offset).max(0) as u64,
                        contact_point: hit.contact_point.to_array(),
                    });
                }
            }
        }
        touches
    }

    /// A player's car pose at replay tick `tick`, interpolated between the recent frames around it (the
    /// rotation along the shortest arc); unknown across two car lives (a respawn).
    fn pose_at(&self, player: usize, tick: u64) -> Option<(Vec3A, Mat3A, bool)> {
        let at = |k: usize| {
            self.recent_poses.get(k).and_then(|(t, cars)| {
                cars.iter()
                    .find(|c| c.0 == player)
                    .map(|c| (*t, c.1, c.2, c.3, c.4))
            })
        };
        let n = self.recent_poses.len();
        let after = (0..n).find(|&k| self.recent_poses[k].0 >= tick)?;
        let (t1, p1, r1, d1, life1) = at(after)?;
        if after == 0 || t1 == tick {
            return Some((p1, r1, d1));
        }
        let (t0, p0, r0, _, life0) = at(after - 1)?;
        if life0 != life1 {
            return None;
        }
        let f = (tick - t0) as f32 / (t1 - t0).max(1) as f32;
        let rotation = Mat3A::from_quat(Quat::from_mat3a(&r0).slerp(Quat::from_mat3a(&r1), f));
        Some((p0 + (p1 - p0) * f, rotation, d1))
    }

    /// The contact whose interval ends at this frame: the first tick a car's hitbox reaches the ball's
    /// contact-free path, else the nearest approach.
    fn contact(&self, frame: &SimulatedFrame) -> Option<BallContact> {
        let interval = self.contacts.get(&frame.index.get())?;
        let (tick_from, tick_to) = (interval.tick_a.max(0) as u64, interval.tick_b.max(0) as u64);
        // (tick, player, gap)
        let mut best: Option<(u64, usize, f32)> = None;
        'ticks: for (k, ball_pos) in interval.path.iter().enumerate() {
            let tick = tick_from + k as u64 + 1;
            let ball = Vec3A::from(*ball_pos);
            let mut here: Option<(usize, f32)> = None;
            for (player, (info, _)) in frame.state.cars.iter().enumerate() {
                let Some((pos, rot, demolished)) = self.pose_at(player, tick) else {
                    continue;
                };
                if demolished {
                    continue;
                }
                let config = &info.config;
                let local = rot.transpose() * (ball - pos) - config.hitbox_pos_offset;
                let q = local.abs() - config.hitbox_size * 0.5;
                let gap = q.max(Vec3A::ZERO).length() + q.max_element().min(0.0) - 91.25;
                if here.is_none_or(|(_, g)| gap < g) {
                    here = Some((player, gap));
                }
            }
            if let Some((player, gap)) = here {
                if best.is_none_or(|(_, _, g)| gap < g) {
                    best = Some((tick, player, gap));
                }
                if gap <= 0.0 {
                    best = Some((tick, player, gap));
                    break 'ticks;
                }
            }
        }
        let simulated_touch = self
            .recent_touch_ticks
            .iter()
            .any(|&t| t + 2 >= tick_from && t <= tick_to + 2);
        let near = best.filter(|&(_, _, gap)| gap <= CONTACT_MAX_GAP);
        Some(BallContact {
            frame_a: interval.frame_a,
            replay_tick: near.map_or((tick_from + tick_to) / 2, |b| b.0),
            tick_from,
            tick_to,
            player: near.and_then(|b| u8::try_from(b.1).ok()).map(PlayerIndex),
            gap: near.map(|b| b.2),
            velocity_residual: interval.velocity_residual,
            simulated_touch,
        })
    }

    /// A player's closest horizontal approach to `pad` over the recent frames (straight lines between the
    /// poses), and its replay tick.
    fn closest(&self, player: usize, pad: Vec3A) -> Option<(f32, u64)> {
        let target = Vec2::new(pad.x, pad.y);
        let mut best: Option<(f32, u64)> = None;
        let mut previous: Option<(u64, Vec3A)> = None;
        for (tick, cars) in &self.recent_poses {
            let Some(&(_, pos, _, demolished, _)) = cars.iter().find(|c| c.0 == player) else {
                previous = None;
                continue;
            };
            if demolished {
                previous = None;
                continue;
            }
            let candidate = if let Some((t0, p0)) = previous {
                let (a, b) = (Vec2::new(p0.x, p0.y), Vec2::new(pos.x, pos.y));
                let ab = b - a;
                let f = if ab.length_squared() > 1e-6 {
                    ((target - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                (
                    (a + ab * f - target).length(),
                    t0 + ((*tick - t0) as f32 * f) as u64,
                )
            } else {
                ((Vec2::new(pos.x, pos.y) - target).length(), *tick)
            };
            if best.is_none_or(|(bd, _)| candidate.0 < bd) {
                best = Some(candidate);
            }
            previous = Some((*tick, pos));
        }
        best
    }

    fn pickup(&self, frame: &SimulatedFrame, pickup: &crate::simulate::PickupMatch) -> BoostPickup {
        let pad = pickup
            .pad_index
            .and_then(|index| frame.state.boost_pads.get(index))
            .map(|(config, _)| (config.pos, config.is_big));
        // The trigger radius plus half a car (the game tests the hitbox, RocketSim the centre).
        let radius = if pad.is_some_and(|p| p.1) {
            208.0
        } else {
            144.0
        } + 60.0;
        let own = pickup
            .player
            .and_then(|player| self.closest(player.get(), pad?.0));
        let verified = own.is_some_and(|(d, _)| d <= radius);
        let mut suggested: Option<(usize, u64)> = None;
        if !verified && let Some((pad_pos, _)) = pad {
            let mut best: Option<(f32, usize, u64)> = None;
            for player in 0..frame.state.cars.len() {
                if Some(player) == pickup.player.map(PlayerIndex::get) {
                    continue;
                }
                if let Some((d, t)) = self.closest(player, pad_pos)
                    && d <= radius
                    && best.is_none_or(|(bd, _, _)| d < bd)
                {
                    best = Some((d, player, t));
                }
            }
            suggested = best.map(|(_, player, t)| (player, t));
        }
        BoostPickup {
            pad_index: pickup.pad_index,
            pad: pickup.pad,
            is_big: pad.map(|p| p.1),
            player: pickup.player,
            verified,
            distance: own.map(|o| o.0),
            suggested_player: suggested
                .and_then(|s| u8::try_from(s.0).ok())
                .map(PlayerIndex),
            replay_tick: if verified {
                own.map_or(frame.replay_tick, |o| o.1)
            } else {
                suggested.map_or(frame.replay_tick, |s| s.1)
            },
        }
    }
}
