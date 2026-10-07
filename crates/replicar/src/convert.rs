//! The whole conversion (docs/v2-plan.md, section 4.5): decode, update ticks, contact alignment, simulation
//! with the fitted inference, annotation, and the rows and header of a replicar file.

use std::path::Path;

use glam::{Mat3A, Quat};
use replicar_format::header::{FORMAT_VERSION, Header, PadInfo, PlayerInfo, SegmentInfo};
use replicar_format::record::{
    Ball, BallContact, Body, BoostPickup, Car, CarInternals, Controls, Event, Frame, Future, Game,
    State, Updates,
};
use replicar_format::{CarStatus, PlayerIndex, WriteOptions};
use rocketsim::{ArenaState, CarControls, CarState, PhysState};
use sha2::{Digest, Sha256};

use crate::annotate::scoreboard::{Decider, reconstruct};
use crate::annotate::segments::{segment_frames, segments};
use crate::annotate::updates::UpdateTracker;
use crate::annotate::{Annotations, Annotator, ball_intervals};
use crate::decode::{DemolitionReport, NetworkEvent, NetworkFrame, NetworkReplay};
use crate::infer::{FittedInference, InferenceOptions};
use crate::simulate::{HoldSource, SimulatedFrame, Simulation, SimulationOptions, simulate};
use crate::update_ticks::{UpdateTicks, Withheld};
use crate::{Error, Meshes};

/// How a replay is converted.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Config {
    pub simulation: SimulationOptions,
    pub inference: InferenceOptions,
    /// Infer each update's tick (off: every update at its frame's own tick).
    pub update_ticks: bool,
    /// Move car update ticks so that simulated hits reproduce the ball's next update (offline).
    pub align_contacts: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            simulation: SimulationOptions::default(),
            inference: InferenceOptions::default(),
            update_ticks: true,
            align_contacts: true,
        }
    }
}

/// A converted replay: the header and every frame's row (the writer keeps the rows in play segments).
#[derive(Debug, Clone)]
pub struct Conversion {
    pub header: Header,
    pub frames: Vec<Frame>,
}

/// Converts replays with one configuration.
pub struct Converter<'m> {
    meshes: &'m Meshes,
    config: Config,
}

impl<'m> Converter<'m> {
    #[must_use]
    pub fn new(meshes: &'m Meshes, config: Config) -> Self {
        Self { meshes, config }
    }

    /// Converts the replay file's bytes.
    pub fn convert(&self, bytes: &[u8]) -> Result<Conversion, Error> {
        let network = crate::decode::decode(&crate::parse(bytes)?)?;
        let mut conversion = self.convert_network(&network)?;
        conversion.header.replay_sha256 = format!("{:x}", Sha256::digest(bytes));
        Ok(conversion)
    }

    /// Converts the replay and writes it to `path` (the file appears only when complete).
    pub fn convert_to_file(
        &self,
        bytes: &[u8],
        path: &Path,
        options: &WriteOptions,
    ) -> Result<(), Error> {
        let conversion = self.convert(bytes)?;
        replicar_format::write(path, &conversion.header, &conversion.frames, options)?;
        Ok(())
    }

    /// Converts a decoded replay (the header's replay hash is left empty).
    pub fn convert_network(&self, network: &NetworkReplay) -> Result<Conversion, Error> {
        let config = &self.config;
        let withheld = Withheld(config.simulation.withheld.as_deref());
        let mut alignment = None;
        let mut ticks = config.update_ticks.then(|| {
            crate::update_ticks::infer(network, config.simulation.loadout_hitboxes, withheld)
        });
        if config.align_contacts
            && let Some(found) = &ticks
        {
            let aligned = crate::align::align_contacts(
                network,
                found,
                self.meshes,
                config.inference,
                &config.simulation,
            )?;
            ticks = Some(aligned.0);
            alignment = Some(aligned.1);
        }
        let mut inference =
            FittedInference::new(network, ticks.as_ref(), config.inference, withheld);
        let mut rows = Rows::new(network, ticks.as_ref(), withheld);
        let simulation = simulate(
            network,
            ticks.as_ref(),
            &mut inference,
            self.meshes,
            config.simulation.clone(),
            |frame| rows.push(&frame),
        )?;
        let mut frames = rows.frames;
        fill_pings(network, &simulation, &mut frames);
        let mut header = header(
            network,
            &simulation,
            rows.last_state.as_ref(),
            &rows.segments,
        );
        header.configuration = serde_json::to_value(config).unwrap_or_default();
        header.diagnostics = serde_json::json!({
            "decode": network.diagnostics,
            "update_ticks": ticks.as_ref().map(|t| serde_json::json!({
                "lag_free": t.lag_free,
                "ball_car_offset": t.ball_car_offset,
                "bridged_hits": t.bridged_hits,
            })),
            "contact_alignment": alignment,
            "simulation": simulation.diagnostics,
            "inference": inference.diagnostics,
        });
        Ok(Conversion { header, frames })
    }
}

/// The per-frame annotation state and the rows built so far.
struct Rows<'a> {
    network: &'a NetworkReplay,
    annotator: Annotator,
    scoreboard: Vec<crate::annotate::scoreboard::Scoreboard>,
    decider: Decider,
    updates: UpdateTracker,
    segments: Vec<crate::annotate::segments::Segment>,
    segment_frames: Vec<Option<crate::annotate::segments::SegmentFrame>>,
    frames: Vec<Frame>,
    last_state: Option<ArenaState>,
}

impl<'a> Rows<'a> {
    fn new(network: &'a NetworkReplay, ticks: Option<&UpdateTicks>, withheld: Withheld) -> Self {
        let scoreboard = reconstruct(&network.frames);
        let segments = segments(&network.frames, &scoreboard);
        Self {
            network,
            annotator: Annotator::new(
                ticks
                    .map(|t| ball_intervals(&network.frames, t, withheld))
                    .unwrap_or_default(),
            ),
            segment_frames: segment_frames(&network.frames, &segments),
            scoreboard,
            decider: Decider::default(),
            updates: UpdateTracker::default(),
            segments,
            frames: Vec::with_capacity(network.frames.len()),
            last_state: None,
        }
    }

    fn push(&mut self, simulated: &SimulatedFrame) {
        let f = simulated.index.get();
        let network_frame = &self.network.frames[f];
        let annotations = self.annotator.annotate(simulated);
        let scoreboard = self.decider.apply(self.scoreboard[f], &simulated.events);
        let updates = self
            .updates
            .frame(&self.network.frames, network_frame, simulated);
        let segment = self.segment_frames[f];
        self.frames.push(Frame {
            frame: simulated.index,
            segment: segment.map(|s| s.segment),
            replay_time: network_frame.time,
            replay_tick: u32::try_from(simulated.replay_tick).unwrap_or(u32::MAX),
            sim_tick: simulated.state.tick_count,
            state: state(simulated),
            game: game(network_frame, simulated, &scoreboard, annotations),
            updates: Updates {
                ball_updated: updates.ball_updated,
                ball_update_tick: update_tick(
                    simulated.replay_tick,
                    updates.ball_ticks_since_update,
                ),
                ball_ticks_since_update: updates.ball_ticks_since_update,
                ball_seconds_since_update: updates.ball_seconds_since_update,
                car_update_tick: updates
                    .car_ticks_since_update
                    .iter()
                    .map(|&t| update_tick(simulated.replay_tick, t))
                    .collect(),
                car_updated: updates.car_updated,
                car_ticks_since_update: updates.car_ticks_since_update,
                car_seconds_since_update: updates.car_seconds_since_update,
                ping_raw: Vec::new(),
            },
            future: segment.map(|s| Future {
                segment_end: s.future_segment_end,
                seconds_until_segment_end: s.future_seconds_until_segment_end,
            }),
        });
        self.last_state = Some(simulated.state.clone());
    }
}

/// The update tick of an update `ticks_since` before `replay_tick`.
fn update_tick(replay_tick: u64, ticks_since: Option<u32>) -> Option<u32> {
    ticks_since.and_then(|t| u32::try_from(replay_tick.checked_sub(u64::from(t))?).ok())
}

/// A rotation matrix as a unit quaternion with w >= 0.
fn quaternion(rotation: Mat3A) -> [f32; 4] {
    let q = Quat::from_mat3a(&rotation).normalize();
    let q = if q.w < 0.0 { -q } else { q };
    q.to_array()
}

fn body(phys: &PhysState) -> Body {
    Body {
        position: phys.pos.to_array(),
        velocity: phys.vel.to_array(),
        angular_velocity: phys.ang_vel.to_array(),
        rotation: quaternion(phys.rot_mat),
    }
}

fn controls(c: &CarControls) -> Controls {
    Controls {
        throttle: c.throttle,
        steer: c.steer,
        pitch: c.pitch,
        yaw: c.yaw,
        roll: c.roll,
        jump: c.jump,
        boost: c.boost,
        handbrake: c.handbrake,
    }
}

fn car(c: &CarState) -> Car {
    Car {
        body: body(&c.phys),
        boost: c.boost,
        controls: controls(&c.controls),
        previous_controls: controls(&c.prev_controls),
        internals: CarInternals {
            is_on_ground: c.is_on_ground,
            wheels_with_contact: c.wheels_with_contact.map(|w| w.is_some()),
            has_jumped: c.has_jumped,
            has_double_jumped: c.has_double_jumped,
            has_flipped: c.has_flipped,
            flip_relative_torque: c.flip_rel_torque.to_array(),
            jump_ticks: c.jump_ticks,
            flip_time: c.flip_time,
            is_flipping: c.is_flipping,
            is_jumping: c.is_jumping,
            air_time: c.air_time,
            air_time_since_jump: c.air_time_since_jump,
            time_since_boosted: c.time_since_boosted,
            is_boosting: c.is_boosting,
            boosting_time: c.boosting_time,
            is_supersonic: c.is_supersonic,
            supersonic_grace_timer: c.supersonic_grace_timer,
            handbrake_value: c.handbrake_val,
            is_auto_flipping: c.is_auto_flipping,
            auto_flip_timer: c.auto_flip_timer,
            auto_flip_torque_scale: c.auto_flip_torque_scale,
            bump_cooldown_timer: c.bump_cooldown_timer,
            last_extra_hit_tick: c.last_extra_hit_tick,
            world_contact_normal: c.world_contact_normal.map(|n| n.to_array()),
            is_demoed: c.is_demoed,
            demo_respawn_timer: c.demo_respawn_timer,
        },
    }
}

/// The `state` group: the simulation's state, each player's car status.
fn state(simulated: &SimulatedFrame) -> State {
    let has = |list: &[PlayerIndex], p: usize| list.iter().any(|q| q.get() == p);
    let mut status = Vec::new();
    let mut inferred = Vec::new();
    for (p, (_, car)) in simulated.state.cars.iter().enumerate() {
        let wreck = simulated.wrecks_held.iter().find(|(q, _)| q.get() == p);
        let (s, i) = if has(&simulated.spawning, p) {
            (CarStatus::Spawning, true)
        } else if let Some((_, source)) = wreck {
            (CarStatus::Demolished, *source == HoldSource::Inferred)
        } else if car.is_demoed {
            (CarStatus::Demolished, false)
        } else {
            (CarStatus::Active, false)
        };
        status.push(s);
        inferred.push(i);
    }
    State {
        ball: Ball {
            body: body(&simulated.state.ball.phys),
            ticks_since_kickoff: simulated.state.ball.tick_count_since_kickoff,
        },
        car_status: status,
        car_status_inferred: inferred,
        cars: simulated
            .state
            .cars
            .iter()
            .map(|(_, c)| Some(car(c)))
            .collect(),
        pad_cooldowns: simulated
            .state
            .boost_pads
            .iter()
            .map(|(_, pad)| pad.cooldown)
            .collect(),
    }
}

/// The `game` group of a frame.
fn game(
    network_frame: &NetworkFrame,
    simulated: &SimulatedFrame,
    scoreboard: &crate::annotate::scoreboard::Scoreboard,
    annotations: Annotations,
) -> Game {
    let player_of = |car: Option<crate::decode::ActorId>| {
        let car = car?;
        simulated
            .car_players
            .iter()
            .find(|(actor, _)| *actor == car)
            .map(|(_, p)| p.0)
    };
    let events = network_frame
        .events
        .iter()
        .map(|event| match event {
            NetworkEvent::GoalScoredOn { team } => Event::Goal {
                scoring_team: team.opponent().number(),
            },
            NetworkEvent::Demolition {
                report,
                attacker_car,
                victim_car,
                repeat,
                ..
            } => Event::Demolition {
                attacker: player_of(*attacker_car),
                victim: player_of(*victim_car),
                repeat: *repeat,
                goal_explosion: *report == DemolitionReport::GoalExplosion,
            },
            NetworkEvent::FlipReset { car, .. } => Event::FlipReset {
                player: player_of(Some(*car)),
            },
        })
        .collect();
    Game {
        period: scoreboard.period,
        clock_phase: scoreboard.clock_phase,
        seconds_remaining: scoreboard.seconds_remaining,
        overtime_seconds: scoreboard.overtime_seconds,
        scores: network_frame
            .team_scores
            .each_ref()
            .map(|s| s.as_ref().map(|v| v.value)),
        events,
        ball_contacts: annotations
            .ball_contacts
            .into_iter()
            .map(|c| BallContact {
                replay_tick: c.replay_tick,
                from_tick: c.tick_from,
                to_tick: c.tick_to,
                player: c.player.map(|p| p.0),
                gap: c.gap,
                velocity_residual: c.velocity_residual,
            })
            .collect(),
        boost_pickups: annotations
            .boost_pickups
            .into_iter()
            .map(|b| BoostPickup {
                pad: b.pad_index.and_then(|i| u16::try_from(i).ok()),
                is_big: b.is_big,
                player: b.player.map(|p| p.0),
                verified: b.verified,
                suggested_player: b.suggested_player.map(|p| p.0),
                replay_tick: b.replay_tick,
            })
            .collect(),
    }
}

/// Each player's ping byte as the frame's network feed has it, once the players are known.
fn fill_pings(network: &NetworkReplay, simulation: &Simulation, frames: &mut [Frame]) {
    for row in frames.iter_mut() {
        let network_frame = &network.frames[row.frame.get()];
        row.updates.ping_raw = simulation
            .players
            .iter()
            .map(|player| {
                network_frame
                    .players
                    .iter()
                    .find(|p| p.key == player.key)
                    .and_then(|p| p.ping_raw.as_ref())
                    .map(|v| v.value)
            })
            .collect();
    }
}

fn header(
    network: &NetworkReplay,
    simulation: &Simulation,
    last_state: Option<&ArenaState>,
    found: &[crate::annotate::segments::Segment],
) -> Header {
    let name_of = |key| {
        network
            .frames
            .iter()
            .rev()
            .flat_map(|f| &f.players)
            .find(|p| &p.key == key)
            .and_then(|p| p.name.clone())
    };
    Header {
        format_version: FORMAT_VERSION,
        replay_sha256: String::new(),
        replicar_version: env!("CARGO_PKG_VERSION").to_owned(),
        rocketsim_version: crate::ROCKETSIM_VERSION.to_owned(),
        groups: Vec::new(),
        precision: "float32".to_owned(),
        all_frames: false,
        players: simulation
            .players
            .iter()
            .map(|p| PlayerInfo {
                index: p.index.0,
                key: p.key.0.clone(),
                name: name_of(&p.key),
                team: p.team.number(),
                body_product_id: p.body_product_id,
                hitbox: p.hitbox.name().to_owned(),
            })
            .collect(),
        pads: last_state.map_or_else(Vec::new, |s| {
            s.boost_pads
                .iter()
                .map(|(config, _)| PadInfo {
                    position: config.pos.to_array(),
                    is_big: config.is_big,
                })
                .collect()
        }),
        segments: found
            .iter()
            .map(|s| SegmentInfo {
                first_frame: s.first as u32,
                last_frame: s.last as u32,
                end: s.end.name().to_owned(),
            })
            .collect(),
        final_scores: network.frames.last().map_or([None, None], |f| {
            f.team_scores
                .each_ref()
                .map(|s| s.as_ref().map(|v| v.value))
        }),
        configuration: serde_json::Value::Null,
        diagnostics: serde_json::Value::Null,
    }
}
