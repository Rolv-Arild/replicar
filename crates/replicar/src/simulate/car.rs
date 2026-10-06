//! One car's update in a frame: the state from its update, its status (spawning, wreck, demolished), its
//! actions (jumps, dodges, double jumps from the counters), and the controls it drives on with.

use glam::{Mat3A, Quat, Vec3A};
use replicar_format::PlayerIndex;
use rocketsim::{CarControls, CarState};

use super::updates::{
    apply_update, dodge_impulse_unseen, dodge_torque, jump_impulse_unseen, network_controls,
    zero_sleeping_velocity,
};
use super::{FittedKind, FrameContext, HoldSource, Simulator};
use crate::decode::{DemolitionReport, NetworkCar, NetworkEvent, NetworkValue};
use crate::infer::AirScheduleQuery;

/// What the simulation keeps per car life.
#[derive(Debug, Clone, Default)]
pub(super) struct CarTrack {
    /// The car has been placed on its spawn pose once.
    pub(super) spawn_started: bool,
    /// The car's spawn hold ended in an observed demolition: an ordinary demolition from then on.
    pub(super) spawn_demolished: bool,
    /// The jump counter's last value said a jump the updates have not shown yet.
    pub(super) jump_gate: Option<bool>,
    pub(super) last_dodge_raw: Option<u8>,
    pub(super) last_double_jump_raw: Option<u8>,
    /// Action counters (jump, double jump, dodge) at the car's last frame, and at its last frame on the
    /// ground.
    pub(super) last_counters: Option<[u8; 3]>,
    pub(super) ground_counters: Option<[u8; 3]>,
}

/// The input a counter says began on this frame, for the car's controls.
#[derive(Default)]
struct Press {
    /// A jump (a double jump without direction, or a dodge with one) to press now.
    jump: bool,
    pitch: f32,
    yaw: f32,
}

impl Simulator<'_, '_> {
    /// Applies one car's update at its update tick.
    pub(super) fn update_car(&mut self, ctx: &mut FrameContext, car: &NetworkCar) {
        let frame = ctx.index;
        let resolved = self.players.resolve(
            car,
            frame,
            &mut self.arena,
            self.options.loadout_hitboxes,
            &mut self.diagnostics,
        );
        let Some(resolved) = resolved else {
            self.diagnostics.unlinked_car_frames += 1;
            return;
        };
        let player = resolved.player;
        let new_life = resolved.new_life;
        if car.player.is_none() && ctx.selected.contains(&player) {
            self.diagnostics.shadowed_car_frames += 1;
            return;
        }
        ctx.selected.insert(player);
        let slot = player.get();
        let mut state = if new_life {
            CarState::default()
        } else {
            *self.arena.get_car_state(slot)
        };
        let mut dirty = apply_update(&mut state.phys, &car.body, frame, new_life) || new_life;
        dirty |= self.spawn(car, player, &mut state);
        dirty |= self.wreck(ctx, car, player, &mut state);
        if let Some(boost) = &car.boost
            && (new_life || boost.frame == frame)
        {
            state.boost = boost.value;
            dirty = true;
        }
        let (press, changed) = self.actions(ctx, car, &mut state, new_life);
        dirty |= changed;
        if dirty {
            self.arena.set_car_state(slot, state);
            // RocketSim asks for this after teleporting a car mid-drive.
            self.arena.refresh_car_sticky_gate(slot);
        }
        let controls = self.controls(ctx, car, &state, new_life, &press);
        self.arena.set_car_controls(slot, controls);
        let updated_now = car.body.position.as_ref().is_some_and(|p| p.frame == frame);
        if ctx.simulated && ctx.in_play && !new_life && !press.jump && updated_now {
            self.plan_air(ctx, car, player);
        }
    }

    /// Replaces the car's air schedule with one to its next update, when the inference has one.
    fn plan_air(&mut self, ctx: &mut FrameContext, car: &NetworkCar, player: PlayerIndex) {
        self.air_schedules
            .retain(|schedule| schedule.player != player);
        let state = *self.arena.get_car_state(player.get());
        let now = self.arena.tick_count();
        let query = AirScheduleQuery {
            index: ctx.index.get(),
            car,
            state: &state,
            ticks_before: self.car_ticks(car, ctx.index.get(), ctx.simulated, ctx.gap),
            player,
            now,
        };
        if let Some(schedule) = self.inference.air_schedule(&query) {
            ctx.fitted.push((
                player,
                now,
                FittedKind::Air {
                    span_ticks: schedule.end_tick - now,
                },
            ));
            self.air_schedules.push(schedule);
        }
    }

    /// Spawn poses. Before its first rigid-body update a car starts from the replay's spawn pose (inferred),
    /// and the car it stands for is kept out of collisions until that update (a respawned car whose real
    /// self already hit the ball before the pose was taken would hit it again). RocketSim has no
    /// per-car collision switch other than the demolished state, so the car is held demolished inside the
    /// simulation and exported as not demolished. The hold belongs to the car life that started it, and
    /// lasts while the car has no body in this frame, also when it has no spawn pose any more (in a
    /// withheld frame the body is hidden but the spawn pose's disappearance is not).
    fn spawn(&mut self, car: &NetworkCar, player: PlayerIndex, state: &mut CarState) -> bool {
        let mut dirty = false;
        let track = self.cars.entry(car.life).or_default();
        if car.body.position.is_none()
            && let Some(spawn) = &car.spawn_pose
            && !track.spawn_started
        {
            track.spawn_started = true;
            state.phys.pos = Vec3A::from(spawn.position);
            if let Some([x, y, z, w]) = spawn.rotation {
                state.phys.rot_mat = Mat3A::from_quat(Quat::from_xyzw(x, y, z, w));
            }
            state.phys.vel = Vec3A::ZERO;
            state.phys.ang_vel = Vec3A::ZERO;
            dirty = true;
            self.diagnostics.cars_started_from_spawn_pose += 1;
        }
        let spawn_demolished = track.spawn_demolished;
        // Another car life on the player ends a hold.
        if self
            .holds
            .spawning
            .get(&player)
            .is_some_and(|&life| life != car.life)
        {
            self.holds.spawning.remove(&player);
            if state.is_demoed && !self.holds.wrecks.contains_key(&player) {
                state.is_demoed = false;
                state.demo_respawn_timer = 0.0;
                dirty = true;
            }
        }
        if car.body.position.is_none()
            && (car.spawn_pose.is_some() || self.holds.spawning.get(&player) == Some(&car.life))
            && !spawn_demolished
        {
            self.holds.spawning.insert(player, car.life);
            if !state.is_demoed || state.demo_respawn_timer < 3.0 {
                state.is_demoed = true;
                state.demo_respawn_timer = 3.0;
                dirty = true;
            }
        } else if car.body.position.is_some()
            && self.holds.spawning.remove(&player).is_some()
            && state.is_demoed
        {
            state.is_demoed = false;
            state.demo_respawn_timer = 0.0;
            dirty = true;
        }
        dirty
    }

    /// Sleeping updates and wrecks. A sleeping update zeroes the omitted velocities. A wreck (a demolished
    /// car's body the replay still sends: no active player link, or a goal explosion's victim) is held
    /// demolished until the player's next car life or a live update; a sleeping update of a car whose link
    /// is inactive starts the hold (inferred). Then an active car the simulation thinks demolished is
    /// corrected, unless an observed demolition or a hold explains it.
    fn wreck(
        &mut self,
        ctx: &mut FrameContext,
        car: &NetworkCar,
        player: PlayerIndex,
        state: &mut CarState,
    ) -> bool {
        let frame = ctx.index;
        let mut dirty = false;
        let sleeping_now = zero_sleeping_velocity(&mut state.phys, &car.body, frame);
        if let Some(changed) = sleeping_now {
            dirty |= changed;
            ctx.sleeping_velocity_zeroed.push(Some(car.life.actor));
            self.diagnostics.sleeping_car_updates += 1;
        }
        let live_update = car.player_link_active
            && car.body.position.as_ref().is_some_and(|p| p.frame == frame)
            && !car.body.sleeping.as_ref().is_some_and(|s| s.value);
        if let Some(&(held, _)) = self.holds.wrecks.get(&player)
            && (held != car.life || live_update)
        {
            self.holds.wrecks.remove(&player);
            // A new car held on its spawn pose stays out of collisions.
            if state.is_demoed && !self.holds.spawning.contains_key(&player) {
                state.is_demoed = false;
                state.demo_respawn_timer = 0.0;
                dirty = true;
            }
            self.diagnostics.wrecks_released += 1;
        }
        // A wreck whose sleeping update comes with the report of the demolition that made it is held from the
        // end of the interval, where that demolition is applied, so that the bump is still simulated. The
        // demolitions are applied only in a simulated, non-withheld frame; elsewhere the sleeping update
        // starts the hold.
        let demolitions_applied = ctx.simulated && !ctx.withheld;
        let reported_now =
            |events: &[NetworkEvent], report_filter: fn(&DemolitionReport, bool) -> bool| {
                events.iter().any(|event| {
                matches!(event, NetworkEvent::Demolition { report, victim_car: Some(v), repeat, .. }
                    if report_filter(report, *repeat) && *v == car.life.actor)
            })
            };
        let frame_events = &self.network.frames[frame.get()].events;
        let demolished_now = demolitions_applied
            && reported_now(frame_events, |report, repeat| {
                *report != DemolitionReport::GoalExplosion && !repeat
            });
        if sleeping_now.is_some()
            && !car.player_link_active
            && !ctx.withheld
            && !demolished_now
            && !self.holds.wrecks.contains_key(&player)
        {
            // A goal-explosion report for this car in the frame is the observed reason (counted where it is
            // applied); only a wreck without one is inferred.
            let observed = reported_now(frame_events, |report, _| {
                *report == DemolitionReport::GoalExplosion
            });
            self.holds
                .wrecks
                .insert(player, (car.life, HoldSource::Inferred));
            if !observed {
                ctx.wrecks_inferred.push(car.life.actor);
                self.diagnostics.wrecks_inferred += 1;
            }
        }
        if self
            .holds
            .wrecks
            .get(&player)
            .is_some_and(|&(life, _)| life == car.life)
            && (!state.is_demoed || state.demo_respawn_timer < 3.0)
        {
            state.is_demoed = true;
            state.demo_respawn_timer = 3.0;
            dirty = true;
        }
        if car.player_link_active
            && state.is_demoed
            && !self.holds.wrecks.contains_key(&player)
            && !self.holds.spawning.contains_key(&player)
            && self
                .holds
                .demolished_until
                .get(&player)
                .is_none_or(|&until| ctx.replay_tick >= until)
        {
            state.is_demoed = false;
            state.demo_respawn_timer = 0.0;
            self.diagnostics.active_car_demolition_corrections += 1;
            dirty = true;
        }
        dirty
    }

    /// Actions from the counters. A dodge counter turning odd with a torque is a dodge: pressed now when its
    /// impulse is not in the updates yet, else only the flip state is set. A double-jump counter turning odd
    /// is a double jump the same way. When airborne, counters that changed since the car last stood on the
    /// ground set the jump, double-jump and flip flags the updates imply.
    fn actions(
        &mut self,
        ctx: &FrameContext,
        car: &NetworkCar,
        state: &mut CarState,
        new_life: bool,
    ) -> (Press, bool) {
        let frame = ctx.index;
        let mut press = Press::default();
        let mut dirty = false;
        let pending_dodge = false;
        let track = self.cars.entry(car.life).or_default();
        let fresh = |value: &Option<NetworkValue<u8>>| {
            value.as_ref().filter(|v| v.frame == frame).map(|v| v.value)
        };
        let rising = |previous: Option<u8>, now: u8| {
            previous.map_or(now % 2 == 1, |p| p % 2 == 0 && now % 2 == 1)
        };
        if let Some(raw) = fresh(&car.inputs.dodge_active_raw) {
            let activated = rising(track.last_dodge_raw.replace(raw), raw);
            let torque = dodge_torque(&self.network.frames, frame.get(), car);
            if activated && torque.is_some() {
                self.diagnostics.dodge_activations += 1;
            }
            if activated && let Some([tx, ty, _]) = torque {
                let (pitch, yaw) = (-ty / 2.24, -tx / 2.60);
                if (pitch * pitch + yaw * yaw).sqrt() > 0.01 {
                    if dodge_impulse_unseen(car, frame, state) {
                        press = Press {
                            jump: true,
                            pitch,
                            yaw,
                        };
                    } else if !state.is_on_ground || state.phys.pos.z > 50.0 {
                        state.has_flipped = true;
                        state.is_flipping = true;
                        state.flip_rel_torque = Vec3A::new(tx / 2.60, ty / 2.24, 0.0);
                        state.flip_time = 0.0;
                        dirty = true;
                    }
                }
            }
        }
        if let Some(raw) = fresh(&car.inputs.double_jump_active_raw) {
            let activated = rising(track.last_double_jump_raw.replace(raw), raw);
            if activated && !press.jump {
                if dodge_impulse_unseen(car, frame, state) {
                    // A jump press with no direction is RocketSim's double jump.
                    press = Press {
                        jump: true,
                        pitch: 0.0,
                        yaw: 0.0,
                    };
                } else if !state.is_on_ground || state.phys.pos.z > 50.0 {
                    // The velocity update at the activation already holds the impulse: only the flags.
                    state.has_jumped = true;
                    state.has_double_jumped = true;
                    dirty = true;
                }
            }
        }
        let counter = |v: &Option<NetworkValue<u8>>| v.as_ref().map_or(0, |v| v.value);
        let current = [
            counter(&car.inputs.jump_active_raw),
            counter(&car.inputs.double_jump_active_raw),
            counter(&car.inputs.dodge_active_raw),
        ];
        if new_life {
            track.ground_counters = None;
        } else if state.is_on_ground
            && let Some(previous) = track.last_counters
        {
            track.ground_counters = Some(previous);
        }
        track.last_counters = Some(current);
        if !state.is_on_ground
            && let Some(ground) = track.ground_counters
        {
            let (jumped, double_jumped, flipped) = (
                current[0] != ground[0],
                current[1] != ground[1],
                current[2] != ground[2],
            );
            let frames = &self.network.frames;
            let time = frames[frame.get()].time;
            // Seconds since the counter last changed (the frame of its value).
            let since = |v: &Option<NetworkValue<u8>>| {
                v.as_ref()
                    .map_or(0.0, |v| (time - frames[v.frame.get()].time).max(0.0))
            };
            if !state.has_jumped && (jumped || double_jumped || flipped) {
                state.has_jumped = true;
                state.air_time_since_jump = state
                    .air_time_since_jump
                    .max(since(&car.inputs.jump_active_raw));
                dirty = true;
            }
            // An action pressed now, or a dodge planned for later, is applied by the simulation itself;
            // setting its flag first would block it.
            let acting = press.jump || pending_dodge;
            if double_jumped && !state.has_double_jumped && !acting {
                state.has_double_jumped = true;
                state.has_jumped = true;
                dirty = true;
            }
            if flipped && !state.has_flipped && !acting {
                state.has_flipped = true;
                state.has_jumped = true;
                // The time since the flip, so that the pitch lock after it ends on time.
                state.flip_time = since(&car.inputs.dodge_active_raw).min(1.0);
                dirty = true;
            }
        }
        (press, dirty)
    }

    /// The controls the car drives on with: the network values; jump only while its counter says a jump the
    /// updates have not shown; in the air, the persisted air controls of the last span or the steer as yaw
    /// (roll with the handbrake); a press from this frame's counters.
    fn controls(
        &mut self,
        ctx: &FrameContext,
        car: &NetworkCar,
        state: &CarState,
        new_life: bool,
        press: &Press,
    ) -> CarControls {
        let frame = ctx.index;
        let mut controls = network_controls(car);
        let track = self.cars.entry(car.life).or_default();
        if let Some(raw) = car
            .inputs
            .jump_active_raw
            .as_ref()
            .filter(|raw| raw.frame == frame)
        {
            track.jump_gate = Some(raw.value % 2 == 1 && jump_impulse_unseen(car, frame));
        }
        controls.jump &= track.jump_gate.unwrap_or(false);
        let airborne = !state.is_on_ground || (new_life && state.phys.pos.z > 50.0);
        let mut air_controls_applied = false;
        if airborne
            && !press.jump
            && ctx.in_play
            && let Some(air) = self.inference.air_controls(frame.get(), car, &controls)
        {
            controls.pitch = air.pitch;
            controls.yaw = air.yaw;
            controls.roll = air.roll;
            air_controls_applied = true;
        }
        if !air_controls_applied && airborne {
            if controls.handbrake {
                controls.roll = controls.steer;
            } else {
                controls.yaw = controls.steer;
            }
        }
        if press.jump {
            controls.jump = true;
            controls.pitch = press.pitch;
            controls.yaw = press.yaw;
            // The dodge direction is (-pitch, yaw + roll): a roll left over would turn it.
            controls.roll = 0.0;
        }
        controls
    }
}
