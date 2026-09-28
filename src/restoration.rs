//! Rebuild soccar RocketSim snapshots from schema-v1 rich frame records.

use std::error::Error;
use std::fmt;

use rocketsim::{
    Arena, ArenaState, BallState, BoostPadConfig, BoostPadState, CarBodyConfig, CarControls,
    CarInfo, CarState, GameMode, Mat3A, PhysState, Team, Vec3A,
};
use serde::Deserialize;

use crate::conversion::CarSlot;
use crate::serialization::{
    BallRecord, CarRecord, ControlsRecord, PhysicsRecord, ROCKETSIM_REVISION, SCHEMA_VERSION,
    StateRecord,
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RestoreError(pub String);

impl fmt::Display for RestoreError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

impl Error for RestoreError {}

fn invalid(message: impl Into<String>) -> RestoreError {
    RestoreError(message.into())
}

fn vec3(values: [f32; 3]) -> Vec3A {
    Vec3A::from_array(values)
}

impl PhysicsRecord {
    fn to_rocketsim(&self) -> PhysState {
        PhysState {
            pos: vec3(self.position),
            rot_mat: Mat3A::from_cols(
                vec3(self.rotation_columns[0]),
                vec3(self.rotation_columns[1]),
                vec3(self.rotation_columns[2]),
            ),
            vel: vec3(self.linear_velocity),
            ang_vel: vec3(self.angular_velocity),
        }
    }
}

impl ControlsRecord {
    fn to_rocketsim(&self) -> CarControls {
        CarControls {
            throttle: self.throttle,
            steer: self.steer,
            pitch: self.pitch,
            yaw: self.yaw,
            roll: self.roll,
            jump: self.jump,
            boost: self.boost,
            handbrake: self.handbrake,
        }
    }
}

impl BallRecord {
    fn to_rocketsim(&self) -> BallState {
        let mut ball = BallState::default();
        ball.phys = self.physics.to_rocketsim();
        ball.tick_count_since_kickoff = self.tick_count_since_kickoff;
        ball.last_extra_hit_tick = self.last_extra_hit_tick;
        ball.hs_info.y_target_dir = self.heatseeker_target_direction;
        ball.hs_info.cur_target_speed = self.heatseeker_target_speed;
        ball.hs_info.time_since_hit = self.heatseeker_time_since_hit;
        ball.ds_info.charge_level = self.dropshot_charge_level;
        ball.ds_info.accumulated_hit_force = self.dropshot_accumulated_hit_force;
        ball.ds_info.y_target_dir = self.dropshot_target_direction;
        ball.ds_info.last_damage_tick = self.dropshot_last_damage_tick;
        ball
    }
}

impl CarRecord {
    fn to_rocketsim(&self) -> CarState {
        CarState {
            phys: self.physics.to_rocketsim(),
            controls: self.controls.to_rocketsim(),
            prev_controls: self.previous_controls.to_rocketsim(),
            boost: self.boost,
            is_boosting: self.is_boosting,
            boosting_time: self.boosting_time,
            time_since_boosted: self.time_since_boosted,
            is_on_ground: self.is_on_ground,
            wheels_with_contact: self.wheels_with_contact,
            has_jumped: self.has_jumped,
            has_double_jumped: self.has_double_jumped,
            has_flipped: self.has_flipped,
            flip_rel_torque: vec3(self.flip_relative_torque),
            jump_ticks: self.jump_ticks,
            flip_time: self.flip_time,
            is_flipping: self.is_flipping,
            is_jumping: self.is_jumping,
            air_time: self.air_time,
            air_time_since_jump: self.air_time_since_jump,
            is_supersonic: self.is_supersonic,
            supersonic_grace_timer: self.supersonic_grace_timer,
            handbrake_val: self.handbrake_value,
            is_auto_flipping: self.is_auto_flipping,
            auto_flip_timer: self.auto_flip_timer,
            auto_flip_torque_scale: self.auto_flip_torque_scale,
            bump_cooldown_timer: self.bump_cooldown_timer,
            world_contact_normal: self.world_contact_normal.map(vec3),
            is_demoed: self.is_demoed,
            demo_respawn_timer: self.demo_respawn_timer,
        }
    }
}

fn hitbox(name: &str) -> Result<CarBodyConfig, RestoreError> {
    match name {
        "octane" => Ok(CarBodyConfig::OCTANE),
        "dominus" => Ok(CarBodyConfig::DOMINUS),
        "plank" => Ok(CarBodyConfig::PLANK),
        "breakout" => Ok(CarBodyConfig::BREAKOUT),
        "hybrid" => Ok(CarBodyConfig::HYBRID),
        "merc" => Ok(CarBodyConfig::MERC),
        "psyclops" => Ok(CarBodyConfig::PSYCLOPS),
        _ => Err(invalid(format!("unsupported car hitbox: {name}"))),
    }
}

fn team(index: u8) -> Result<Team, RestoreError> {
    match index {
        0 => Ok(Team::Blue),
        1 => Ok(Team::Orange),
        _ => Err(invalid(format!("invalid team index: {index}"))),
    }
}

/// Read only the state field from a complete JSONL frame or Parquet `frame_json`.
pub fn state_from_frame_json(json: &[u8]) -> Result<StateRecord, Box<dyn Error>> {
    #[derive(Deserialize)]
    struct Frame {
        record_type: String,
        state: StateRecord,
    }
    let frame: Frame = serde_json::from_slice(json)?;
    if frame.record_type != "frame" {
        return Err(Box::new(invalid("expected a frame record")));
    }
    Ok(frame.state)
}

/// Read car identities and hitboxes from a schema-v1 JSONL header or Parquet metadata.
pub fn car_slots_from_header_json(json: &[u8]) -> Result<Vec<CarSlot>, Box<dyn Error>> {
    #[derive(Deserialize)]
    struct Header {
        record_type: String,
        schema_version: u32,
        rocketsim_revision: String,
        header: ReplayMode,
        car_slots: Vec<CarSlot>,
    }
    #[derive(Deserialize)]
    struct ReplayMode {
        game_type: String,
    }
    let header: Header = serde_json::from_slice(json)?;
    if header.record_type != "header"
        || header.schema_version != SCHEMA_VERSION
        || header.rocketsim_revision != ROCKETSIM_REVISION
        || header.header.game_type != "TAGame.Replay_Soccar_TA"
    {
        return Err(Box::new(invalid("unsupported replay state header")));
    }
    Ok(header.car_slots)
}

/// Rebuild an exact detached soccar `ArenaState` snapshot. This preserves the
/// serialized tick and all public soccar ball/car/pad state fields. It does not
/// restore RocketSim's private live Arena physics caches or RNG state.
pub fn restore_soccar_state(
    record: &StateRecord,
    slots: &[CarSlot],
) -> Result<ArenaState, RestoreError> {
    if record.cars.len() > slots.len() {
        return Err(invalid("state has more cars than header slots"));
    }
    let mut state = ArenaState::new_empty(GameMode::Soccar);
    state.tick_count = record.arena_tick;
    state.ball = record.ball.to_rocketsim();
    for (index, car) in record.cars.iter().enumerate() {
        let slot = &slots[index];
        if slot.slot != index || car.slot != index || car.team != slot.team {
            return Err(invalid(format!("car slot/team mismatch at index {index}")));
        }
        state.cars.push((
            CarInfo {
                idx: index,
                team: team(slot.team)?,
                config: hitbox(&slot.hitbox)?,
            },
            car.to_rocketsim(),
        ));
    }
    for (index, pad) in record.boost_pads.iter().enumerate() {
        if pad.is_active != (pad.cooldown <= 0.0) || !pad.cooldown.is_finite() {
            return Err(invalid(format!(
                "boost pad {index} has inconsistent cooldown"
            )));
        }
        state.boost_pads.push((
            BoostPadConfig {
                pos: vec3(pad.position),
                is_big: pad.is_big,
            },
            BoostPadState {
                cooldown: pad.cooldown,
            },
        ));
    }
    Ok(state)
}

/// What a live Arena could not preserve when seeded from a serialized snapshot.
#[derive(Debug, Clone, Copy)]
pub struct ArenaApplyReport {
    pub source_tick: u64,
    pub arena_tick: u64,
    /// RocketSim quantizes pad pickup time to a whole 120 Hz tick.
    pub max_pad_cooldown_error_seconds: f32,
}

/// Seed an already initialized soccar Arena with the snapshot's public states.
/// Absolute tick, RNG, and private collision/wheel caches cannot be set through
/// the pinned RocketSim API; use the detached `ArenaState` for exact inspection.
pub fn apply_soccar_state_to_arena(
    snapshot: &ArenaState,
    arena: &mut Arena,
) -> Result<ArenaApplyReport, RestoreError> {
    if arena.game_mode() != GameMode::Soccar {
        return Err(invalid("expected a soccar arena"));
    }
    if (arena.num_cars() != 0 && arena.num_cars() != snapshot.cars.len())
        || arena.num_boost_pads() != snapshot.boost_pads.len()
    {
        return Err(invalid("arena car or pad count does not match snapshot"));
    }
    for (index, (info, _)) in snapshot.cars.iter().enumerate() {
        if info.idx != index {
            return Err(invalid(format!("nonsequential car index at slot {index}")));
        }
        if arena.num_cars() != 0 {
            let actual = arena.get_car_info(index);
            if actual.idx != info.idx || actual.team != info.team || actual.config != info.config {
                return Err(invalid(format!("arena car config differs at slot {index}")));
            }
        }
    }
    for (index, (config, _)) in snapshot.boost_pads.iter().enumerate() {
        let actual = arena.get_boost_pad_config(index);
        if actual.is_big != config.is_big || (actual.pos - config.pos).length() > 0.01 {
            return Err(invalid(format!(
                "arena pad config differs at index {index}"
            )));
        }
    }
    if arena.num_cars() == 0 {
        for (info, _) in &snapshot.cars {
            let actual = arena.add_car(info.team, info.config);
            if actual != info.idx {
                return Err(invalid("arena assigned an unexpected car index"));
            }
        }
    }
    arena.set_ball_state(snapshot.ball);
    for (index, (_, car)) in snapshot.cars.iter().enumerate() {
        arena.set_car_state(index, *car);
    }
    let mut max_pad_error = 0.0f32;
    for (index, (_, pad)) in snapshot.boost_pads.iter().enumerate() {
        arena.set_boost_pad_state(index, *pad);
        max_pad_error =
            max_pad_error.max((arena.get_boost_pad_state(index).cooldown - pad.cooldown).abs());
    }
    Ok(ArenaApplyReport {
        source_tick: snapshot.tick_count,
        arena_tick: arena.tick_count(),
        max_pad_cooldown_error_seconds: max_pad_error,
    })
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;
    use crate::serialization::StateRecord;

    #[test]
    fn soccar_state_round_trips_and_seeds_live_arena() {
        let meshes = Path::new(env!("CARGO_MANIFEST_DIR")).join("collision_meshes");
        if !meshes.join("soccar").is_dir() {
            eprintln!("skipping state restoration test: no local collision meshes");
            return;
        }
        rocketsim::init(&meshes, true).unwrap();
        let mut source = Arena::new(GameMode::Soccar);
        source.add_car(Team::Blue, CarBodyConfig::DOMINUS);
        source.add_car(Team::Orange, CarBodyConfig::OCTANE);
        let mut original = source.get_arena_state();
        original.tick_count = 123;
        original.ball.phys.pos = Vec3A::new(12.5, -31.25, 114.0);
        original.ball.phys.vel = Vec3A::new(100.0, 200.0, -12.0);
        original.ball.last_extra_hit_tick = Some(119);
        original.ball.hs_info.cur_target_speed = 2300.0;
        original.cars[0].1.phys.pos = Vec3A::new(-400.0, 250.0, 38.0);
        original.cars[0].1.phys.rot_mat = Mat3A::from_rotation_z(0.5);
        original.cars[0].1.phys.ang_vel = Vec3A::new(0.2, -0.3, 1.0);
        original.cars[0].1.controls.pitch = -0.5;
        original.cars[0].1.controls.boost = true;
        original.cars[0].1.prev_controls.jump = true;
        original.cars[0].1.boost = 42.0;
        original.cars[0].1.has_flipped = true;
        original.cars[0].1.flip_rel_torque = Vec3A::new(0.1, 0.2, 0.3);
        original.cars[0].1.jump_ticks = 17;
        original.cars[0].1.world_contact_normal = Some(Vec3A::Z);
        original.boost_pads[0].1.cooldown = 2.0;
        let slots = vec![
            CarSlot {
                slot: 0,
                player_key: "blue".into(),
                team: 0,
                body_product_id: None,
                hitbox: "dominus".into(),
            },
            CarSlot {
                slot: 1,
                player_key: "orange".into(),
                team: 1,
                body_product_id: None,
                hitbox: "octane".into(),
            },
        ];
        let record = StateRecord::from_arena_state(&original);
        let line = serde_json::json!({"record_type": "frame", "state": record});
        let decoded = state_from_frame_json(&serde_json::to_vec(&line).unwrap()).unwrap();
        let restored = restore_soccar_state(&decoded, &slots).unwrap();
        assert_eq!(
            serde_json::to_value(&record).unwrap(),
            serde_json::to_value(StateRecord::from_arena_state(&restored)).unwrap()
        );
        assert_eq!(restored.cars[0].0.config, CarBodyConfig::DOMINUS);
        let header = serde_json::json!({
            "record_type": "header", "schema_version": SCHEMA_VERSION,
            "rocketsim_revision": ROCKETSIM_REVISION,
            "header": {"game_type": "TAGame.Replay_Soccar_TA"},
            "car_slots": slots,
        });
        let parsed_slots =
            car_slots_from_header_json(&serde_json::to_vec(&header).unwrap()).unwrap();
        assert_eq!(parsed_slots, slots);
        let mut live = Arena::new(GameMode::Soccar);
        let report = apply_soccar_state_to_arena(&restored, &mut live).unwrap();
        assert_eq!(report.source_tick, 123);
        assert_eq!(report.arena_tick, 0);
        assert!(report.max_pad_cooldown_error_seconds <= 1.0 / 120.0);
        let mut live_record = StateRecord::from_arena_state(&live.get_arena_state());
        live_record.arena_tick = 123;
        live_record.boost_pads[0].cooldown = record.boost_pads[0].cooldown;
        assert_eq!(
            serde_json::to_value(record).unwrap(),
            serde_json::to_value(live_record).unwrap()
        );
    }

    #[test]
    fn rejects_inconsistent_pad_and_slot() {
        let mut record = StateRecord {
            arena_tick: 0,
            ball: (&BallState::default()).into(),
            cars: Vec::new(),
            boost_pads: vec![crate::serialization::PadRecord {
                position: [0.0; 3],
                is_big: false,
                cooldown: 1.0,
                is_active: true,
            }],
        };
        assert!(restore_soccar_state(&record, &[]).is_err());
        record.boost_pads[0].is_active = false;
        assert!(restore_soccar_state(&record, &[]).is_ok());

        let mut source = ArenaState::new_empty(GameMode::Soccar);
        source.cars.push((
            CarInfo {
                idx: 0,
                team: Team::Blue,
                config: CarBodyConfig::OCTANE,
            },
            CarState::default(),
        ));
        let car_record = StateRecord::from_arena_state(&source);
        let wrong_team = [CarSlot {
            slot: 0,
            player_key: "player".into(),
            team: 1,
            body_product_id: None,
            hitbox: "octane".into(),
        }];
        assert!(restore_soccar_state(&car_record, &wrong_team).is_err());
    }
}
