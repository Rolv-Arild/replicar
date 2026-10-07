"""A replicar row as an RLGym `GameState` (rlgym 2, `rlgym.rocket_league`), for `RocketSimEngine.set_state` or a
state mutator: an RLGym environment that starts from any replay frame.

    import replicar, replicar.rlgym
    f = replicar.read("match.parquet")
    state = replicar.rlgym.game_state(f, row=1200)        # cars keyed by player index
    engine.set_state(state, {})

A `GameState` has less than a RocketSim state: no controls (an environment's actions supply them), of the previous
controls only the jump (`is_holding_jump`), and no air time, world contact or bump cooldown. `RocketSimEngine.set_state`
fills a RocketSim arena from it; for everything a row has, use `replicar.rocketsim` instead. The derived values follow
RLGym's definitions: `is_demoed` is a positive `demo_respawn_timer`, `on_ground` three wheels in contact, and
`is_supersonic` a positive `supersonic_time`.
"""

from __future__ import annotations

from typing import Any, Hashable

import numpy as np

from .rocketsim import PAD_TOLERANCE, has_car, rotation_matrix

#: replicar's hitbox names and RLGym's hitbox constants (`rlgym.rocket_league.common_values`; it has no `psyclops`).
HITBOXES = {"octane": 0, "dominus": 1, "plank": 2, "breakout": 3, "hybrid": 4, "merc": 5}

#: Seconds of one tick: a supersonic car gets at least this much `supersonic_time`, else RLGym counts it as not
#: supersonic.
_TICK = 1.0 / 120.0


def _modules():
    try:
        from rlgym.rocket_league import common_values
        from rlgym.rocket_league.api import Car, GameConfig, GameState, PhysicsObject
    except ImportError as error:
        raise ImportError("replicar.rlgym needs RLGym 2: pip install replicar[rlgym]") from error
    return common_values, Car, GameConfig, GameState, PhysicsObject


def _physics(PhysicsObject, position, velocity, angular_velocity, rotation):
    physics = PhysicsObject()
    physics.position = np.asarray(position, dtype=np.float32)
    physics.linear_velocity = np.asarray(velocity, dtype=np.float32)
    physics.angular_velocity = np.asarray(angular_velocity, dtype=np.float32)
    physics.rotation_mtx = rotation_matrix(rotation).astype(np.float32)
    return physics


def game_state(
    file,
    row: int,
    *,
    agent_ids: dict[int, Hashable] | None = None,
    hitboxes: dict[str, int] | None = None,
) -> Any:
    """The row as a `GameState`: the ball, a `Car` per player with a car in the row (keyed by `agent_ids[player]`,
    else the player index), the pad timers in RLGym's pad order (`BOOST_LOCATIONS`), the row's sim tick as
    `tick_count`, and RLGym's default `GameConfig` (standard gravity, boost consumption and dodge deadzone).
    `hitboxes` adds or replaces entries of `HITBOXES`."""
    common_values, Car, GameConfig, GameState, PhysicsObject = _modules()
    a = file.arrays()
    if "car_position" not in a or "ball_position" not in a:
        raise ValueError(f"{file.path}: no state group (read the file with replay=... to rebuild it)")
    presets = {**HITBOXES, **(hitboxes or {})}

    state = GameState()
    state.tick_count = int(a["sim_tick"][row])
    state.config = GameConfig()
    state.config.gravity = 1.0
    state.config.boost_consumption = 1.0
    state.config.dodge_deadzone = 0.5
    state.ball = _physics(
        PhysicsObject,
        a["ball_position"][row],
        a["ball_velocity"][row],
        a["ball_angular_velocity"][row],
        a["ball_rotation"][row],
    )
    state.goal_scored = bool(abs(a["ball_position"][row][1]) > common_values.GOAL_THRESHOLD)

    state.cars = {}
    for info in file.players:
        p = info["index"]
        if not has_car(file, row, p):
            continue
        if info["hitbox"] not in presets:
            raise ValueError(f"RLGym has no {info['hitbox']} hitbox (player {p}); pass hitboxes=")
        flag = lambda name: bool(a[f"car_{name}"][row, p] == 1)  # noqa: E731
        number = lambda name: float(a[f"car_{name}"][row, p])  # noqa: E731
        car = Car()
        car.team_num = int(info["team"])
        car.hitbox_type = presets[info["hitbox"]]
        car.ball_touches = 0
        car.bump_victim_id = None
        car.physics = _physics(
            PhysicsObject,
            a["car_position"][row, p],
            a["car_velocity"][row, p],
            a["car_angular_velocity"][row, p],
            a["car_rotation"][row, p],
        )
        car.demo_respawn_timer = number("demo_respawn_timer") if flag("is_demoed") else 0.0
        car.wheels_with_contact = tuple(bool(a[f"car_wheel_{k}_contact"][row, p] == 1) for k in range(4))
        # As in replicar.rocketsim: the time left in the grace below start speed, and at least a tick while supersonic.
        car.supersonic_time = max(number("supersonic_grace_timer"), _TICK) if flag("is_supersonic") else 0.0
        car.boost_amount = number("boost")
        car.boost_active_time = number("boosting_time")
        car.handbrake = number("handbrake_value")
        car.is_jumping = flag("is_jumping")
        car.has_jumped = flag("has_jumped")
        car.is_holding_jump = flag("previous_controls_jump")
        car.jump_time = float(a["car_jump_ticks"][row, p]) / 120.0
        car.has_flipped = flag("has_flipped")
        car.has_double_jumped = flag("has_double_jumped")
        car.air_time_since_jump = number("air_time_since_jump")
        car.flip_time = number("flip_time")
        car.flip_torque = np.asarray(a["car_flip_relative_torque"][row, p], dtype=np.float32)
        car.is_autoflipping = flag("is_auto_flipping")
        car.autoflip_timer = number("auto_flip_timer")
        car.autoflip_direction = number("auto_flip_torque_scale")
        state.cars[agent_ids[p] if agent_ids else p] = car

    # RLGym indexes the pad timers in its BOOST_LOCATIONS order: match the header's pads by position.
    header = np.array([pad["position"][:2] for pad in file.header["pads"]], dtype=np.float64)
    locations = np.array(common_values.BOOST_LOCATIONS, dtype=np.float64)[:, :2]
    timers = np.zeros(len(locations), dtype=np.float32)
    cooldowns = a.get("pad_cooldown")
    for i, (x, y) in enumerate(locations):
        distance = np.hypot(header[:, 0] - x, header[:, 1] - y)
        k = int(np.argmin(distance))
        if distance[k] > PAD_TOLERANCE:
            raise ValueError(f"no pad of the file at RLGym's pad {i} ({x}, {y})")
        timers[i] = 0.0 if cooldowns is None else max(float(cooldowns[row, k]), 0.0)
    state.boost_pad_timers = timers
    return state
