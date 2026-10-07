"""A replicar row as a state of mtheall's RocketSim bindings (the `rocketsim` package, module `RocketSim`, as RLGym
uses it), to continue a match from any frame or to start an RLGym environment there.

    import RocketSim, replicar, replicar.rocketsim
    RocketSim.init("collision_meshes")
    f = replicar.read("match.parquet")
    arena, cars = replicar.rocketsim.arena(f, row=1200)   # cars: {player index: RocketSim.Car}
    arena.step(8)

replicar simulates with the Rust port of RocketSim (the `rocketsim` crate); the bindings are the C++ RocketSim. They
model the same game, but a state stepped in the bindings is not replicar's: one frame ahead, a car is about 0.2 UU
apart at the median (docs/v2-guide.md, "Continuing in RocketSim"), and the difference grows with time. What the bindings' state has and a replicar row does not is left at its
default: the ball's heatseeker state, a car's flip-reset flags, the car a bump cooldown is for, and the tick of a car's
last extra ball-hit impulse.
"""

from __future__ import annotations

from typing import Any

import numpy as np

#: How far (UU) a pad may be from the file's to be the same pad (RLGym's table has one 2 UU off; pads are hundreds
#: of UU apart).
PAD_TOLERANCE = 10.0

#: replicar's hitbox names and the bindings' `CarConfig` presets (the bindings have no `psyclops`).
HITBOXES = {
    "octane": "OCTANE",
    "dominus": "DOMINUS",
    "plank": "PLANK",
    "breakout": "BREAKOUT",
    "hybrid": "HYBRID",
    "merc": "MERC",
}


def _module():
    try:
        import RocketSim
    except ImportError as error:
        raise ImportError(
            "replicar.rocketsim needs mtheall's RocketSim bindings: pip install rocketsim"
        ) from error
    return RocketSim


def rotation_matrix(rotation: np.ndarray) -> np.ndarray:
    """A unit quaternion (x, y, z, w) as the rotation matrix whose columns are the forward, right and up axes."""
    x, y, z, w = (float(v) for v in rotation)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _rot_mat(rs, rotation: np.ndarray):
    """The bindings' rotation matrix, given by its forward, right and up axes."""
    return rs.RotMat(*rotation_matrix(rotation).T.flatten())


def _vec(rs, values: np.ndarray):
    return rs.Vec(*(float(v) for v in values))


def _controls(rs, a: dict[str, np.ndarray], prefix: str, row: int, p: int):
    controls = rs.CarControls()
    for name in ("throttle", "steer", "pitch", "yaw", "roll"):
        controls.__setattr__(name, float(a[f"car_{prefix}_{name}"][row, p]))
    for name in ("jump", "boost", "handbrake"):
        controls.__setattr__(name, bool(a[f"car_{prefix}_{name}"][row, p] == 1))
    return controls


def _arrays(file) -> dict[str, np.ndarray]:
    a = file.arrays()
    if "car_position" not in a or "ball_position" not in a:
        raise ValueError(f"{file.path}: no state group (read the file with replay=... to rebuild it)")
    return a


def has_car(file, row: int, player: int) -> bool:
    """The player has a car in the row."""
    a = _arrays(file)
    return a["car_status"][row, player] not in ("", "absent") and not np.isnan(a["car_position"][row, player, 0])


def ball_state(file, row: int):
    """The row's ball as a `RocketSim.BallState`."""
    rs = _module()
    a = _arrays(file)
    state = rs.BallState()
    state.pos = _vec(rs, a["ball_position"][row])
    state.vel = _vec(rs, a["ball_velocity"][row])
    state.ang_vel = _vec(rs, a["ball_angular_velocity"][row])
    state.rot_mat = _rot_mat(rs, a["ball_rotation"][row])
    return state


def car_state(file, row: int, player: int):
    """The row's car of `player` as a `RocketSim.CarState`, with its previous controls as `last_controls`."""
    rs = _module()
    a = _arrays(file)
    if not has_car(file, row, player):
        raise ValueError(f"player {player} has no car in row {row}")
    p = player
    flag = lambda name: bool(a[f"car_{name}"][row, p] == 1)  # noqa: E731
    number = lambda name: float(a[f"car_{name}"][row, p])  # noqa: E731
    state = rs.CarState()
    state.pos = _vec(rs, a["car_position"][row, p])
    state.vel = _vec(rs, a["car_velocity"][row, p])
    state.ang_vel = _vec(rs, a["car_angular_velocity"][row, p])
    state.rot_mat = _rot_mat(rs, a["car_rotation"][row, p])
    state.boost = number("boost")
    state.is_on_ground = flag("is_on_ground")
    state.wheels_with_contact = tuple(bool(a[f"car_wheel_{k}_contact"][row, p] == 1) for k in range(4))
    state.has_jumped = flag("has_jumped")
    state.has_double_jumped = flag("has_double_jumped")
    state.has_flipped = flag("has_flipped")
    state.flip_rel_torque = _vec(rs, a["car_flip_relative_torque"][row, p])
    # The Rust port counts the jump in ticks; the bindings in seconds.
    state.jump_time = float(a["car_jump_ticks"][row, p]) / 120.0
    state.flip_time = number("flip_time")
    state.is_flipping = flag("is_flipping")
    state.is_jumping = flag("is_jumping")
    state.air_time = number("air_time")
    state.air_time_since_jump = number("air_time_since_jump")
    state.time_spent_boosting = number("boosting_time")
    state.is_supersonic = flag("is_supersonic")
    # The bindings keep a car supersonic below the start speed while `supersonic_time` is under the maintain time;
    # the port counts the time spent below it: the same time left.
    state.supersonic_time = number("supersonic_grace_timer")
    state.handbrake_val = number("handbrake_value")
    state.is_auto_flipping = flag("is_auto_flipping")
    state.auto_flip_timer = number("auto_flip_timer")
    state.auto_flip_torque_scale = number("auto_flip_torque_scale")
    state.car_contact_cooldown_timer = number("bump_cooldown_timer")
    normal = a["car_world_contact_normal"][row, p]
    state.has_world_contact = not np.isnan(normal[0])
    if state.has_world_contact:
        state.world_contact_normal = _vec(rs, normal)
    state.is_demoed = flag("is_demoed")
    state.demo_respawn_timer = number("demo_respawn_timer")
    state.last_controls = _controls(rs, a, "previous_controls", row, p)
    return state


def car_controls(file, row: int, player: int):
    """The controls the row's car of `player` drives on with from this row, as `RocketSim.CarControls`."""
    return _controls(_module(), _arrays(file), "controls", row, player)


def _pad_order(file, arena) -> list[int]:
    """For each of the arena's pads, the header's pad at the same place."""
    header = np.array([pad["position"][:2] for pad in file.header["pads"]], dtype=np.float64)
    order = []
    for pad in arena.get_boost_pads():
        position = pad.get_pos()
        distance = np.hypot(header[:, 0] - position.x, header[:, 1] - position.y)
        nearest = int(np.argmin(distance))
        if distance[nearest] > PAD_TOLERANCE:
            raise ValueError(f"no pad of the file at the arena's pad ({position.x}, {position.y})")
        order.append(nearest)
    return order


def set_state(arena, cars: dict[int, Any], file, row: int) -> None:
    """Writes the row into `arena`: the ball, each car of `cars` (player index: `RocketSim.Car`; its state and
    controls) and the pads' cooldowns."""
    rs = _module()
    a = _arrays(file)
    arena.ball.set_state(ball_state(file, row))
    for player, car in cars.items():
        car.set_state(car_state(file, row, player))
        car.set_controls(car_controls(file, row, player))
    cooldowns = a.get("pad_cooldown")
    if cooldowns is not None:
        for pad, k in zip(arena.get_boost_pads(), _pad_order(file, arena)):
            state = rs.BoostPadState()
            cooldown = float(cooldowns[row, k])
            state.cooldown = cooldown
            state.is_active = cooldown <= 0.0
            pad.set_state(state)


def arena(file, row: int, *, game_mode=None, hitboxes: dict[str, str] | None = None) -> tuple[Any, dict[int, Any]]:
    """A new `RocketSim.Arena` (soccar unless `game_mode`) in the row's state, with a car for every player who has
    one in the row (their hitbox and team), and those cars by player index. `hitboxes` adds or replaces entries of
    `HITBOXES` (`{"psyclops": "OCTANE"}` to accept a stand-in). `RocketSim.init` must have been called."""
    rs = _module()
    presets = {**HITBOXES, **(hitboxes or {})}
    new = rs.Arena(rs.GameMode.SOCCAR if game_mode is None else game_mode)
    cars = {}
    for info in file.players:
        player = info["index"]
        if not has_car(file, row, player):
            continue
        if info["hitbox"] not in presets:
            raise ValueError(f"the bindings have no {info['hitbox']} hitbox (player {player}); pass hitboxes=")
        preset = getattr(rs.CarConfig, presets[info["hitbox"]])
        team = rs.Team.BLUE if info["team"] == 0 else rs.Team.ORANGE
        cars[player] = new.add_car(team, rs.CarConfig(preset))
    set_state(new, cars, file, row)
    return new, cars
