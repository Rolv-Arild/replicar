"""Read schema-v1 replay-to-rocketsim JSON Lines exports.

``iter_frames`` streams rich records. ``load_numpy`` builds dense arrays for ML;
NumPy is imported only when that function is called.
"""

from __future__ import annotations

import gzip
import json
from pathlib import Path
from typing import Any, Iterator

# Codes of `load_numpy`'s `dead_shell_held` array (0: not held).
DEAD_SHELL_CODES = {"observed": 1, "inferred": 2}


SCHEMA_VERSION = 1


def _lines(path: str | Path) -> Iterator[dict[str, Any]]:
    opener = gzip.open if Path(path).suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as source:
        for line_number, line in enumerate(source, 1):
            if not line.strip():
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"invalid JSON at line {line_number}: {error}") from error


def _header(lines: Iterator[dict[str, Any]]) -> dict[str, Any]:
    header = next(lines, None)
    if header is None or header.get("record_type") != "header":
        raise ValueError("first record must be a header")
    if header.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"unsupported schema version: {header.get('schema_version')}")
    return header


def read_header(path: str | Path) -> dict[str, Any]:
    """Return metadata, player slots, dependency revisions and diagnostics."""
    return _header(_lines(path))


def iter_frames(path: str | Path) -> Iterator[dict[str, Any]]:
    """Yield each frame's RocketSim snapshot and replay observations in order."""
    lines = _lines(path)
    _header(lines)
    for record in lines:
        if record.get("record_type") != "frame":
            raise ValueError(f"unexpected record type: {record.get('record_type')}")
        yield record


def load_numpy(path: str | Path) -> dict[str, Any]:
    """Load dense per-frame arrays. Missing cars use NaNs and a false ``car_present`` mask.

    Car columns are ordered by ``car_slots``. ``car_present`` means the slot exists
    in the RocketSim snapshot; use ``car_demoed`` and the rich observations for
    finer lifecycle meaning. Score and clock use NaN when missing from replay.
    Control arrays contain the inputs passed to RocketSim, including inferred
    boost; use ``iter_frames`` for original action counters and provenance.

    The ``scoreboard_*`` entries are the reconstructed match clock: ``scoreboard_period``
    (``regulation``/``overtime``) and ``scoreboard_clock_state`` (``pregame``, ``countdown``,
    ``kickoff``, ``running``, ``expired``, ``decided``, ``goal_pause``, ``other``) are object arrays
    with ``None`` where the frame has no scoreboard; the two clock arrays use NaN for unknown.

    ``dead_shell_held`` (frames x car slots, uint8) marks the frames in which a slot is held
    demolished as a dead pawn shell: 0 not held, 1 by an observed goal-explosion demolition, 2 by an
    inference from a sleeping packet of a car with no active pawn link (``DEAD_SHELL_CODES``). Whether
    a car sleeps (``sleeping_velocity_inferred``) and the start of an inferred hold
    (``demolition_inferred``) are in the rich frame only (``iter_frames``).
    """
    import numpy as np

    header = read_header(path)
    slots = header["car_slots"]
    slot_columns = {slot["slot"]: index for index, slot in enumerate(slots)}
    count = 0
    first_state = None
    for frame in iter_frames(path):
        if first_state is None:
            first_state = frame["state"]
        count += 1
    car_count = len(slots)
    pad_config = first_state["boost_pads"] if first_state is not None else []
    pad_count = len(pad_config)
    time = np.empty(count, dtype=np.float64)
    timeline_tick = np.empty(count, dtype=np.uint64)
    arena_tick = np.empty(count, dtype=np.uint64)
    ball_position = np.full((count, 3), np.nan, dtype=np.float32)
    ball_rotation = np.full((count, 3, 3), np.nan, dtype=np.float32)
    ball_velocity = np.full((count, 3), np.nan, dtype=np.float32)
    ball_angular_velocity = np.full((count, 3), np.nan, dtype=np.float32)
    car_position = np.full((count, car_count, 3), np.nan, dtype=np.float32)
    car_rotation = np.full((count, car_count, 3, 3), np.nan, dtype=np.float32)
    car_velocity = np.full((count, car_count, 3), np.nan, dtype=np.float32)
    car_angular_velocity = np.full((count, car_count, 3), np.nan, dtype=np.float32)
    car_boost = np.full((count, car_count), np.nan, dtype=np.float32)
    car_present = np.zeros((count, car_count), dtype=np.bool_)
    car_demoed = np.zeros((count, car_count), dtype=np.bool_)
    control_axes = np.full((count, car_count, 5), np.nan, dtype=np.float32)
    control_buttons = np.zeros((count, car_count, 3), dtype=np.bool_)
    boost_pad_position = np.array([pad["position"] for pad in pad_config], dtype=np.float32).reshape(pad_count, 3)
    boost_pad_is_big = np.array([pad["is_big"] for pad in pad_config], dtype=np.bool_)
    boost_pad_active = np.zeros((count, pad_count), dtype=np.bool_)
    boost_pad_cooldown = np.full((count, pad_count), np.nan, dtype=np.float32)
    scores = np.full((count, 2), np.nan, dtype=np.float32)
    seconds_remaining = np.full(count, np.nan, dtype=np.float32)
    scoreboard_period = np.full(count, None, dtype=object)
    scoreboard_clock_state = np.full(count, None, dtype=object)
    scoreboard_seconds_remaining = np.full(count, np.nan, dtype=np.float32)
    scoreboard_overtime_seconds = np.full(count, np.nan, dtype=np.float32)
    dead_shell_held = np.zeros((count, car_count), dtype=np.uint8)
    for row, frame in enumerate(iter_frames(path)):
        state = frame["state"]
        time[row] = frame["replay_time"]
        timeline_tick[row] = frame["timeline_tick"]
        arena_tick[row] = state["arena_tick"]
        ball_position[row] = state["ball"]["physics"]["position"]
        ball_rotation[row] = state["ball"]["physics"]["rotation_columns"]
        ball_velocity[row] = state["ball"]["physics"]["linear_velocity"]
        ball_angular_velocity[row] = state["ball"]["physics"]["angular_velocity"]
        for car in state["cars"]:
            column = slot_columns[car["slot"]]
            car_present[row, column] = True
            car_position[row, column] = car["physics"]["position"]
            car_rotation[row, column] = car["physics"]["rotation_columns"]
            car_velocity[row, column] = car["physics"]["linear_velocity"]
            car_angular_velocity[row, column] = car["physics"]["angular_velocity"]
            car_boost[row, column] = car["boost"]
            car_demoed[row, column] = car["is_demoed"]
            controls = car["controls"]
            control_axes[row, column] = [controls[key] for key in ("throttle", "steer", "pitch", "yaw", "roll")]
            control_buttons[row, column] = [controls[key] for key in ("jump", "boost", "handbrake")]
        if len(state["boost_pads"]) != pad_count:
            raise ValueError(f"boost pad count changed at frame {frame['frame']}")
        for pad_index, pad in enumerate(state["boost_pads"]):
            boost_pad_active[row, pad_index] = pad["is_active"]
            boost_pad_cooldown[row, pad_index] = pad["cooldown"]
        observed = frame["observations"]
        for team, score in enumerate(observed["team_scores"]):
            if score is not None:
                scores[row, team] = score["value"]
        clock = observed["seconds_remaining"]
        if clock is not None:
            seconds_remaining[row] = clock["value"]
        for held in frame.get("dead_shell_held", ()):
            dead_shell_held[row, slot_columns[held["slot"]]] = DEAD_SHELL_CODES[held["source"]]
        board = frame.get("scoreboard")
        if board is not None:
            scoreboard_period[row] = board["period"]
            scoreboard_clock_state[row] = board["clock_state"]
            if board["seconds_remaining"] is not None:
                scoreboard_seconds_remaining[row] = board["seconds_remaining"]
            if board["overtime_seconds"] is not None:
                scoreboard_overtime_seconds[row] = board["overtime_seconds"]
    return {
        "header": header,
        "time": time,
        "timeline_tick": timeline_tick,
        "arena_tick": arena_tick,
        "ball_position": ball_position,
        "ball_rotation_columns": ball_rotation,
        "ball_velocity": ball_velocity,
        "ball_angular_velocity": ball_angular_velocity,
        "car_position": car_position,
        "car_rotation_columns": car_rotation,
        "car_velocity": car_velocity,
        "car_angular_velocity": car_angular_velocity,
        "car_boost": car_boost,
        "car_present": car_present,
        "car_demoed": car_demoed,
        "control_axes": control_axes,
        "control_axes_order": ("throttle", "steer", "pitch", "yaw", "roll"),
        "control_buttons": control_buttons,
        "control_buttons_order": ("jump", "boost", "handbrake"),
        "boost_pad_position": boost_pad_position,
        "boost_pad_is_big": boost_pad_is_big,
        "boost_pad_active": boost_pad_active,
        "boost_pad_cooldown": boost_pad_cooldown,
        "scores": scores,
        "seconds_remaining": seconds_remaining,
        "scoreboard_period": scoreboard_period,
        "scoreboard_clock_state": scoreboard_clock_state,
        "scoreboard_seconds_remaining": scoreboard_seconds_remaining,
        "scoreboard_overtime_seconds": scoreboard_overtime_seconds,
        "dead_shell_held": dead_shell_held,
    }
