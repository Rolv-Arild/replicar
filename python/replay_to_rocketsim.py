"""Read schema-v1 replay-to-rocketsim JSON Lines exports.

``iter_frames`` streams rich records. ``load_numpy`` builds dense arrays for ML;
NumPy is imported only when that function is called.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterator


SCHEMA_VERSION = 1


def _lines(path: str | Path) -> Iterator[dict[str, Any]]:
    with open(path, "r", encoding="utf-8") as source:
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

    Car columns are ordered by ``car_slots``. Score and clock columns use NaN for
    values missing from the replay. The returned ``frames`` generator is not kept;
    use ``iter_frames`` separately for the full observations and events.
    """
    import numpy as np

    header = read_header(path)
    slots = header["car_slots"]
    slot_columns = {slot["slot"]: index for index, slot in enumerate(slots)}
    count = sum(1 for _ in iter_frames(path))
    car_count = len(slots)
    time = np.empty(count, dtype=np.float64)
    timeline_tick = np.empty(count, dtype=np.uint64)
    arena_tick = np.empty(count, dtype=np.uint64)
    ball_position = np.full((count, 3), np.nan, dtype=np.float32)
    ball_velocity = np.full((count, 3), np.nan, dtype=np.float32)
    car_position = np.full((count, car_count, 3), np.nan, dtype=np.float32)
    car_velocity = np.full((count, car_count, 3), np.nan, dtype=np.float32)
    car_boost = np.full((count, car_count), np.nan, dtype=np.float32)
    car_present = np.zeros((count, car_count), dtype=np.bool_)
    scores = np.full((count, 2), np.nan, dtype=np.float32)
    seconds_remaining = np.full(count, np.nan, dtype=np.float32)
    for row, frame in enumerate(iter_frames(path)):
        state = frame["state"]
        time[row] = frame["replay_time"]
        timeline_tick[row] = frame["timeline_tick"]
        arena_tick[row] = state["arena_tick"]
        ball_position[row] = state["ball"]["physics"]["position"]
        ball_velocity[row] = state["ball"]["physics"]["linear_velocity"]
        for car in state["cars"]:
            column = slot_columns[car["slot"]]
            car_present[row, column] = True
            car_position[row, column] = car["physics"]["position"]
            car_velocity[row, column] = car["physics"]["linear_velocity"]
            car_boost[row, column] = car["boost"]
        observed = frame["observations"]
        for team, score in enumerate(observed["team_scores"]):
            if score is not None:
                scores[row, team] = score["value"]
        clock = observed["seconds_remaining"]
        if clock is not None:
            seconds_remaining[row] = clock["value"]
    return {
        "header": header,
        "time": time,
        "timeline_tick": timeline_tick,
        "arena_tick": arena_tick,
        "ball_position": ball_position,
        "ball_velocity": ball_velocity,
        "car_position": car_position,
        "car_velocity": car_velocity,
        "car_boost": car_boost,
        "car_present": car_present,
        "scores": scores,
        "seconds_remaining": seconds_remaining,
    }
