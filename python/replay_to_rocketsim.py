"""Read schema-v1 replay-to-rocketsim JSON Lines exports.

``iter_frames`` streams rich records. ``load_numpy`` builds dense arrays for ML;
NumPy is imported only when that function is called.
"""

from __future__ import annotations

import gzip
import json
import struct
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


def f32(value: float) -> float:
    """``value`` rounded to float32 (the frame times are float32 on the Rust side)."""
    return struct.unpack("f", struct.pack("f", value))[0]


def slot_pings(
    frame: dict[str, Any], slots: list[dict[str, Any]], created: set[int], times: list[float]
) -> tuple[list[int | None], list[float | None]]:
    """The raw ping byte of each car slot's player in one frame and its age in seconds (frame time minus the
    time of the ping's source frame), ``None`` where unknown.

    ``times`` holds the float32-rounded ``replay_time`` of the frames so far (``f32``), this frame's last.

    ``slots`` is the header's ``car_slots`` (the column of a slot is its index). A slot counts as created from
    the first frame its car is in the state (``created`` holds the columns seen so far; keep it across
    frames); before that, and until the player's first ``ping_raw`` update, the ping is unknown (a ping of 0
    is a value).
    """
    slot_columns = {slot["slot"]: index for index, slot in enumerate(slots)}
    slot_by_key = {slot["player_key"]: index for index, slot in enumerate(slots)}
    for car in frame["state"]["cars"]:
        created.add(slot_columns[car["slot"]])
    pings: list[int | None] = [None] * len(slots)
    ages: list[float | None] = [None] * len(slots)
    for player in frame["observations"].get("players", ()):
        column = slot_by_key.get(player["key"])
        ping = player.get("ping_raw")
        if column is not None and ping is not None and column in created:
            pings[column] = ping["value"]
            ages[column] = times[-1] - times[ping["frame"]]
    return pings, ages


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

    ``spawn_pose_held`` (frames x car slots, bool) marks the frames in which a slot's car is known only from
    its spawn pose (no rigid-body packet yet in its lifetime): the pose is inferred and the car takes no part
    in collisions.

    The ``label_*`` entries are the training labels (the frame's ``labels`` object; see ``src/labels.rs``).
    They are kept apart from the state and observations, and ALL of them are FUTURE-DERIVED (read from
    later frames of the same replay; each ``labels`` object says ``future_derived: true``):
    ``label_episode`` (int32, the goal-to-goal segment, -1 outside play),
    ``label_episode_seconds_remaining`` (float32, replay time to the end of the episode, NaN outside one),
    ``label_next_scoring_team`` (int8, 0 blue / 1 orange from the observed goal events, -1 when no goal
    follows) and ``label_seconds_until_next_goal`` (float32, NaN when none). In goal-pause, countdown and
    tail frames the next-goal labels refer to the next goal anywhere, which may be in the next episode or in
    overtime. A frame without a ``labels`` object (an older export) reads as unknown throughout. The replay's final score and winning team are in
    ``header["labels"]`` (final, future-derived; not per frame).

    ``ping_raw`` (frames x car slots, int16) is an observation, not a label: the raw
    ``Engine.PlayerReplicationInfo:Ping`` byte of the slot's player (probably milliseconds / 4; the unit is
    not calibrated), -1 until the player's first update (a ping of 0 is a value) and for a slot whose car does
    not exist yet. Host and bot players of a host or LAN replay never have one. ``ping_age_seconds`` (float32,
    NaN with the ping) is the frame time minus the time of the frame of the ping's last update, so a stale
    value is visible.

    The packet-freshness entries (the frame's ``freshness`` object, ``src/freshness.rs``) are observed or
    inferred, not labels: ``ball_fresh`` (frames, int8) and ``car_fresh`` (frames x car slots, int8) say whether
    the converter applied a fresh rigid-body packet at that frame (1 yes, 0 no, -1 unknown: a slot with no car
    or whose primary car cannot be resolved in the frame, and a frame of an export without freshness). They
    count fresh packets in frames the converter does not simulate too, so they are a superset of the
    ``packet_lags`` rows. ``ball_update_age_seconds`` and ``car_update_age_seconds`` (float32; frames and
    frames x car slots) are frame time minus the time of the frame with the body's last packet, NaN before the
    first packet, for an unresolved slot and for a respawned car until its first packet.
    ``ball_packet_age_ticks`` (int32) and ``car_packet_age_ticks`` (frames x car
    slots, int32) are the frame's timeline tick minus the inferred server tick of the last applied packet (the
    tick of the packet's frame minus its inferred lag, carried forward: 0 to 4 at a fresh frame, growing between
    packets). They use the OFFLINE lag inference (``packet_lags``) and are -1 when unknown: before the first
    packet, when no lag was inferred (inference off, a frame the converter does not simulate, the ``default``
    lag source), for a respawned car until its first packet, and for a slot with no car. At a fresh frame they
    are normally 0 to 4, larger when the frame gap exceeds 4 ticks (train maximum 9 for cars, 5 for the ball).
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
    spawn_pose_held = np.zeros((count, car_count), dtype=np.bool_)
    label_episode = np.full(count, -1, dtype=np.int32)
    label_episode_seconds_remaining = np.full(count, np.nan, dtype=np.float32)
    label_next_scoring_team = np.full(count, -1, dtype=np.int8)
    label_seconds_until_next_goal = np.full(count, np.nan, dtype=np.float32)
    ping_raw = np.full((count, car_count), -1, dtype=np.int16)
    ping_age_seconds = np.full((count, car_count), np.nan, dtype=np.float32)
    ball_fresh = np.full(count, -1, dtype=np.int8)
    car_fresh = np.full((count, car_count), -1, dtype=np.int8)
    ball_update_age_seconds = np.full(count, np.nan, dtype=np.float32)
    car_update_age_seconds = np.full((count, car_count), np.nan, dtype=np.float32)
    ball_packet_age_ticks = np.full(count, -1, dtype=np.int32)
    car_packet_age_ticks = np.full((count, car_count), -1, dtype=np.int32)
    created_slots: set[int] = set()
    times: list[float] = []
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
        for slot in frame.get("spawn_pose_held", ()):
            spawn_pose_held[row, slot_columns[slot]] = True
        times.append(f32(frame["replay_time"]))
        pings, ping_ages = slot_pings(frame, slots, created_slots, times)
        for column, ping in enumerate(pings):
            if ping is not None:
                ping_raw[row, column] = ping
                ping_age_seconds[row, column] = ping_ages[column]
        freshness = frame.get("freshness")
        if freshness is not None:
            ball_fresh[row] = int(freshness["ball_fresh"])
            if freshness["ball_update_age_seconds"] is not None:
                ball_update_age_seconds[row] = freshness["ball_update_age_seconds"]
            if freshness["ball_packet_age_ticks"] is not None:
                ball_packet_age_ticks[row] = freshness["ball_packet_age_ticks"]
            for column, fresh in enumerate(freshness["car_fresh"]):
                if fresh is not None:
                    car_fresh[row, column] = int(fresh)
            for column, age in enumerate(freshness["car_update_age_seconds"]):
                if age is not None:
                    car_update_age_seconds[row, column] = age
            for column, age in enumerate(freshness["car_packet_age_ticks"]):
                if age is not None:
                    car_packet_age_ticks[row, column] = age
        labels = frame.get("labels")
        if labels is not None:
            if labels["episode"] is not None:
                label_episode[row] = labels["episode"]
            if labels["episode_seconds_remaining"] is not None:
                label_episode_seconds_remaining[row] = labels["episode_seconds_remaining"]
            if labels["next_scoring_team"] is not None:
                label_next_scoring_team[row] = labels["next_scoring_team"]
            if labels["seconds_until_next_goal"] is not None:
                label_seconds_until_next_goal[row] = labels["seconds_until_next_goal"]
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
        "spawn_pose_held": spawn_pose_held,
        "label_episode": label_episode,
        "label_episode_seconds_remaining": label_episode_seconds_remaining,
        "label_next_scoring_team": label_next_scoring_team,
        "label_seconds_until_next_goal": label_seconds_until_next_goal,
        "ping_raw": ping_raw,
        "ping_age_seconds": ping_age_seconds,
        "ball_fresh": ball_fresh,
        "car_fresh": car_fresh,
        "ball_update_age_seconds": ball_update_age_seconds,
        "car_update_age_seconds": car_update_age_seconds,
        "ball_packet_age_ticks": ball_packet_age_ticks,
        "car_packet_age_ticks": car_packet_age_ticks,
    }
