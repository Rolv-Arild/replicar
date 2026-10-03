"""Compare a Rust-written Parquet export, its record tables included, with the schema-v1 JSONL reference.

One command checks the main file (header, row groups, NumPy arrays, rich frames) and the seven record
tables written beside it (``<stem>.<table>.parquet``): every table row against the JSONL record it
comes from, the car-slot columns against the slot of the car's linked player in that frame, and the
tables' ``source_sha256`` and ``frames`` metadata against the main file.
"""

from __future__ import annotations

import argparse
import math
from itertools import zip_longest

import numpy as np
import pyarrow.parquet as pq

from replay_columnar import (
    RECORD_TABLES,
    iter_columnar_frames,
    load_columnar_numpy,
    read_columnar_header,
    record_table_path,
)
from replay_to_rocketsim import iter_frames, load_numpy, read_header


def verify(jsonl: str, parquet: str) -> int:
    file = pq.ParquetFile(parquet)
    if any(file.metadata.row_group(i).num_rows > 512 for i in range(file.metadata.num_row_groups)):
        raise AssertionError("Parquet row group exceeded the 512-frame batch bound")
    expected_header = read_header(jsonl)
    actual_header = read_columnar_header(parquet)
    if actual_header != expected_header:
        raise AssertionError("Parquet header differs from JSONL header")
    expected = load_numpy(jsonl)
    actual = load_columnar_numpy(parquet)
    if actual.keys() != expected.keys():
        raise AssertionError(f"array keys differ: {actual.keys() ^ expected.keys()}")
    for name, value in expected.items():
        if isinstance(value, np.ndarray):
            np.testing.assert_equal(actual[name], value, err_msg=name)
        elif actual[name] != value:
            raise AssertionError(f"{name} differs")
    count = 0
    for count, (left, right) in enumerate(
        zip_longest(iter_frames(jsonl), iter_columnar_frames(parquet)), start=1
    ):
        if left != right:
            raise AssertionError(f"rich frame {count - 1} differs")
    return count


IN_PLAY_CLOCK_STATES = ("kickoff", "running", "expired", "decided")


def expected_labels(jsonl: str) -> dict:
    """Re-derive the training labels from the JSONL frames' scoreboard, goal events and car observations
    (a second implementation of the rule in ``src/labels.rs``), as arrays in ``load_numpy``'s layout, plus
    the header-level values."""
    header = read_header(jsonl)
    slot_by_key = {slot["player_key"]: index for index, slot in enumerate(header["car_slots"])}
    slot_index = {slot["slot"]: index for index, slot in enumerate(header["car_slots"])}
    cars = len(slot_by_key)
    frames = list(iter_frames(jsonl))
    count = len(frames)
    # float32 times, as the Rust side reads them from the frames
    time = [float(np.float32(frame["replay_time"])) for frame in frames]
    goals = []  # (frame, scoring team) of each goal frame
    for index, frame in enumerate(frames):
        for event in frame["observations"]["events"]:
            if event["kind"] == "goal_scored_on":
                goals.append((index, 1 - event["team"]))
                break
    goal_frames = dict(goals)
    episode = [-1] * count
    end = [None] * count
    open_frames: list[int] = []
    next_episode = 0
    blocked = False

    def close(last: int) -> None:
        for member in open_frames:
            end[member] = last
        open_frames.clear()

    for index, frame in enumerate(frames):
        board = frame.get("scoreboard")
        play = board is not None and board["clock_state"] in IN_PLAY_CLOCK_STATES
        if index in goal_frames:
            if open_frames or (play and not blocked):
                if not open_frames:
                    next_episode += 1
                episode[index] = next_episode - 1
                open_frames.append(index)
                close(index)
            blocked = True
        elif play:
            if blocked:
                continue
            if not open_frames:
                next_episode += 1
            episode[index] = next_episode - 1
            open_frames.append(index)
        else:
            if open_frames:
                close(open_frames[-1])
            blocked = False
    if open_frames:
        close(open_frames[-1])
    episode_remaining = np.full(count, np.nan, dtype=np.float32)
    next_team = np.full(count, -1, dtype=np.int8)
    until_goal = np.full(count, np.nan, dtype=np.float32)
    upcoming = None
    for index in range(count - 1, -1, -1):
        if index in goal_frames:
            upcoming = index
        if end[index] is not None:
            episode_remaining[index] = time[end[index]] - time[index]
        if upcoming is not None:
            next_team[index] = goal_frames[upcoming]
            until_goal[index] = time[upcoming] - time[index]
    update_age = np.full((count, cars), np.nan, dtype=np.float32)
    for index, frame in enumerate(frames):
        present = {slot_index[car["slot"]] for car in frame["state"]["cars"]}
        for slot, car in _slot_cars(frame, index, slot_by_key).items():
            position = car["body"]["position"]
            if slot in present and position is not None:
                update_age[index, slot] = time[index] - time[position["frame"]]
    last = frames[-1]["observations"]["team_scores"] if frames else [None, None]
    scores = [None if score is None else score["value"] for score in last]
    winner = None
    if None not in scores and scores[0] != scores[1]:
        winner = 0 if scores[0] > scores[1] else 1
    return {
        "label_episode": np.array(episode, dtype=np.int32),
        "label_episode_seconds_remaining": episode_remaining,
        "label_next_scoring_team": next_team,
        "label_seconds_until_next_goal": until_goal,
        "label_update_age_seconds": update_age,
        "final_score": scores,
        "winning_team": winner,
        "episodes": next_episode,
        "observed_goals": [sum(1 for _, team in goals if team == side) for side in (0, 1)],
    }


def _slot_cars(frame: dict, index: int, slot_by_key: dict) -> dict:
    """The car each slot shows in a frame: of the observed cars with the slot's player key, the one with
    the highest (link active, creation frame, has a packet in this frame) priority."""
    best = {}
    for car in frame["observations"]["cars"]:
        key = car.get("player_key")
        if key not in slot_by_key:
            continue
        position = car["body"]["position"]
        priority = (car["player_link_active"], car["actor_created_frame"],
                    position is not None and position["frame"] == index)
        if key not in best or priority > best[key][0]:
            best[key] = (priority, car)
    return {slot_by_key[key]: car for key, (_, car) in best.items()}


def verify_freshness(jsonl: str, parquet: str) -> dict:
    """Check the packet-freshness columns against a re-derivation from the JSONL: the masks from the
    observed bodies' packet frames, the frame-time age from the frame times, and the tick ages from the
    frames' timeline ticks and `packet_lag_ticks` records (inferred server tick = timeline tick of the
    packet's frame minus its lag, carried forward; no `default` source). Also reports the tick ages
    at fresh frames (0..4 except rare larger chained lags) and compares the `car_fresh` count per slot with the slot's `packet_lags`
    rows (differences are expected only where the converter records no lag: frames it does not simulate)."""
    header = read_header(jsonl)
    slot_by_key = {slot["player_key"]: index for index, slot in enumerate(header["car_slots"])}
    column_of_slot = {slot["slot"]: index for index, slot in enumerate(header["car_slots"])}
    cars = len(slot_by_key)
    frames = list(iter_frames(jsonl))
    count = len(frames)
    time = [float(np.float32(frame["replay_time"])) for frame in frames]
    ball_fresh = np.zeros(count, dtype=np.bool_)
    car_fresh = np.zeros((count, cars), dtype=np.bool_)
    car_exists = np.zeros((count, cars), dtype=np.bool_)
    ball_age = np.full(count, np.nan, dtype=np.float32)
    ball_ticks = np.full(count, -1, dtype=np.int32)
    car_ticks = np.full((count, cars), -1, dtype=np.int32)
    ball_server = None  # (known, tick): the ball's last packet; None before the first
    last_car = {}  # slot -> ((actor id, creation frame), server tick or None)
    lag_rows = np.zeros(cars, dtype=np.int64)
    for index, frame in enumerate(frames):
        tick = frame["timeline_tick"]
        lags = frame.get("packet_lag_ticks", [])
        ball = frame["observations"]["ball"]
        ball_position = None if ball is None else ball["position"]
        ball_fresh[index] = ball_position is not None and ball_position["frame"] == index
        if ball_position is not None:
            ball_age[index] = time[index] - time[ball_position["frame"]]
        if ball_fresh[index]:
            record = next((lag for lag in lags if lag["actor_id"] is None), None)
            ball_server = None if record is None or record["source"] == "default" else tick - record["ticks"]
            ball_server = (True, ball_server)
        if ball_server is not None and ball_server[1] is not None:
            ball_ticks[index] = tick - ball_server[1]
        present = {column_of_slot[car["slot"]] for car in frame["state"]["cars"]}
        slot_cars = _slot_cars(frame, index, slot_by_key)
        for slot in present:
            car_exists[index, slot] = True
        for slot, car in slot_cars.items():
            if slot not in present:
                continue
            lifetime = (car["actor_id"], car["actor_created_frame"])
            if slot in last_car and last_car[slot][0] != lifetime:
                del last_car[slot]
            position = car["body"]["position"]
            fresh = position is not None and position["frame"] == index
            car_fresh[index, slot] = fresh
            if fresh:
                record = next((lag for lag in lags if lag["actor_id"] == car["actor_id"]), None)
                server = None if record is None or record["source"] == "default" else tick - record["ticks"]
                last_car[slot] = (lifetime, server)
                lag_rows[slot] += record is not None
            if slot in last_car and last_car[slot][1] is not None:
                car_ticks[index, slot] = tick - last_car[slot][1]
    actual = load_columnar_numpy(parquet)
    np.testing.assert_array_equal(actual["ball_fresh"], ball_fresh, err_msg="ball_fresh")
    np.testing.assert_array_equal(actual["car_fresh"], car_fresh, err_msg="car_fresh")
    np.testing.assert_array_equal(actual["ball_update_age_seconds"], ball_age, err_msg="ball_update_age_seconds")
    np.testing.assert_array_equal(actual["ball_packet_age_ticks"], ball_ticks, err_msg="ball_packet_age_ticks")
    np.testing.assert_array_equal(actual["car_packet_age_ticks"], car_ticks, err_msg="car_packet_age_ticks")
    # A null `car_fresh` is exactly a slot with no car in the state.
    present_array = actual["car_present"]
    if not np.array_equal(present_array, car_exists):
        raise AssertionError("car_present differs from the slots with a car in the state")
    if np.any(actual["car_fresh"] & ~present_array):
        raise AssertionError("a slot with no car is marked fresh")
    ages_at_fresh = actual["car_packet_age_ticks"][actual["car_fresh"] & (actual["car_packet_age_ticks"] >= 0)]
    ball_at_fresh = actual["ball_packet_age_ticks"][actual["ball_fresh"] & (actual["ball_packet_age_ticks"] >= 0)]
    # The chained lags are 0..4 almost always; a rare larger one (a lag limited only by the frame gap) is reported.
    for name, values in (("car", ages_at_fresh), ("ball", ball_at_fresh)):
        if values.size and values.min() < 0:
            raise AssertionError(f"negative {name} tick age at a fresh frame")
    return {
        "car_fresh_per_slot": actual["car_fresh"].sum(axis=0).tolist(),
        "packet_lag_rows_per_slot": lag_rows.tolist(),
        "ball_fresh": int(actual["ball_fresh"].sum()),
        "car_age_at_fresh_max": int(ages_at_fresh.max()) if ages_at_fresh.size else None,
        "car_ages_over_4": int((ages_at_fresh > 4).sum()),
        "car_ages_at_fresh": int(ages_at_fresh.size),
        "ball_age_at_fresh_max": int(ball_at_fresh.max()) if ball_at_fresh.size else None,
        "car_age_known_fraction": float((actual["car_packet_age_ticks"] >= 0)[present_array].mean()),
    }


def verify_labels(jsonl: str, parquet: str) -> dict:
    """Check the Parquet label columns and the header labels against a re-derivation from the JSONL, and
    the label invariants (null outside play, episode ids contiguous, 0 s at the goal frame)."""
    want = expected_labels(jsonl)
    actual = load_columnar_numpy(parquet)
    for name in (key for key in want if key.startswith("label_")):
        np.testing.assert_array_equal(actual[name], want[name], err_msg=name)
    header = read_columnar_header(parquet)["labels"]
    if not header["future_derived"]:
        raise AssertionError("header labels are not marked future-derived")
    if header["final_score"] != want["final_score"]:
        raise AssertionError(f"final_score {header['final_score']} != {want['final_score']}")
    for key in ("winning_team", "episodes", "observed_goals"):
        if header[key] != want[key]:
            raise AssertionError(f"header labels {key}: {header[key]!r}, expected {want[key]!r}")
    episode = actual["label_episode"]
    in_episode = episode >= 0
    if not np.array_equal(in_episode, ~np.isnan(actual["label_episode_seconds_remaining"])):
        raise AssertionError("episode and episode_seconds_remaining are not null together")
    ids = episode[in_episode]
    if len(ids) and (ids[0] != 0 or np.any(np.diff(ids) < 0) or np.any(np.diff(ids) > 1) or ids[-1] + 1 != header["episodes"]):
        raise AssertionError("episode ids are not contiguous from 0")
    if np.any(actual["label_episode_seconds_remaining"][in_episode] < 0):
        raise AssertionError("negative episode_seconds_remaining")
    return {"episodes": header["episodes"], "goals": header["observed_goals"], "frames_in_episode": int(in_episode.sum())}


def verify_ping(jsonl: str, parquet: str) -> dict:
    """Check the ``ping_raw`` column against the raw ping bytes of the JSONL's players, resolved through the
    car slots of each frame (a second implementation of `slot_pings`)."""
    header = read_header(jsonl)
    slot_by_key = {slot["player_key"]: index for index, slot in enumerate(header["car_slots"])}
    column_of_slot = {slot["slot"]: index for index, slot in enumerate(header["car_slots"])}
    expected = []
    created = set()
    for frame in iter_frames(jsonl):
        created.update(column_of_slot[car["slot"]] for car in frame["state"]["cars"])
        row = [-1] * len(slot_by_key)
        for player in frame["observations"]["players"]:
            column = slot_by_key.get(player["key"])
            if column in created and player["ping_raw"] is not None:
                row[column] = player["ping_raw"]["value"]
        expected.append(row)
    expected = np.array(expected, dtype=np.int16).reshape(len(expected), len(slot_by_key))
    actual = load_columnar_numpy(parquet)["ping_raw"]
    np.testing.assert_array_equal(actual, expected, err_msg="ping_raw")
    seen = expected[expected >= 0]
    return {
        "slots_with_ping": int((expected >= 0).any(axis=0).sum()),
        "slots": expected.shape[1],
        "median": float(np.median(seen)) if seen.size else None,
    }


def _same(left, right) -> bool:
    """Equality of a table cell and its JSONL value: None only equals None, floats compare as float32
    (the tables store f32; the JSON holds the shortest decimal of the same f32), lists elementwise."""
    if left is None or right is None:
        return left is None and right is None
    if isinstance(left, (list, tuple)) or isinstance(right, (list, tuple)):
        return len(left) == len(right) and all(_same(a, b) for a, b in zip(left, right))
    if isinstance(left, float) or isinstance(right, float):
        a, b = np.float32(left), np.float32(right)
        return bool(a == b) or (math.isnan(a) and math.isnan(b))
    return left == right


EVENT_COLUMNS = (
    "team", "source", "attacker_car", "victim_car", "attacker_pri", "self_demolish",
    "attacker_velocity_x", "attacker_velocity_y", "attacker_velocity_z",
    "victim_velocity_x", "victim_velocity_y", "victim_velocity_z", "repeat", "car",
    "refreshed_count", "victim_slot", "attacker_slot", "car_slot",
)


def _expected_rows(table: str, frame: dict, slot_of) -> list[dict]:
    """The rows `table` must hold for one JSONL frame record, as {column: value} for every column but
    ``frame``. The slot columns come from the frame's cars (`slot_of`), not from the JSONL records."""
    observations = frame["observations"]
    rows = []
    if table == "touches":
        for r in frame.get("touches", []):
            rows.append({"car_slot": r["car_slot"], "tick": r["tick"], "contact_point": r["contact_point"]})
    elif table == "ball_contacts":
        for r in frame.get("ball_contacts", []):
            rows.append({key: r[key] for key in (
                "frame_a", "tick", "tick_from", "tick_to", "car_slot", "gap_uu", "velocity_residual",
                "simulated_touch")})
    elif table == "boost_pickups":
        for r in frame.get("boost_pickups", []):
            rows.append({key: r[key] for key in (
                "pad_index", "pad_actor_id", "is_big", "car_slot", "verified", "distance_uu",
                "suggested_car_slot", "tick")})
    elif table == "fitted_inputs":
        for r in frame.get("fitted_inputs", []):
            # A jump has no dodge fields and an air interval no pitch, yaw, cancel or activation
            # frame: they are 0 in the JSON and null in the table.
            dodge = r["kind"] == "dodge"
            rows.append({
                "slot": r["slot"], "kind": r["kind"], "tick": r["tick"],
                "activation_frame": r["activation_frame"] if dodge else None,
                "pitch": r["pitch"] if dodge else None,
                "yaw": r["yaw"] if dodge else None,
                "cancel": r["cancel"] if dodge else None,
                "span_ticks": r.get("span_ticks"),
            })
    elif table == "packet_lags":
        for r in frame.get("packet_lag_ticks", []):
            rows.append({
                "actor_id": r["actor_id"], "ticks": r["ticks"], "source": r["source"],
                "car_slot": slot_of(r["actor_id"]),
            })
    elif table == "events":
        for r in observations["events"]:
            row = dict.fromkeys(EVENT_COLUMNS)
            row["kind"] = r["kind"]
            if r["kind"] == "goal_scored_on":
                row["team"] = r["team"]
            elif r["kind"] == "demolish":
                for key in ("source", "attacker_car", "victim_car", "attacker_pri", "self_demolish", "repeat"):
                    row[key] = r[key]
                for axis, value in zip("xyz", r["attacker_velocity"]):
                    row[f"attacker_velocity_{axis}"] = value
                for axis, value in zip("xyz", r["victim_velocity"]):
                    row[f"victim_velocity_{axis}"] = value
                row["victim_slot"] = slot_of(r["victim_car"])
                row["attacker_slot"] = slot_of(r["attacker_car"])
            elif r["kind"] == "dodge_refreshed":
                row["car"] = r["car"]
                row["refreshed_count"] = r["count"]  # the JSON key is `count`
                row["car_slot"] = slot_of(r["car"])
            else:
                raise AssertionError(f"unknown event kind {r['kind']!r}")
            rows.append(row)
    elif table == "pad_pickups":
        for r in observations["pad_pickups"]:
            rows.append({
                "pad_actor_id": r["pad_actor_id"], "pad_actor_name": r["pad_actor_name"],
                "instigator_car_id": r["instigator_car_id"], "picked_up": r["picked_up"],
                "repeat": r["repeat"], "instigator_slot": slot_of(r["instigator_car_id"]),
            })
    else:
        raise AssertionError(f"no expectation for table {table}")
    return rows


def verify_tables(jsonl: str, parquet: str) -> dict[str, int]:
    """Check every record table beside `parquet` against `jsonl`; returns the row count per table."""
    header = read_header(jsonl)
    slot_by_key = {slot["player_key"]: slot["slot"] for slot in header["car_slots"]}
    frames = 0
    expected: dict[str, list[tuple[int, dict]]] = {name: [] for name in RECORD_TABLES}
    for frame in iter_frames(jsonl):
        frames += 1
        actor_slot = {
            car["actor_id"]: slot_by_key[car["player_key"]]
            for car in frame["observations"]["cars"]
            if car.get("player_key") in slot_by_key
        }

        def slot_of(actor, actor_slot=actor_slot):
            return None if actor is None else actor_slot.get(actor)

        for name in RECORD_TABLES:
            expected[name].extend((frame["frame"], row) for row in _expected_rows(name, frame, slot_of))
    main_footer = pq.ParquetFile(parquet).metadata.metadata or {}
    options_sha256 = main_footer.get(b"options_sha256", b"").decode()
    main_frames = pq.ParquetFile(parquet).metadata.num_rows
    if main_frames != frames:
        raise AssertionError(f"main file has {main_frames} frames, the JSONL {frames}")
    counts = {}
    for name in RECORD_TABLES:
        path = record_table_path(parquet, name)
        if not path.exists():
            raise AssertionError(f"record table {path} is missing (written with --no-event-tables?)")
        table_file = pq.ParquetFile(str(path))
        metadata = table_file.metadata.metadata or {}
        # The schema metadata as PyArrow shows it (`pq.read_table(..).schema.metadata`), not just the footer.
        schema_metadata = pq.read_schema(str(path)).metadata or {}
        if schema_metadata.get(b"source_sha256", b"").decode() != header["source_sha256"]:
            raise AssertionError(f"{name}: source_sha256 is missing from the schema metadata or differs from the header's")
        if not options_sha256 or schema_metadata.get(b"options_sha256", b"").decode() != options_sha256:
            raise AssertionError(f"{name}: options_sha256 is missing from the schema metadata or differs from the main file's")
        if metadata.get(b"frames", b"").decode() != str(frames):
            raise AssertionError(f"{name}: frames metadata differs from the main file's {frames}")
        rows = table_file.read().to_pylist()
        if len(rows) != len(expected[name]):
            raise AssertionError(f"{name}: {len(rows)} rows, the JSONL has {len(expected[name])}")
        for index, (row, (frame, want)) in enumerate(zip(rows, expected[name])):
            if row["frame"] != frame:
                raise AssertionError(f"{name} row {index}: frame {row['frame']}, expected {frame}")
            unknown = set(row) - set(want) - {"frame"}
            if unknown:
                raise AssertionError(f"{name}: columns without an expectation: {sorted(unknown)}")
            for column, value in want.items():
                if column not in row:
                    raise AssertionError(f"{name}: column {column} is missing")
                if not _same(row[column], value):
                    raise AssertionError(
                        f"{name} row {index} (frame {frame}) column {column}: {row[column]!r}, expected {value!r}"
                    )
        counts[name] = len(rows)
    return counts


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("jsonl")
    parser.add_argument("parquet")
    parser.add_argument(
        "--no-tables", action="store_true", help="skip the record tables (a --no-event-tables export)"
    )
    args = parser.parse_args()
    frames = verify(args.jsonl, args.parquet)
    arrays = load_columnar_numpy(args.parquet)
    held = arrays["dead_shell_held"]
    print(
        f"Verified {frames} frames, header, and all NumPy arrays (dead-shell and spawn-pose columns against the "
        f"JSONL's per-frame lists: {int((held == 1).sum())} observed and {int((held == 2).sum())} inferred "
        f"dead-shell slot-frames, {int(arrays['spawn_pose_held'].sum())} spawn-pose slot-frames)"
    )
    labels = verify_labels(args.jsonl, args.parquet)
    print(
        f"Verified the label columns and header labels against a re-derivation from the JSONL: "
        f"{labels['episodes']} episodes, {labels['frames_in_episode']} frames in an episode, "
        f"goals (blue, orange) {labels['goals']}"
    )
    ping = verify_ping(args.jsonl, args.parquet)
    print(
        f"Verified ping_raw: {ping['slots_with_ping']} of {ping['slots']} slots ever have a ping, "
        f"median raw byte {ping['median']}"
    )
    fresh = verify_freshness(args.jsonl, args.parquet)
    print(
        f"Verified the freshness columns against a re-derivation from the JSONL: ball fresh in {fresh['ball_fresh']} "
        f"frames; car_fresh per slot {fresh['car_fresh_per_slot']} against packet_lags rows per slot "
        f"{fresh['packet_lag_rows_per_slot']}; tick age at a fresh frame at most {fresh['car_age_at_fresh_max']} (cars; {fresh['car_ages_over_4']} of "
        f"{fresh['car_ages_at_fresh']} above 4) and {fresh['ball_age_at_fresh_max']} (ball); the car tick age is known in "
        f"{fresh['car_age_known_fraction']:.3f} of the slot-frames with a car"
    )
    if not args.no_tables:
        counts = verify_tables(args.jsonl, args.parquet)
        print("Verified record tables: " + ", ".join(f"{name} {rows}" for name, rows in counts.items()))


if __name__ == "__main__":
    main()
