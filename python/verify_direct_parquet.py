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
    held = load_columnar_numpy(args.parquet)["dead_shell_held"]
    print(
        f"Verified {frames} frames, header, and all NumPy arrays (dead-shell column against the JSONL's per-frame "
        f"list: {int((held == 1).sum())} observed and {int((held == 2).sum())} inferred slot-frames)"
    )
    if not args.no_tables:
        counts = verify_tables(args.jsonl, args.parquet)
        print("Verified record tables: " + ", ".join(f"{name} {rows}" for name, rows in counts.items()))


if __name__ == "__main__":
    main()
