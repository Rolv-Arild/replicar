"""Prototype Arrow IPC and Parquet exports for schema-v1 Rust JSONL output.

The typed columns support fast ML reads. ``frame_json`` preserves the complete
RocketSim snapshot, replay observations/provenance, events, and residuals until
those evolving structures have a settled column schema. PyArrow is optional.
"""

from __future__ import annotations

import argparse
import json
import math
import warnings
from pathlib import Path
from typing import Any, Iterator

from replay_to_rocketsim import DEAD_SHELL_CODES, iter_frames, read_header


COLUMNAR_VERSION = 1
AXES_ORDER = ("throttle", "steer", "pitch", "yaw", "roll")
BUTTONS_ORDER = ("jump", "boost", "handbrake")
VECTOR_COLUMNS = {
    "ball_position": 3,
    "ball_rotation_columns": 9,
    "ball_velocity": 3,
    "ball_angular_velocity": 3,
    "car_position": 3,
    "car_rotation_columns": 9,
    "car_velocity": 3,
    "car_angular_velocity": 3,
    "car_boost": 1,
    "car_present": 1,
    "car_demoed": 1,
    "control_axes": 5,
    "control_buttons": 3,
    "boost_pad_active": 1,
    "boost_pad_cooldown": 1,
    "scores": 2,
}
# Appended after ``frame_json`` by the Rust writer (and by ``write_columnar``): the reconstructed match
# clock. A file written before they existed lacks them; the loader then reports unknown.
SCOREBOARD_COLUMNS = (
    "scoreboard_period",
    "scoreboard_clock_state",
    "scoreboard_seconds_remaining",
    "scoreboard_overtime_seconds",
)
# Side tables written beside the main file by the Rust writer as ``<stem>.<table>.parquet``.
RECORD_TABLES = (
    "touches",
    "ball_contacts",
    "boost_pickups",
    "fitted_inputs",
    "packet_lags",
    "events",
    "pad_pickups",
)
HOT_COLUMNS = (
    "frame", "replay_time", "timeline_tick", "arena_tick",
    *VECTOR_COLUMNS, "seconds_remaining",
)


def _arrow_modules():
    try:
        import pyarrow as pa
        import pyarrow.ipc as ipc
        import pyarrow.parquet as pq
    except ImportError as error:
        raise ImportError("columnar exports require pyarrow; see python/requirements-columnar.txt") from error
    return pa, ipc, pq


def _kind(path: str | Path) -> str:
    suffix = Path(path).suffix.lower()
    if suffix == ".arrow":
        return "arrow"
    if suffix == ".parquet":
        return "parquet"
    raise ValueError("columnar path must end in .arrow or .parquet")


def _schema(pa: Any, header: dict[str, Any], pads: list[dict[str, Any]]):
    cars = len(header["car_slots"])
    pad_count = len(pads)
    f32 = pa.float32()
    boolean = pa.bool_()
    fields = [
        pa.field("frame", pa.uint32()),
        pa.field("replay_time", pa.float64()),
        pa.field("timeline_tick", pa.uint64()),
        pa.field("arena_tick", pa.uint64()),
    ]
    for name, width in VECTOR_COLUMNS.items():
        if name.startswith("boost_pad_"):
            entities = pad_count
        elif name.startswith(("car_", "control_")):
            entities = cars
        else:
            entities = 1
        count = width * entities
        dtype = boolean if name in ("car_present", "car_demoed", "control_buttons", "boost_pad_active") else f32
        fields.append(pa.field(name, pa.list_(dtype, count)))
    fields.extend((
        pa.field("seconds_remaining", f32),
        pa.field("frame_json", pa.large_binary()),
        # The Rust writer dictionary-encodes these two; the strings are the contract.
        pa.field("scoreboard_period", pa.string()),
        pa.field("scoreboard_clock_state", pa.string()),
        pa.field("scoreboard_seconds_remaining", f32),
        pa.field("scoreboard_overtime_seconds", f32),
        # Per car slot, null when the slot is not held as a dead pawn shell: 1 observed (goal explosion),
        # 2 inferred (sleeping packet of an unlinked car).
        pa.field("dead_shell_held", pa.list_(pa.uint8(), cars)),
        # Per car slot: the car is known only from its spawn pose (no rigid-body packet yet).
        pa.field("spawn_pose_held", pa.list_(boolean, cars)),
    ))
    metadata = {
        b"columnar_version": str(COLUMNAR_VERSION).encode(),
        b"replay_header_json": json.dumps(header, separators=(",", ":")).encode(),
        b"pad_config_json": json.dumps(pads, separators=(",", ":")).encode(),
    }
    return pa.schema(fields, metadata=metadata)


def _flatten(matrix: list[list[Any]]) -> list[Any]:
    return [item for row in matrix for item in row]


def _row(frame: dict[str, Any], slots: list[dict[str, Any]], pad_count: int) -> dict[str, Any]:
    state = frame["state"]
    ball = state["ball"]["physics"]
    cars = len(slots)
    slot_columns = {slot["slot"]: index for index, slot in enumerate(slots)}
    missing = math.nan
    car_position = [missing] * (cars * 3)
    car_rotation = [missing] * (cars * 9)
    car_velocity = [missing] * (cars * 3)
    car_angular = [missing] * (cars * 3)
    car_boost = [missing] * cars
    car_present = [False] * cars
    car_demoed = [False] * cars
    control_axes = [missing] * (cars * 5)
    control_buttons = [False] * (cars * 3)
    spawn_pose_held = [False] * cars
    for slot in frame.get("spawn_pose_held", ()):
        spawn_pose_held[slot_columns[slot]] = True
    dead_shell_held: list[int | None] = [None] * cars
    for held in frame.get("dead_shell_held", ()):
        dead_shell_held[slot_columns[held["slot"]]] = DEAD_SHELL_CODES[held["source"]]
    for car in state["cars"]:
        index = slot_columns[car["slot"]]
        physics = car["physics"]
        car_present[index] = True
        car_demoed[index] = car["is_demoed"]
        car_position[index * 3:(index + 1) * 3] = physics["position"]
        car_rotation[index * 9:(index + 1) * 9] = _flatten(physics["rotation_columns"])
        car_velocity[index * 3:(index + 1) * 3] = physics["linear_velocity"]
        car_angular[index * 3:(index + 1) * 3] = physics["angular_velocity"]
        car_boost[index] = car["boost"]
        control_axes[index * 5:(index + 1) * 5] = [car["controls"][key] for key in AXES_ORDER]
        control_buttons[index * 3:(index + 1) * 3] = [car["controls"][key] for key in BUTTONS_ORDER]
    pads = state["boost_pads"]
    if len(pads) != pad_count:
        raise ValueError(f"boost pad count changed at frame {frame['frame']}")
    scores = [
        missing if score is None else score["value"]
        for score in frame["observations"]["team_scores"]
    ]
    clock = frame["observations"]["seconds_remaining"]
    board = frame.get("scoreboard")
    return {
        "frame": frame["frame"],
        "replay_time": frame["replay_time"],
        "timeline_tick": frame["timeline_tick"],
        "arena_tick": state["arena_tick"],
        "ball_position": ball["position"],
        "ball_rotation_columns": _flatten(ball["rotation_columns"]),
        "ball_velocity": ball["linear_velocity"],
        "ball_angular_velocity": ball["angular_velocity"],
        "car_position": car_position,
        "car_rotation_columns": car_rotation,
        "car_velocity": car_velocity,
        "car_angular_velocity": car_angular,
        "car_boost": car_boost,
        "car_present": car_present,
        "car_demoed": car_demoed,
        "control_axes": control_axes,
        "control_buttons": control_buttons,
        "boost_pad_active": [pad["is_active"] for pad in pads],
        "boost_pad_cooldown": [pad["cooldown"] for pad in pads],
        "scores": scores,
        "seconds_remaining": missing if clock is None else clock["value"],
        "frame_json": json.dumps(frame, separators=(",", ":"), ensure_ascii=False).encode(),
        "scoreboard_period": None if board is None else board["period"],
        "scoreboard_clock_state": None if board is None else board["clock_state"],
        "scoreboard_seconds_remaining": None if board is None else board["seconds_remaining"],
        "scoreboard_overtime_seconds": None if board is None else board["overtime_seconds"],
        "dead_shell_held": dead_shell_held,
        "spawn_pose_held": spawn_pose_held,
    }


def write_columnar(jsonl_path: str | Path, output_path: str | Path, batch_size: int = 512) -> int:
    """Write a complete, batched columnar projection of a Rust JSONL conversion."""
    pa, ipc, pq = _arrow_modules()
    kind = _kind(output_path)
    if batch_size < 1:
        raise ValueError("batch_size must be positive")
    header = read_header(jsonl_path)
    frames = iter_frames(jsonl_path)
    first = next(frames, None)
    if first is not None and first["frame"] != 0:
        raise ValueError(f"unexpected first frame index {first['frame']}; expected 0")
    pads = first["state"]["boost_pads"] if first is not None else []
    schema = _schema(pa, header, pads)
    writer = (
        ipc.new_file(str(output_path), schema, options=ipc.IpcWriteOptions(compression="zstd"))
        if kind == "arrow"
        else pq.ParquetWriter(str(output_path), schema, compression="zstd", compression_level=3)
    )
    count = 0
    batch: list[dict[str, Any]] = []
    try:
        if first is not None:
            batch.append(_row(first, header["car_slots"], len(pads)))
            count = 1
        for frame in frames:
            if frame["frame"] != count:
                raise ValueError(f"unexpected frame index {frame['frame']}; expected {count}")
            batch.append(_row(frame, header["car_slots"], len(pads)))
            count += 1
            if len(batch) >= batch_size:
                writer.write_table(pa.Table.from_pylist(batch, schema=schema))
                batch.clear()
        if batch:
            writer.write_table(pa.Table.from_pylist(batch, schema=schema))
    finally:
        writer.close()
    return count


def _metadata(path: str | Path) -> dict[bytes, bytes]:
    _, ipc, pq = _arrow_modules()
    if _kind(path) == "arrow":
        return ipc.open_file(str(path)).schema.metadata or {}
    parquet = pq.ParquetFile(str(path))
    # The Rust writer converts in one pass and appends ``replay_header_json`` (which holds the
    # conversion's diagnostics) to the file's key-value metadata at close; PyArrow's ``schema_arrow``
    # only carries the schema metadata written when the file was opened.
    metadata = dict(parquet.schema_arrow.metadata or {})
    metadata.update(parquet.metadata.metadata or {})
    return metadata


def read_columnar_header(path: str | Path) -> dict[str, Any]:
    metadata = _metadata(path)
    if metadata.get(b"columnar_version") != b"1":
        raise ValueError("unsupported columnar version")
    header = json.loads(metadata[b"replay_header_json"])
    if header.get("schema_version") != 1:
        raise ValueError("unsupported replay schema version")
    return header


def iter_columnar_frames(path: str | Path) -> Iterator[dict[str, Any]]:
    """Recover the complete rich frame records, including observations and events."""
    pa, ipc, pq = _arrow_modules()
    read_columnar_header(path)
    if _kind(path) == "parquet":
        batches = pq.ParquetFile(str(path)).iter_batches(columns=["frame_json"])
        for batch in batches:
            for value in batch.column(0).to_pylist():
                yield json.loads(value)
    else:
        with pa.memory_map(str(path), "r") as source:
            reader = ipc.open_file(source)
            column = reader.schema.get_field_index("frame_json")
            for index in range(reader.num_record_batches):
                for value in reader.get_batch(index).column(column).to_pylist():
                    yield json.loads(value)


def load_columnar_numpy(path: str | Path) -> dict[str, Any]:
    """Read typed ML columns; rich frame JSON is skipped for Parquet projection."""
    import numpy as np

    _, ipc, pq = _arrow_modules()
    header = read_columnar_header(path)
    pad_config = json.loads(_metadata(path)[b"pad_config_json"])
    if _kind(path) == "parquet":
        table = pq.read_table(str(path), columns=list(HOT_COLUMNS))
    else:
        table = ipc.open_file(str(path)).read_all().select(list(HOT_COLUMNS))
    count = table.num_rows
    cars = len(header["car_slots"])
    pads = len(pad_config)

    def primitive(name: str, dtype: Any):
        return table[name].combine_chunks().to_numpy(zero_copy_only=False).astype(dtype, copy=True)

    def fixed(name: str, shape: tuple[int, ...], dtype: Any):
        values = table[name].combine_chunks().values.to_numpy(zero_copy_only=False)
        return values.astype(dtype, copy=True).reshape(shape)

    # The scoreboard columns are optional (older files); a null is unknown, not zero.
    if _kind(path) == "parquet":
        available = set(pq.ParquetFile(str(path)).schema_arrow.names)
    else:
        available = set(ipc.open_file(str(path)).schema.names)
    board = None
    if set(SCOREBOARD_COLUMNS) <= available:
        if _kind(path) == "parquet":
            board = pq.read_table(str(path), columns=list(SCOREBOARD_COLUMNS))
        else:
            board = ipc.open_file(str(path)).read_all().select(list(SCOREBOARD_COLUMNS))

    # Per car slot, null (not held) as 0; a file without the column (older export) has none held.
    if "dead_shell_held" in available:
        if _kind(path) == "parquet":
            held_table = pq.read_table(str(path), columns=["dead_shell_held"])
        else:
            held_table = ipc.open_file(str(path)).read_all().select(["dead_shell_held"])
        held_values = held_table["dead_shell_held"].combine_chunks().values.to_numpy(zero_copy_only=False)
        dead_shell_held = np.nan_to_num(held_values.astype(np.float64), nan=0.0).astype(np.uint8).reshape((count, cars))
    else:
        dead_shell_held = np.zeros((count, cars), dtype=np.uint8)
    if "spawn_pose_held" in available:
        if _kind(path) == "parquet":
            spawn_table = pq.read_table(str(path), columns=["spawn_pose_held"])
        else:
            spawn_table = ipc.open_file(str(path)).read_all().select(["spawn_pose_held"])
        spawn_pose_held = (
            spawn_table["spawn_pose_held"].combine_chunks().values.to_numpy(zero_copy_only=False)
            .astype(np.bool_).reshape((count, cars))
        )
    else:
        spawn_pose_held = np.zeros((count, cars), dtype=np.bool_)

    def label(name: str):
        if board is None:
            return np.full(count, None, dtype=object)
        values = np.empty(count, dtype=object)
        values[:] = board[name].to_pylist()
        return values

    def clock(name: str):
        if board is None:
            return np.full(count, np.nan, dtype=np.float32)
        return np.array(board[name].to_pylist(), dtype=np.float32)  # None becomes NaN

    return {
        "header": header,
        "time": primitive("replay_time", np.float64),
        "timeline_tick": primitive("timeline_tick", np.uint64),
        "arena_tick": primitive("arena_tick", np.uint64),
        "ball_position": fixed("ball_position", (count, 3), np.float32),
        "ball_rotation_columns": fixed("ball_rotation_columns", (count, 3, 3), np.float32),
        "ball_velocity": fixed("ball_velocity", (count, 3), np.float32),
        "ball_angular_velocity": fixed("ball_angular_velocity", (count, 3), np.float32),
        "car_position": fixed("car_position", (count, cars, 3), np.float32),
        "car_rotation_columns": fixed("car_rotation_columns", (count, cars, 3, 3), np.float32),
        "car_velocity": fixed("car_velocity", (count, cars, 3), np.float32),
        "car_angular_velocity": fixed("car_angular_velocity", (count, cars, 3), np.float32),
        "car_boost": fixed("car_boost", (count, cars), np.float32),
        "car_present": fixed("car_present", (count, cars), np.bool_),
        "car_demoed": fixed("car_demoed", (count, cars), np.bool_),
        "control_axes": fixed("control_axes", (count, cars, 5), np.float32),
        "control_axes_order": AXES_ORDER,
        "control_buttons": fixed("control_buttons", (count, cars, 3), np.bool_),
        "control_buttons_order": BUTTONS_ORDER,
        "boost_pad_position": np.array([pad["position"] for pad in pad_config], dtype=np.float32).reshape(pads, 3),
        "boost_pad_is_big": np.array([pad["is_big"] for pad in pad_config], dtype=np.bool_),
        "boost_pad_active": fixed("boost_pad_active", (count, pads), np.bool_),
        "boost_pad_cooldown": fixed("boost_pad_cooldown", (count, pads), np.float32),
        "scores": fixed("scores", (count, 2), np.float32),
        "seconds_remaining": primitive("seconds_remaining", np.float32),
        "scoreboard_period": label("scoreboard_period"),
        "scoreboard_clock_state": label("scoreboard_clock_state"),
        "scoreboard_seconds_remaining": clock("scoreboard_seconds_remaining"),
        "scoreboard_overtime_seconds": clock("scoreboard_overtime_seconds"),
        "dead_shell_held": dead_shell_held,
        "spawn_pose_held": spawn_pose_held,
    }


def record_table_path(path: str | Path, table: str) -> Path:
    """``<dir>/<stem>.<table>.parquet`` beside the main file ``<dir>/<stem>.parquet``."""
    if table not in RECORD_TABLES:
        raise ValueError(f"unknown record table {table!r}; expected one of {RECORD_TABLES}")
    path = Path(path)
    return path.with_name(f"{path.stem}.{table}.parquet")


def read_record_tables(path: str | Path, verify: bool = True) -> dict[str, Any]:
    """Read the side tables written beside a Rust Parquet export, as ``pyarrow.Table`` values.

    Every table has a ``frame`` column (the frame the record belongs to). A table that was not
    written (``convert_replay --no-event-tables``, or a file made by ``write_columnar``) is missing
    from the result; an empty table keeps its schema. The Rust writer is the only one that makes them.

    Each table's schema metadata holds the ``source_sha256`` of its replay and the ``options_sha256`` of
    the conversion options, and its footer the main file's ``frames`` count; the main file has the same
    hashes (the replay hash in its header, the options hash in its footer). With ``verify`` (the default) a table whose hashes or frame count differ from the main file's,
    that has none (written before the metadata existed), or that cannot be read (truncated), is skipped
    with a warning: it belongs to another export that left it beside this file, or to an export that did
    not finish. ``verify=False`` reads whatever is there (an unreadable table still raises).
    """
    _, _, pq = _arrow_modules()
    expected = {}
    if verify:
        expected["source_sha256"] = read_columnar_header(path).get("source_sha256")
        main_metadata = _metadata(path)
        options = main_metadata.get(b"options_sha256")
        # A main file without the options hash (an older export, or `write_columnar`) cannot vouch for it.
        expected["options_sha256"] = options.decode() if options else None
        expected["frames"] = str(pq.ParquetFile(str(path)).metadata.num_rows)
    tables = {}
    for name in RECORD_TABLES:
        table_path = record_table_path(path, name)
        if not table_path.exists():
            continue
        if verify:
            try:
                metadata = _metadata(table_path)
            except Exception as error:  # a truncated or corrupt file
                warnings.warn(f"skipping unreadable record table {table_path}: {error}", stacklevel=2)
                continue
            problem = None
            found = {key: metadata.get(key.encode(), b"").decode() or None for key in expected}
            if found["source_sha256"] is None or expected["source_sha256"] is None:
                problem = "its source hash cannot be checked (no source_sha256 in the table or the main file)"
            elif found["source_sha256"] != expected["source_sha256"]:
                problem = (
                    f"it is from another replay or export (source_sha256 {found['source_sha256'][:12]}, "
                    f"main file {expected['source_sha256'][:12]})"
                )
            elif expected["options_sha256"] is not None and found["options_sha256"] != expected["options_sha256"]:
                problem = "it was written by a conversion with other options (options_sha256 differs or is missing)"
            elif found["frames"] != expected["frames"]:
                problem = f"it has {found['frames']} frames, the main file {expected['frames']}"
            if problem is not None:
                warnings.warn(f"skipping stale record table {table_path}: {problem}", stacklevel=2)
                continue
        try:
            tables[name] = pq.read_table(str(table_path))
        except Exception as error:
            if not verify:
                raise
            warnings.warn(f"skipping unreadable record table {table_path}: {error}", stacklevel=2)
    return tables


def main() -> None:
    parser = argparse.ArgumentParser(description="Prototype columnar export of Rust JSONL states")
    parser.add_argument("jsonl", type=Path)
    parser.add_argument("output", type=Path, help=".arrow or .parquet")
    parser.add_argument("--batch-size", type=int, default=512)
    args = parser.parse_args()
    frames = write_columnar(args.jsonl, args.output, args.batch_size)
    print(f"{frames} frames -> {args.output}")


if __name__ == "__main__":
    main()
