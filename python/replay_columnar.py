"""Prototype Arrow IPC and Parquet exports for schema-v1 Rust JSONL output.

The typed columns support fast ML reads. ``frame_json`` preserves the complete
RocketSim snapshot, replay observations/provenance, events, and residuals until
those evolving structures have a settled column schema. PyArrow is optional.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Iterator

from replay_to_rocketsim import iter_frames, read_header


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
    return pq.ParquetFile(str(path)).schema_arrow.metadata or {}


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
    }


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
