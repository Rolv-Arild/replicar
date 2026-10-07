"""Read replicar files: Rocket League replays reconstructed as RocketSim states, one Parquet file per replay.

A replicar file is an ordinary Parquet file; this package only adds conveniences (docs/glossary.md):

>>> import replicar
>>> f = replicar.read("match.parquet")
>>> f.header["players"]                   # who is at each player index
>>> a = f.arrays()                        # NumPy: a["car_position"] has shape (rows, players, 3)
>>> f.records("events")                   # a pyarrow table with one row per event and its frame

Floats are float32 with NaN for unknown values; integers use -1 for unknown; quantized columns are decoded with
their scale. Reading needs only pyarrow and NumPy.
"""

from __future__ import annotations

import json
import re
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pyarrow as pa
import pyarrow.compute as pc
import pyarrow.parquet as pq

__all__ = ["FORMAT_VERSION", "ReplicarFile", "convert", "convert_many", "iter_frames", "read", "resimulate"]

#: The newest file format this reader reads.
FORMAT_VERSION = 1

_PLAYER = re.compile(r"^(car|player|network_car|network_player)_(\d+)_(.+)$")
_PAD = re.compile(r"^pad_(\d+)_(.+)$")
_COMPONENTS = ("x", "y", "z", "w")


def _values(column: pa.ChunkedArray, field: pa.Field) -> np.ndarray:
    """A column as NumPy: float32 with NaN, integers with -1, booleans (int8 with -1 when any is null),
    names as strings ('' for null); integers with a `scale` decoded to float32."""
    array = column.combine_chunks()
    metadata = field.metadata or {}
    kind = field.type
    if b"scale" in metadata:
        scale = float(metadata[b"scale"])
        values = array.to_numpy(zero_copy_only=False).astype(np.float64)
        if array.null_count:
            values[np.asarray(array.is_null())] = np.nan
        return (values * scale).astype(np.float32)
    if pa.types.is_dictionary(kind):
        array = array.cast(pa.string())
        kind = pa.string()
    if pa.types.is_floating(kind):
        return array.to_numpy(zero_copy_only=False).astype(np.float32)
    if pa.types.is_integer(kind):
        return pc.fill_null(array.cast(pa.int64()), -1).to_numpy(zero_copy_only=False)
    if pa.types.is_boolean(kind):
        if array.null_count:
            return pc.fill_null(array.cast(pa.int8()), -1).to_numpy(zero_copy_only=False)
        return array.to_numpy(zero_copy_only=False)
    if pa.types.is_string(kind):
        return np.array(pc.fill_null(array, "").to_pylist(), dtype=object)
    raise TypeError(f"{field.name}: {kind} is not a plain column; use records()")


@dataclass
class ReplicarFile:
    """An opened replicar file: its header and its columns."""

    path: Path
    header: dict[str, Any]
    table: pa.Table
    _arrays: dict[str, np.ndarray] | None = field(default=None, repr=False)

    @property
    def players(self) -> list[dict[str, Any]]:
        return self.header["players"]

    @property
    def groups(self) -> list[str]:
        return self.header["groups"]

    def arrays(self) -> dict[str, np.ndarray]:
        """Every plain column as a NumPy array, with the components of a vector stacked last (`ball_position`:
        (rows, 3)), per-player columns stacked by player index (`car_position`: (rows, players, 3)) and
        per-pad columns by pad index (`pad_cooldown`: (rows, pads)). Record lists are left to `records`."""
        if self._arrays is not None:
            return self._arrays
        columns: dict[str, np.ndarray] = {}
        for name, column in zip(self.table.column_names, self.table.columns):
            field = self.table.schema.field(name)
            if pa.types.is_list(field.type) or pa.types.is_struct(field.type):
                continue
            columns[name] = _values(column, field)
        # Per player or pad: car_0_position_x -> car_position_x at index 0.
        indexed: dict[str, dict[int, np.ndarray]] = {}
        plain: dict[str, np.ndarray] = {}
        for name, values in columns.items():
            match = _PLAYER.match(name)
            if match:
                indexed.setdefault(f"{match[1]}_{match[3]}", {})[int(match[2])] = values
                continue
            match = _PAD.match(name)
            if match:
                indexed.setdefault(f"pad_{match[2]}", {})[int(match[1])] = values
                continue
            plain[name] = values
        rows = self.table.num_rows
        for name, by_index in indexed.items():
            count = max(by_index) + 1
            first = next(iter(by_index.values()))
            stacked = np.full((rows, count), np.nan if first.dtype.kind == "f" else -1, dtype=first.dtype)
            for index, values in by_index.items():
                stacked[:, index] = values
            plain[name] = stacked
        self._arrays = _stack_components(plain)
        return self._arrays

    def players_table(self) -> pa.Table:
        """The header's players as a table: index, name, team (0 blue, 1 orange), hitbox, body, key, and one
        column per final statistic (null for a statistic the header's `counted_stats` does not list: unknown)."""
        players = self.header["players"]
        stats = sorted({k for p in players for k in p.get("final_stats", {})})
        columns = {
            "player": pa.array([p["index"] for p in players], pa.uint8()),
            "name": pa.array([p.get("name") for p in players], pa.string()),
            "team": pa.array([p["team"] for p in players], pa.uint8()),
            "hitbox": pa.array([p["hitbox"] for p in players], pa.string()),
            "body_product_id": pa.array([p.get("body_product_id") for p in players], pa.uint32()),
            "key": pa.array([p["key"] for p in players], pa.string()),
        }
        for stat in stats:
            columns[f"final_{stat}"] = pa.array([p.get("final_stats", {}).get(stat) for p in players], pa.int32())
        return pa.table(columns)

    def long(self, prefix: str = "car") -> pa.Table:
        """The per-player columns of `prefix` (`car`, `player`, `network_car`, `network_player`) in long form: one
        row per file row and player, with the row's `frame` (and `sim_tick` and `frame_row` when the file has them),
        `player` and the columns without their index (`car_0_boost` -> `car_boost`), for pandas and SQL. Players
        absent from a row have null values there."""
        by_name: dict[str, dict[int, str]] = {}
        for name in self.table.column_names:
            match = _PLAYER.match(name)
            if match and match[1] == prefix:
                by_name.setdefault(f"{prefix}_{match[3]}", {})[int(match[2])] = name
        players = sorted({i for indexed in by_name.values() for i in indexed})
        rows = self.table.num_rows
        keys = {k: self.table.column(k).combine_chunks() for k in ("frame", "sim_tick", "frame_row")
                if k in self.table.column_names}
        order = pa.array(np.arange(rows, dtype=np.int64))
        pieces = []
        for player in players:
            columns = {**keys, "player": pa.array(np.full(rows, player, np.uint8)), "_row": order}
            for base, indexed in by_name.items():
                if player in indexed:
                    columns[base] = self.table.column(indexed[player]).combine_chunks()
                else:
                    first = self.table.column(next(iter(indexed.values()))).type
                    columns[base] = pa.nulls(rows, first)
            pieces.append(pa.table(columns))
        if not pieces:
            return pa.table({"frame": pa.array([], pa.uint32()), "player": pa.array([], pa.uint8())})
        return pa.concat_tables(pieces).sort_by([("_row", "ascending"), ("player", "ascending")]).drop_columns("_row")

    def records(self, name: str) -> pa.Table:
        """A record-list column (`events`, `ball_contacts`, `boost_pickups`, `prediction_errors`, ...) as one row
        per record, with the `frame` (and `sim_tick`) of its row."""
        column = self.table.column(name).combine_chunks()
        lengths = pc.list_value_length(column).fill_null(0).to_numpy(zero_copy_only=False)
        frames = np.repeat(self.table.column("frame").to_numpy(), lengths)
        flat = pc.list_flatten(column)
        fields = {"frame": pa.array(frames, pa.uint32())}
        if "sim_tick" in self.table.column_names:
            fields["sim_tick"] = pa.array(np.repeat(self.table.column("sim_tick").to_numpy(), lengths), pa.uint64())
        for i, child in enumerate(flat.type):
            fields[child.name] = flat.field(i)
        return pa.table(fields)


def _stack_components(columns: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """`<name>_x`, `_y`, `_z` (and `_w`) of the same shape and type become one array `<name>` with the
    components last; a lone `_x` stays as it is."""
    out: dict[str, np.ndarray] = {}
    used: set[str] = set()
    for name in columns:
        if name in used or not name.endswith("_x"):
            continue
        base = name[:-2]
        parts = []
        for component in _COMPONENTS:
            part = columns.get(f"{base}_{component}")
            if part is None or part.shape != columns[name].shape or part.dtype != columns[name].dtype:
                break
            parts.append(f"{base}_{component}")
        if len(parts) >= 3 and base not in columns:
            out[base] = np.stack([columns[p] for p in parts], axis=-1)
            used.update(parts)
    for name, values in columns.items():
        if name not in used:
            out[name] = values
    return out


def read(path: str | Path, columns: list[str] | None = None, replay: str | Path | None = None) -> ReplicarFile:
    """Open a replicar file (all columns, or the named ones). `replay` is for a file without the `state` group,
    which must be resimulated: that needs the native extra (`pip install replicar[convert]`)."""
    path = Path(path)
    metadata = pq.read_metadata(path).metadata or {}
    if b"replicar" not in metadata:
        raise ValueError(f"{path} has no replicar header: not a replicar file")
    header = json.loads(metadata[b"replicar"])
    if header.get("format_version", 0) > FORMAT_VERSION:
        raise ValueError(
            f"{path} is format version {header['format_version']}; this reader reads {FORMAT_VERSION} and earlier"
        )
    if replay is not None and "state" not in header.get("groups", []):
        # Resimulate the states and read them with the file's own groups.
        native = _native("reading a file without the state group")
        with tempfile.TemporaryDirectory() as directory:
            full = Path(directory) / "full.parquet"
            groups = [g for g in header.get("groups", []) if g != "resimulation"] + ["state"]
            native.resimulate(str(path), str(replay), str(full), precision=header.get("precision", "float32"),
                              groups=groups, all_frames=header.get("all_frames", False),
                              rows=header.get("rows", "frames"), tick_step=header.get("tick_step", 1))
            opened = read(full, columns)
            opened.table = opened.table.combine_chunks()
            return ReplicarFile(path=path, header={**opened.header, "groups": header.get("groups", [])},
                                table=opened.table)
    if columns is not None and "frame" not in columns:
        columns = ["frame", *columns]
    table = pq.read_table(path, columns=columns)
    return ReplicarFile(path=path, header=header, table=table)


def _native(purpose: str):
    try:
        import replicar_native
    except ImportError as error:
        raise ImportError(f"{purpose} needs the native extra: pip install replicar[convert]") from error
    return replicar_native


def iter_frames(path: str | Path, replay: str | Path | None = None, batch_frames: int = 4096):
    """The file's rows in pyarrow record batches of `batch_frames` (resimulating a file without states when
    `replay` is given)."""
    if replay is not None:
        opened = read(path, replay=replay)
        yield from opened.table.to_batches(max_chunksize=batch_frames)
        return
    yield from pq.ParquetFile(path).iter_batches(batch_size=batch_frames)


def convert(replay: str | Path, output: str | Path, *, precision: str = "float32", groups: list[str] | None = None,
            with_groups: list[str] | None = None, all_frames: bool = False, rows: str = "ticks", tick_step: int = 1,
            meshes: str | Path | None = None) -> None:
    """Convert a replay to a replicar file (needs the native extra): a row per simulated tick in play (`rows="ticks"`,
    every `tick_step`-th) or per replay frame (`rows="frames"`). `meshes`: RocketSim's collision meshes, else
    $REPLICAR_MESHES, else ./collision_meshes."""
    _native("converting").convert(str(replay), str(output), precision=precision, groups=groups,
                                  with_groups=with_groups, all_frames=all_frames, rows=rows, tick_step=tick_step,
                                  meshes_dir=None if meshes is None else str(meshes))


def convert_many(replays: list[str | Path], output_dir: str | Path, *, jobs: int | None = None,
                 skip_existing: bool = False, precision: str = "float32", groups: list[str] | None = None,
                 with_groups: list[str] | None = None, all_frames: bool = False, rows: str = "ticks",
                 tick_step: int = 1, meshes: str | Path | None = None) -> list[dict[str, Any]]:
    """Convert replays to `output_dir/<name>.parquet`, `jobs` at a time, and write `output_dir/index.parquet`.
    Returns one dict per replay; a failed one has its `error`."""
    return _native("converting").convert_many(
        [str(r) for r in replays], str(output_dir), jobs=jobs, skip_existing=skip_existing, precision=precision,
        groups=groups, with_groups=with_groups, all_frames=all_frames, rows=rows, tick_step=tick_step,
        meshes_dir=None if meshes is None else str(meshes))


def resimulate(file: str | Path, replay: str | Path, output: str | Path, *, precision: str = "float32",
               groups: list[str] | None = None, with_groups: list[str] | None = None, all_frames: bool = False,
               rows: str = "ticks", tick_step: int = 1, meshes: str | Path | None = None) -> None:
    """Rebuild a file from its replay and its `resimulation` group, without fitting (needs the native extra)."""
    _native("resimulating").resimulate(str(file), str(replay), str(output), precision=precision, groups=groups,
                                       with_groups=with_groups, all_frames=all_frames, rows=rows, tick_step=tick_step,
                                       meshes_dir=None if meshes is None else str(meshes))
