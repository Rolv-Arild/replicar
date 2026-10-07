"""Read replicar files: Rocket League replays reconstructed as RocketSim states, one Parquet file per replay.

A replicar file is an ordinary Parquet file; this package only adds conveniences (docs/glossary.md):

>>> import replicar
>>> f = replicar.read("match.parquet")
>>> f.header["players"]                   # who is at each player index
>>> a = f.arrays()                        # NumPy: a["car_position"] has shape (frames, players, 3)
>>> f.records("events")                   # a pyarrow table with one row per event and its frame

Floats are float32 with NaN for unknown values; integers use -1 for unknown; quantized columns are decoded with
their scale. Reading needs only pyarrow and NumPy.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pyarrow as pa
import pyarrow.compute as pc
import pyarrow.parquet as pq

__all__ = ["FORMAT_VERSION", "ReplicarFile", "read"]

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
        (frames, 3)), per-player columns stacked by player index (`car_position`: (frames, players, 3)) and
        per-pad columns by pad index (`pad_cooldown`: (frames, pads)). Record lists are left to `records`."""
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

    def records(self, name: str) -> pa.Table:
        """A record-list column (`events`, `ball_contacts`, `boost_pickups`, `prediction_errors`, ...) as one row
        per record, with the frame of its row."""
        column = self.table.column(name).combine_chunks()
        lengths = pc.list_value_length(column).fill_null(0).to_numpy(zero_copy_only=False)
        frames = np.repeat(self.table.column("frame").to_numpy(), lengths)
        flat = pc.list_flatten(column)
        fields = {"frame": pa.array(frames, pa.uint32())}
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
        try:
            from replicar import _native  # noqa: F401  (the native extra)
        except ImportError as error:
            raise ImportError(
                "this file has no state group; resimulating it needs the native extra: pip install replicar[convert]"
            ) from error
        raise NotImplementedError("resimulation from Python comes with the native extra")
    if columns is not None and "frame" not in columns:
        columns = ["frame", *columns]
    table = pq.read_table(path, columns=columns)
    return ReplicarFile(path=path, header=header, table=table)
