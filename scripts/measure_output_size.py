"""Where the bytes of a v1 Parquet export go, and what candidate v2 encodings would cost.

usage: python scripts/measure_output_size.py <export.parquet>...   (needs pyarrow and numpy)

For each export it prints:
1. the compressed size per column of the main file;
2. the typed columns rewritten without `frame_json`, with rotations as quaternions (a size proxy: the
   magnitudes of the components, signs not resolved), with BYTE_STREAM_SPLIT on float columns, at zstd 3/9/19;
3. car state quantized to integers (0.01 UU, 0.01 UU/s, 1e-4 rad/s, rotation 1/30000) in a car-sorted long
   layout with DELTA_BINARY_PACKED (lossy; for comparison only);
4. the typed cost of what only `frame_json` holds: the remaining car-state fields, the observations with their
   update frames, and the position residuals.
Nothing is written to disk.
"""

import io
import json
import os
import sys
from collections import Counter

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq


def flatten(array):
    while pa.types.is_list(array.type) or pa.types.is_fixed_size_list(array.type):
        array = array.flatten()
    return array.to_numpy(zero_copy_only=False).astype(np.float32)


def written_size(table, level=3, split_floats=False, encodings=None, dictionary=True):
    encoding = dict(encodings or {})
    if split_floats:
        encoding.update({f.name: "BYTE_STREAM_SPLIT" for f in table.schema if "float" in str(f.type)})
    use_dictionary = dictionary and not encoding
    if encoding and dictionary:
        use_dictionary = [f.name for f in table.schema if f.name not in encoding]
    buffer = io.BytesIO()
    pq.write_table(table, buffer, compression="zstd", compression_level=level,
                   use_dictionary=use_dictionary, column_encoding=encoding or None)
    return buffer.tell()


def quaternion_magnitudes(m):
    """|x|, |y|, |z|, |w| of the rotation whose basis columns are m[..., column, component]."""
    r = np.swapaxes(m, -1, -2)
    t = r[..., 0, 0] + r[..., 1, 1] + r[..., 2, 2]
    parts = [1 + r[..., 0, 0] - r[..., 1, 1] - r[..., 2, 2], 1 - r[..., 0, 0] + r[..., 1, 1] - r[..., 2, 2],
             1 - r[..., 0, 0] - r[..., 1, 1] + r[..., 2, 2], 1 + t]
    return np.stack([np.sqrt(np.maximum(0, p)) / 2 for p in parts], -1).astype(np.float32)


def column_breakdown(path):
    md = pq.ParquetFile(path).metadata
    compressed = Counter()
    for g in range(md.num_row_groups):
        rg = md.row_group(g)
        for c in range(rg.num_columns):
            col = rg.column(c)
            compressed[col.path_in_schema.split(".")[0]] += col.total_compressed_size
    total = sum(compressed.values())
    print(f"  columns: {total / 1e6:.2f} MB in {len(compressed)} columns; largest:")
    for name, size in compressed.most_common(8):
        print(f"    {name:28s} {size / 1e6:7.3f} MB {100 * size / total:5.1f}%")


def typed_encodings(path, table):
    base = table.drop_columns(["frame_json"])
    quat = base
    n = len(table)
    for name in [c for c in base.column_names if c.endswith("rotation_columns")]:
        values = flatten(table.column(name).combine_chunks())
        q = quaternion_magnitudes(values.reshape(n, -1, 3, 3)).reshape(n, -1)
        quat = quat.drop_columns([name]).append_column(
            name.replace("rotation_columns", "quaternion"),
            pa.FixedSizeListArray.from_arrays(pa.array(q.ravel()), q.shape[1]))
    rows = [
        ("v1 file on disk", os.path.getsize(path)),
        ("without frame_json", written_size(base)),
        ("  + quaternions", written_size(quat)),
        ("  + byte_stream_split floats", written_size(quat, split_floats=True, dictionary=False)),
        ("  + zstd 9", written_size(quat, 9, True, dictionary=False)),
        ("  + zstd 19", written_size(quat, 19, True, dictionary=False)),
    ]
    for name, size in rows:
        print(f"  {name:32s} {size / 1e6:7.2f} MB")


def quantized_cars(table):
    n = len(table)
    columns = {}
    for kind, per, scale in [("position", 3, 100), ("velocity", 3, 100), ("angular_velocity", 3, 1e4),
                             ("rotation_columns", 9, 3e4)]:
        values = flatten(table.column(f"car_{kind}").combine_chunks())
        v = values.reshape(n, -1, per)[..., :6 if per == 9 else per]
        for j in range(v.shape[-1]):
            series = np.ascontiguousarray(v[..., j].T).ravel()
            columns[(kind, j, scale)] = series
    floats = pa.table({f"{k}{j}": pa.array(x) for (k, j, _), x in columns.items()})
    ints = pa.table({f"{k}{j}": pa.array(np.where(np.isfinite(x), np.round(x * s), 0).astype(np.int32))
                     for (k, j, s), x in columns.items()})
    print(f"  cars, float32 long + split, zstd 9     {written_size(floats, 9, True, dictionary=False) / 1e6:7.2f} MB")
    print(f"  cars, int32 quantized + delta, zstd 9  "
          f"{written_size(ints, 9, encodings={c: 'DELTA_BINARY_PACKED' for c in ints.column_names}, dictionary=False) / 1e6:7.2f} MB")


def frame_json_only(table):
    skip = {"physics", "controls", "slot", "team", "boost", "is_demoed"}
    scalar = lambda x: None if x is None else float(x)
    car_rows, observation_rows, residuals = [], [], []
    for text in table.column("frame_json").to_pylist():
        frame = json.loads(text)
        car_row, observation_row = {}, {}
        for car in frame["state"]["cars"]:
            for key, value in car.items():
                if key in skip:
                    continue
                if isinstance(value, dict):
                    value = list(value.values())
                for j, x in enumerate(value if isinstance(value, list) else [value]):
                    car_row[f"s{car['slot']}_{key}{j}"] = scalar(x)
        for index, car in enumerate(frame["observations"]["cars"]):
            for key, entry in list(car["body"].items()) + list((car.get("inputs") or {}).items()):
                if entry is None:
                    continue
                value = entry["value"]
                for j, x in enumerate(value if isinstance(value, list) else [value]):
                    observation_row[f"c{index}_{key}{j}"] = scalar(x)
                observation_row[f"c{index}_{key}_frame"] = entry["frame"]
        car_rows.append(car_row)
        observation_rows.append(observation_row)
        residuals.extend({k: v for k, v in r.items() if not isinstance(v, (list, dict))}
                         for r in frame["position_residuals"])
    cars = pa.Table.from_pylist(car_rows)
    cars = cars.cast(pa.schema([pa.field(c, pa.float32()) for c in cars.column_names]))
    observations = pa.Table.from_pylist(observation_rows)
    observations = observations.cast(pa.schema(
        [pa.field(c, pa.int32() if c.endswith("_frame") else pa.float32()) for c in observations.column_names]))
    residual_table = pa.Table.from_pylist(residuals)
    print(f"  only in frame_json: other car state {written_size(cars, 9, True) / 1e6:.2f} MB, "
          f"observations {written_size(observations, 9, True) / 1e6:.2f} MB, "
          f"position residuals ({len(residuals)} rows) {written_size(residual_table, 9, True) / 1e6:.2f} MB")


def main():
    for path in sys.argv[1:]:
        table = pq.read_table(path)
        print(f"== {path}: {len(table)} frames")
        column_breakdown(path)
        typed_encodings(path, table)
        quantized_cars(table)
        frame_json_only(table)


if __name__ == "__main__":
    main()
