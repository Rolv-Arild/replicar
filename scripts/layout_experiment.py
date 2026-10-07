"""Size of per-player column layouts for the v2 file (docs/v2-plan.md, story 7.1), from v1 Parquet exports.

usage: python scripts/layout_experiment.py <export.parquet>...   (needs pyarrow and numpy; writes nothing)

The car physics of a v1 export (position, velocity, angular velocity: 3 floats per player) are written in three
layouts, each float32 with BYTE_STREAM_SPLIT and zstd 9, and as integers (0.01 units) with DELTA_BINARY_PACKED:
- `interleaved`: one fixed-size list per quantity, players and components interleaved (v1: width 3n);
- `per_component`: one fixed-size list per quantity and component, players interleaved (width n);
- `per_series`: one plain column per player, quantity and component.
"""

import io
import sys

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

QUANTITIES = ["car_position", "car_velocity", "car_angular_velocity"]


def size(table, encoding):
    b = io.BytesIO()
    pq.write_table(table, b, compression="zstd", compression_level=9, use_dictionary=False,
                   column_encoding={name: encoding for name in table.column_names})
    return b.tell()


def fixed(values, width):
    return pa.FixedSizeListArray.from_arrays(pa.array(values.reshape(-1)), width)


def main():
    for path in sys.argv[1:]:
        table = pq.read_table(path, columns=QUANTITIES)
        rows = table.num_rows
        data = {q: np.asarray(table.column(q).combine_chunks().flatten(), np.float32).reshape(rows, -1, 3)
                for q in QUANTITIES}
        players = data[QUANTITIES[0]].shape[1]
        print(f"{path}: {rows} rows, {players} players")
        for label, convert, encoding in [("float32", lambda x: x, "BYTE_STREAM_SPLIT"),
                                         ("int32 0.01", lambda x: np.round(x * 100).astype(np.int32),
                                          "DELTA_BINARY_PACKED")]:
            values = {q: convert(np.nan_to_num(v)) for q, v in data.items()}
            interleaved = pa.table({q: fixed(v, 3 * players) for q, v in values.items()})
            per_component = pa.table({f"{q}_{'xyz'[c]}": fixed(np.ascontiguousarray(v[:, :, c]), players)
                                      for q, v in values.items() for c in range(3)})
            per_series = pa.table({f"{q}_{p}_{'xyz'[c]}": pa.array(np.ascontiguousarray(v[:, p, c]))
                                   for q, v in values.items() for p in range(players) for c in range(3)})
            sizes = [size(t, encoding) / 1e6 for t in (interleaved, per_component, per_series)]
            print(f"  {label}: interleaved {sizes[0]:.2f} MB, per_component {sizes[1]:.2f} MB, "
                  f"per_series {sizes[2]:.2f} MB")


if __name__ == "__main__":
    main()
