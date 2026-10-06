"""Size of candidate v2 state-file encodings, built from a v1 Parquet export (no conversion is run).

usage: python scripts/prototype_v2_encodings.py <export.parquet>...   (needs pyarrow and numpy)

The physics columns (ball and cars: position, velocity, angular velocity, rotation) are re-encoded in several
ways; every other typed column of the v1 file (except `frame_json`) is written once as "rest". Rotations: the
3x3 basis matrix as in v1, a unit quaternion (Shepperd's method, sign fixed to w >= 0) or two basis columns
(forward and up; the third is their cross product). Lossy variants quantize to integers. Each variant is
written with the best of plain and BYTE_STREAM_SPLIT encodings (zstd 9), and the lossy ones also with per-series
delta encoding (one column per slot and component). Prints sizes and the worst quantization error.
Nothing is written to disk.
"""

import io
import sys

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

PHYSICS = ["position", "velocity", "angular_velocity", "rotation_columns"]


def values(table, name):
    a = table.column(name).combine_chunks()
    while pa.types.is_list(a.type) or pa.types.is_fixed_size_list(a.type):
        a = a.flatten()
    return a.to_numpy(zero_copy_only=False)


def write(table, encoding=None, dictionary=False):
    b = io.BytesIO()
    pq.write_table(table, b, compression="zstd", compression_level=9, use_dictionary=dictionary,
                   column_encoding=encoding)
    return b.tell()


def best(columns):
    """Smallest of plain and BYTE_STREAM_SPLIT, per column, for a dict of name -> 1-D array."""
    total = 0
    for name, x in columns.items():
        t = pa.table({name: pa.array(x)})
        total += min(write(t), write(t, {name: "BYTE_STREAM_SPLIT"}))
    return total


def delta(columns):
    total = 0
    for name, x in columns.items():
        t = pa.table({name: pa.array(x)})
        total += min(write(t), write(t, {name: "DELTA_BINARY_PACKED"}), write(t, {name: "BYTE_STREAM_SPLIT"}))
    return total


def quaternion(m):
    """Unit quaternions (x, y, z, w), w >= 0, for rotation matrices m[..., row, col]."""
    q = np.empty(m.shape[:-2] + (4,), np.float64)
    t = m[..., 0, 0] + m[..., 1, 1] + m[..., 2, 2]
    cases = np.argmax(np.stack([t, m[..., 0, 0], m[..., 1, 1], m[..., 2, 2]], -1), -1)
    for case in range(4):
        s = cases == case
        a = m[s]
        if case == 0:
            r = np.sqrt(1 + a[:, 0, 0] + a[:, 1, 1] + a[:, 2, 2]) * 2
            q[s] = np.stack([(a[:, 2, 1] - a[:, 1, 2]) / r, (a[:, 0, 2] - a[:, 2, 0]) / r,
                             (a[:, 1, 0] - a[:, 0, 1]) / r, r / 4], -1)
        else:
            i = case - 1
            j, k = (i + 1) % 3, (i + 2) % 3
            r = np.sqrt(1 + a[:, i, i] - a[:, j, j] - a[:, k, k]) * 2
            v = np.empty((len(a), 4))
            v[:, i] = r / 4
            v[:, j] = (a[:, j, i] + a[:, i, j]) / r
            v[:, k] = (a[:, k, i] + a[:, i, k]) / r
            v[:, 3] = (a[:, k, j] - a[:, j, k]) / r
            q[s] = v
    q *= np.where(q[..., 3:4] < 0, -1, 1)
    return q


def matrix(q):
    x, y, z, w = (q[..., i] for i in range(4))
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], -2)


def series(prefix, arr):
    """arr[frame, slot, component] -> one 1-D column per (slot, component)."""
    return {f"{prefix}_{s}_{c}": np.ascontiguousarray(arr[:, s, c]) for s in range(arr.shape[1])
            for c in range(arr.shape[2])}


def interleaved(prefix, arr):
    """arr[frame, slot, component] -> one column with the v1 row-major interleaving."""
    return {prefix: np.ascontiguousarray(arr.reshape(-1))}


def main():
    for path in sys.argv[1:]:
        t = pq.read_table(path)
        n = len(t)
        physics = {}
        for body in ["ball", "car"]:
            for kind in PHYSICS:
                v = values(t, f"{body}_{kind}").astype(np.float32)
                per = 9 if kind == "rotation_columns" else 3
                physics[(body, kind)] = v.reshape(n, -1, per)
        rest_names = [c for c in t.column_names
                      if c != "frame_json" and not any(c == f"{b}_{k}" for b in ["ball", "car"] for k in PHYSICS)]
        rest = write(t.select(rest_names), dictionary=True)
        rest_split = write(t.select(rest_names), {c: "BYTE_STREAM_SPLIT" for c in rest_names
                                                  if pa.types.is_floating(t.schema.field(c).type)
                                                  or (pa.types.is_fixed_size_list(t.schema.field(c).type)
                                                      and pa.types.is_floating(t.schema.field(c).type.value_type))})
        rest = min(rest, rest_split)

        def rotations(body):
            cols = physics[(body, "rotation_columns")]  # [frame, slot, 9]: forward, right, up columns
            m = np.swapaxes(cols.reshape(n, -1, 3, 3), -1, -2).astype(np.float64)  # m[..., row, col]
            ok = np.isfinite(m).all(axis=(-1, -2))
            q = np.zeros(m.shape[:-2] + (4,))
            q[ok] = quaternion(m[ok])
            q[~ok] = np.nan
            back = matrix(q)
            err = np.nanmax(np.abs(back - m)) if ok.any() else 0.0
            return cols, q.astype(np.float32), err

        report = {}
        for name, enc in [("float32, rotation matrix (v1)", "matrix"), ("float32, quaternion", "quat"),
                          ("float32, forward+up columns", "two")]:
            columns = {}
            for body in ["ball", "car"]:
                for kind in ["position", "velocity", "angular_velocity"]:
                    columns.update(interleaved(f"{body}_{kind}", physics[(body, kind)]))
                cols, q, _ = rotations(body)
                if enc == "matrix":
                    columns.update(interleaved(f"{body}_rot", cols))
                elif enc == "quat":
                    columns.update(interleaved(f"{body}_rot", q))
                else:
                    columns.update(interleaved(f"{body}_rot", np.concatenate([cols[..., 0:3], cols[..., 6:9]], -1)))
            report[name] = best(columns)
        rotation_only = {}
        for body in ["ball", "car"]:
            cols, q, err = rotations(body)
            rotation_only[body] = (best(interleaved("m", cols)), best(interleaved("q", q)), err)

        lossy = {}
        for label, scales in [
            ("lossy fine: 0.01 UU, 0.01 UU/s, 1e-4 rad/s, quat 1/32767",
             {"position": 100, "velocity": 100, "angular_velocity": 1e4}),
            ("lossy coarse: 0.1 UU, 0.1 UU/s, 1e-3 rad/s, quat 1/32767",
             {"position": 10, "velocity": 10, "angular_velocity": 1e3}),
        ]:
            inter, per_series, worst = {}, {}, {}
            for body in ["ball", "car"]:
                for kind, s in scales.items():
                    v = physics[(body, kind)].astype(np.float64)
                    qv = np.where(np.isfinite(v), np.round(v * s), np.iinfo(np.int32).min).astype(np.int32)
                    worst[kind] = max(worst.get(kind, 0), float(np.nanmax(np.abs(qv / s - v)[np.isfinite(v)])))
                    inter.update(interleaved(f"{body}_{kind}", qv))
                    per_series.update(series(f"{body}_{kind}", qv))
                _, q, _ = rotations(body)
                qq = np.where(np.isfinite(q), np.round(q * 32767), -32768).astype(np.int16)
                inter.update(interleaved(f"{body}_rot", qq))
                per_series.update(series(f"{body}_rot", qq))
            lossy[label] = (best(inter), delta(per_series), worst)

        print(f"== {path}: {n} frames; every other typed column (rest): {rest / 1e6:.2f} MB")
        for name, size in report.items():
            print(f"  physics {name:34s} {size / 1e6:6.2f} MB   file ~ {(size + rest) / 1e6:6.2f} MB")
        for body, (m, q, err) in rotation_only.items():
            print(f"  {body} rotation alone: matrix {m / 1e6:.2f} MB, quaternion {q / 1e6:.2f} MB "
                  f"(matrix rebuilt from the f64 quaternion: max element error {err:.1e})")
        for label, (a, b, worst) in lossy.items():
            w = ", ".join(f"{k} {v:.3g}" for k, v in worst.items())
            print(f"  physics {label}: interleaved {a / 1e6:.2f} MB, per-series delta {b / 1e6:.2f} MB "
                  f"(file ~ {(min(a, b) + rest) / 1e6:.2f} MB; max error {w})")


if __name__ == "__main__":
    main()
