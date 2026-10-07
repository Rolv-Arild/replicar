"""Check a quantized replicar file against the float32 file of the same conversion (docs/v2-plan.md, story 7.2):
every quantized column decoded with its field's `scale` is within half a quantum of the float32 value, and every
other column is identical.

usage: python scripts/check_quantized.py <float32.parquet> <quantized.parquet> [...pairs]   (needs pyarrow, numpy)
"""

import sys

import numpy as np
import pyarrow.parquet as pq


def main():
    paths = sys.argv[1:]
    failures = 0
    for exact_path, quantized_path in zip(paths[::2], paths[1::2]):
        exact = pq.read_table(exact_path)
        quantized = pq.read_table(quantized_path)
        assert exact.column_names == quantized.column_names
        worst = {}
        for field in quantized.schema:
            scale = (field.metadata or {}).get(b"scale")
            if scale is None:
                if not exact.column(field.name).equals(quantized.column(field.name)):
                    failures += 1
                    print(f"  DIFFERENT {field.name}")
                continue
            scale = float(scale)
            decoded = quantized.column(field.name).to_numpy(zero_copy_only=False).astype(np.float64) * scale
            values = exact.column(field.name).to_numpy(zero_copy_only=False).astype(np.float64)
            error = np.nanmax(np.abs(decoded - values)) / scale
            quantity = field.name.rsplit("_", 1)[0].split("_", 2)[-1] if field.name.startswith("car_") else \
                field.name.rsplit("_", 1)[0].split("_", 1)[-1]
            worst[quantity] = max(worst.get(quantity, 0.0), error * scale)
            if error > 0.5 + 1e-6:
                failures += 1
                print(f"  DIFFERENT {field.name}: {error:.3f} quanta")
        print(f"{quantized_path}: worst {', '.join(f'{k} {v:.2g}' for k, v in sorted(worst.items()))}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
