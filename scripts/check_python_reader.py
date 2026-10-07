"""Check the pure-Python reader (python/v2, story 8.2) against the Rust reader on generated files: every row's
ball and car bodies, boost, car status and pad cooldowns as `replicar.read(...).arrays()` decodes them must equal
what `dump_states` (the Rust `read_states`) prints, float32 for float32 to the bit and quantized files to the same
decoded value within float32 rounding.

usage: python scripts/check_python_reader.py <dump_states binary> <file.parquet>...   (needs the package on the
path: PYTHONPATH=python/v2/src)
"""

import json
import subprocess
import sys

import numpy as np

import replicar


def main():
    dump, paths = sys.argv[1], sys.argv[2:]
    failures = 0
    for path in paths:
        rust = [json.loads(line) for line in subprocess.run([dump, path], check=True, capture_output=True,
                                                             text=True).stdout.splitlines()]
        f = replicar.read(path)
        a = f.arrays()
        quantized = f.header["precision"] == "quantized"
        tolerance = 1e-6 if quantized else 0.0

        def check(label, expected, actual):
            nonlocal failures
            expected = np.asarray(expected, dtype=np.float64)
            actual = np.asarray(actual, dtype=np.float64)
            both_nan = np.isnan(expected) & np.isnan(actual)
            worst = np.where(both_nan, 0, np.abs(expected - actual)).max(initial=0)
            if worst > tolerance * max(1.0, np.nanmax(np.abs(expected), initial=0)):
                failures += 1
                print(f"  DIFFERENT {label}: worst {worst}")

        check("frame", [r["frame"] for r in rust], a["frame"])
        check("sim_tick", [r["sim_tick"] for r in rust], a["sim_tick"])
        for k, quantity in enumerate(["position", "velocity", "angular_velocity", "rotation"]):
            check(f"ball_{quantity}", [r["ball"][k] for r in rust], a[f"ball_{quantity}"])
            for p in range(len(f.players)):
                expected = [r["cars"][p][0][k] if r["cars"][p] else [np.nan] * (4 if k == 3 else 3) for r in rust]
                check(f"car_{p}_{quantity}", expected, a[f"car_{quantity}"][:, p])
        for p in range(len(f.players)):
            check(f"car_{p}_boost", [r["cars"][p][1] if r["cars"][p] else np.nan for r in rust], a["car_boost"][:, p])
            if [r["status"][p] for r in rust] != list(a["car_status"][:, p]):
                failures += 1
                print(f"  DIFFERENT car_{p}_status")
        check("pad_cooldown", [r["pads"] for r in rust], a["pad_cooldown"])
        print(f"{'equal' if not failures else 'DIFFERENT'}  {path}: {len(rust)} rows, "
              f"car_position {a['car_position'].shape}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
