"""End-to-end check of the native Python extra (story 8.3), run from the repository root with the `replicar` and
`replicar-native` packages installed: convert a replay with the default groups and one without states but with
the resimulation group; reading the latter with the replay must give the same state arrays as the former;
convert_many converts a folder's replays and writes the index.

usage: python scripts/check_python_native.py <replay> <replay folder> <scratch dir>
"""

import sys
import time
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

import replicar


def main():
    replay, folder, scratch = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    scratch.mkdir(parents=True, exist_ok=True)
    full, light = scratch / "full.parquet", scratch / "light.parquet"
    start = time.time()
    replicar.convert(replay, full)
    replicar.convert(replay, light, groups=["game", "updates", "future", "resimulation"])
    print(f"converted twice in {time.time() - start:.1f} s; sizes {full.stat().st_size} and {light.stat().st_size}")
    a = replicar.read(full).arrays()
    start = time.time()
    b = replicar.read(light, replay=replay).arrays()
    print(f"read without states (resimulated) in {time.time() - start:.1f} s")
    for name in ["ball_position", "car_position", "car_rotation", "car_boost", "car_status", "clock_phase"]:
        assert np.array_equal(a[name], b[name], equal_nan=a[name].dtype.kind == "f"), name
    batches = list(replicar.iter_frames(light, replay=replay, batch_frames=1000))
    assert sum(batch.num_rows for batch in batches) == len(a["frame"])
    replays = sorted(folder.glob("*.replay"))[:4]
    rows = replicar.convert_many(replays, scratch / "many", jobs=4)
    assert len(rows) == len(replays) and all(r["error"] is None for r in rows), rows
    index = pq.read_table(scratch / "many" / "index.parquet")
    assert index.num_rows == len(replays)
    print(f"ok: states equal after resimulation; {len(batches)} batches; convert_many {len(rows)} replays; "
          f"native {replicar_native_version()}")


def replicar_native_version():
    import replicar_native
    return replicar_native.version()


if __name__ == "__main__":
    main()
