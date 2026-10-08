"""Read a RocketSim `.rlpr` recording (versions 7-9): per physics tick, each car's physics frame, position and
controls, and the ball. Layout from RocketSim's reader (`rocketsim_test/src/rlpr`, MIT): magic `RLPR`, a big-endian
flag, the version (u32), a sized `RecordingInfo` (num_cars, hitbox), the tick count (u32), then per tick a sized
record per car (`CarRecord`) and one for the ball (`PhysRecord`, 332 bytes), all little-endian `repr(C)`.

`load(path)` returns {"version", "frame" (ticks,), "pos" (ticks, cars, 3), "controls" (ticks, cars, 8: throttle,
steer, pitch, yaw, roll, jump, boost, handbrake), "on_ground" (ticks, cars), "boost" (ticks, cars, 0-100),
"ball_pos" (ticks, 3), "has_jumped", "double_jumped_or_flipped", "is_flipping" (ticks, cars)}, cached beside the file as `.npz`.
"""

import struct
import sys
from pathlib import Path

import numpy as np

PHYS_SIZE = 332
CAR_SIZES = {7: 596, 8: 600, 9: 604}


def load(path) -> dict:
    path = Path(path)
    cache = path.with_suffix(".npz")
    if cache.exists() and cache.stat().st_mtime >= path.stat().st_mtime:
        return dict(np.load(cache))
    data = path.read_bytes()
    if data[:4] != b"RLPR" or data[4] != 0:
        raise ValueError(f"{path}: not a little-endian RLPR file")
    version = struct.unpack_from("<I", data, 5)[0]
    if version not in CAR_SIZES:
        raise ValueError(f"{path}: RLPR version {version} not supported")
    at = 9
    info_size = struct.unpack_from("<I", data, at)[0]
    num_cars = struct.unpack_from("<I", data, at + 4)[0]
    at += 4 + info_size
    num_ticks = struct.unpack_from("<I", data, at)[0]
    at += 4
    frame = np.zeros(num_ticks, np.int64)
    pos = np.zeros((num_ticks, num_cars, 3), np.float32)
    controls = np.zeros((num_ticks, num_cars, 8), np.float32)
    on_ground = np.zeros((num_ticks, num_cars), bool)
    boost = np.zeros((num_ticks, num_cars), np.float32)
    jumped = np.zeros((num_ticks, num_cars), bool)
    flipped = np.zeros((num_ticks, num_cars), bool)
    flipping = np.zeros((num_ticks, num_cars), bool)
    ball = np.zeros((num_ticks, 3), np.float32)
    car_size = CAR_SIZES[version]
    for t in range(num_ticks):
        car = 0
        while True:
            size = struct.unpack_from("<I", data, at)[0]
            record = at + 4
            at = record + size
            if size == PHYS_SIZE:
                ball[t] = struct.unpack_from("<3f", data, record + 4)
                break
            if size != car_size:
                raise ValueError(f"{path}: record of {size} bytes at tick {t}")
            frame[t] = struct.unpack_from("<I", data, record)[0]
            pos[t, car] = struct.unpack_from("<3f", data, record + 4)
            on_ground[t, car] = data[record + 332] != 0
            jumped[t, car] = data[record + 344] != 0
            flipped[t, car] = data[record + 345] != 0
            flipping[t, car] = data[record + 334] != 0
            boost[t, car] = struct.unpack_from("<f", data, record + 360)[0] * 100.0
            c = struct.unpack_from("<5f3?", data, record + 368)
            controls[t, car] = c
            car += 1
    out = {"version": np.array(version), "frame": frame, "pos": pos, "controls": controls,
           "on_ground": on_ground, "boost": boost, "ball_pos": ball, "has_jumped": jumped,
           "double_jumped_or_flipped": flipped, "is_flipping": flipping}
    np.savez(cache, **out)
    return out


if __name__ == "__main__":
    r = load(sys.argv[1])
    print({k: getattr(v, "shape", v) for k, v in r.items()})
    print("frames", r["frame"][:3], "...", r["frame"][-1], "car 0 pos", r["pos"][0, 0], "controls", r["controls"][100, 0])
