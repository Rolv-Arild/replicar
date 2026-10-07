"""Check `replicar.rocketsim` (a row as a state of mtheall's RocketSim bindings) on replicar files.

    python scripts/check_rocketsim_bridge.py <meshes> <file.parquet>... [--rows N] [--all-sources]
    (needs numpy, pyarrow and the `rocketsim` package; PYTHONPATH=python/v2/src)

1. Round trip: every value the bridge sets reads back from the arena as the file has it.
2. One frame ahead: from N rows per file, the bindings step to the next row's sim tick with the row's controls and the
   ball and cars are compared with the next row. Only bodies the next row does not update are compared (else the row
   is the replay's, not a simulation), and cars only when their controls were constant over the interval (no
   per-tick schedule or dodge press; their sources in the row and the next row are neither `schedule` nor `dodge`).
   The baseline is no step at all (the row itself against the next row). In a file of tick rows a row's controls are
   those of its tick, so `--all-sources` compares every car (one tick ahead).
"""

import sys

import numpy as np

import replicar
import replicar.rocketsim as bridge
import RocketSim as rs

PER_TICK = {"schedule", "dodge", "press"}


def vec(v) -> np.ndarray:
    return np.array([v.x, v.y, v.z], dtype=np.float64)


def round_trip(f, row: int) -> float:
    """The largest difference between the row and the arena's state read back."""
    a = f.arrays()
    arena, cars = bridge.arena(f, row)
    worst = 0.0
    ball = arena.ball.get_state()
    worst = max(worst, np.abs(vec(ball.pos) - a["ball_position"][row]).max())
    for p, car in cars.items():
        s = car.get_state()
        for got, name in ((s.pos, "car_position"), (s.vel, "car_velocity"), (s.ang_vel, "car_angular_velocity")):
            worst = max(worst, np.abs(vec(got) - a[name][row, p]).max())
        worst = max(worst, abs(s.boost - a["car_boost"][row, p]))
        assert s.is_on_ground == bool(a["car_is_on_ground"][row, p] == 1)
        assert s.has_flipped == bool(a["car_has_flipped"][row, p] == 1)
        c = car.get_controls()
        worst = max(worst, abs(c.throttle - a["car_controls_throttle"][row, p]))
    return float(worst)


def main() -> None:
    args = sys.argv[1:]
    rows_per_file = 400
    all_sources = "--all-sources" in args
    if all_sources:
        args.remove("--all-sources")
    if "--rows" in args:
        i = args.index("--rows")
        rows_per_file = int(args[i + 1])
        del args[i : i + 2]
    rs.init(args[0])
    rng = np.random.default_rng(0)
    errors: dict[str, list[float]] = {"ball": [], "ball (no step)": [], "car": [], "car (no step)": []}
    worst_round_trip = 0.0
    skipped_hitboxes = 0
    for path in args[1:]:
        f = replicar.read(path)
        a = f.arrays()
        n = f.table.num_rows
        segment = a["segment"]
        for row in rng.choice(n - 1, size=min(rows_per_file, n - 1), replace=False):
            row = int(row)
            ticks = int(a["sim_tick"][row + 1] - a["sim_tick"][row])
            if segment[row] < 0 or segment[row] != segment[row + 1] or not 0 < ticks <= 16:
                continue
            try:
                worst_round_trip = max(worst_round_trip, round_trip(f, row))
                arena, cars = bridge.arena(f, row)
            except ValueError:
                skipped_hitboxes += 1
                continue
            arena.step(ticks)
            if a["ball_updated"][row + 1] != 1:
                got = vec(arena.ball.get_state().pos)
                errors["ball"].append(float(np.linalg.norm(got - a["ball_position"][row + 1])))
                errors["ball (no step)"].append(
                    float(np.linalg.norm(a["ball_position"][row] - a["ball_position"][row + 1]))
                )
            for p, car in cars.items():
                if a["car_updated"][row + 1, p] == 1 or not bridge.has_car(f, row + 1, p):
                    continue
                sources = {a[f"car_{k}_controls_source"][r, p] for k in ("air", "ground") for r in (row, row + 1)}
                if (sources & PER_TICK and not all_sources) or a["car_is_demoed"][row, p] == 1:
                    continue
                got = vec(car.get_state().pos)
                errors["car"].append(float(np.linalg.norm(got - a["car_position"][row + 1, p])))
                errors["car (no step)"].append(
                    float(np.linalg.norm(a["car_position"][row, p] - a["car_position"][row + 1, p]))
                )
    print(f"round trip: largest difference {worst_round_trip:.3g}; rows refused (hitbox) {skipped_hitboxes}")
    for name, values in errors.items():
        v = np.array(values)
        if len(v):
            p50, p90, p99 = np.percentile(v, [50, 90, 99])
            print(f"{name:15} n={len(v):6}  position error UU p50 {p50:8.3f}  p90 {p90:8.3f}  p99 {p99:8.3f}")


if __name__ == "__main__":
    main()
