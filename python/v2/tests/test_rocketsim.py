"""Tests of `replicar.rocketsim`: the rotation without the bindings, and a real file with them (skipped without the
`rocketsim` package, `collision_meshes/` or `target/sample/match-default.parquet` at the repository root)."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import replicar
import replicar.rocketsim as bridge

ROOT = Path(__file__).resolve().parents[3]
SAMPLE = ROOT / "target" / "sample" / "match-default.parquet"
MESHES = ROOT / "collision_meshes"


def test_a_quaternion_becomes_rows_forward_right_up():
    fake = SimpleNamespace(RotMat=lambda *values: np.array(values).reshape(3, 3))
    # A quarter turn left about z: forward is +y, right is -x, up stays +z.
    s = np.sqrt(0.5)
    rows = bridge._rot_mat(fake, np.array([0.0, 0.0, s, s]))
    np.testing.assert_allclose(rows, [[0, 1, 0], [-1, 0, 0], [0, 0, 1]], atol=1e-6)
    np.testing.assert_allclose(bridge._rot_mat(fake, np.array([0.0, 0.0, 0.0, 1.0])), np.eye(3))


@pytest.fixture(scope="module")
def sample():
    rs = pytest.importorskip("RocketSim")
    if not SAMPLE.exists() or not MESHES.exists():
        pytest.skip("no sample file or collision meshes")
    rs.init(str(MESHES))
    return rs, replicar.read(SAMPLE)


def test_a_row_restores_into_an_arena_and_steps_like_the_file(sample):
    rs, f = sample
    a = f.arrays()
    row = int(np.flatnonzero((a["segment"][:-1] >= 0) & (a["segment"][:-1] == a["segment"][1:]))[200])
    arena, cars = bridge.arena(f, row)
    assert sorted(cars) == [p["index"] for p in f.players if bridge.has_car(f, row, p["index"])]
    ball = arena.ball.get_state()
    np.testing.assert_allclose([ball.pos.x, ball.pos.y, ball.pos.z], a["ball_position"][row], atol=1e-3)
    for p, car in cars.items():
        s = car.get_state()
        np.testing.assert_allclose([s.pos.x, s.pos.y, s.pos.z], a["car_position"][row, p], atol=1e-3)
        assert car.team == (rs.Team.BLUE if f.players[p]["team"] == 0 else rs.Team.ORANGE)
    cooldowns = sorted(pad.get_state().cooldown for pad in arena.get_boost_pads())
    np.testing.assert_allclose(cooldowns, sorted(a["pad_cooldown"][row]), atol=1e-4)
    # One frame ahead the ball is where the file has it, unless the next row updated it.
    arena.step(int(a["sim_tick"][row + 1] - a["sim_tick"][row]))
    if a["ball_updated"][row + 1] != 1:
        ball = arena.ball.get_state()
        np.testing.assert_allclose([ball.pos.x, ball.pos.y, ball.pos.z], a["ball_position"][row + 1], atol=0.5)
