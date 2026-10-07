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


def test_a_row_as_an_rlgym_game_state_restores_through_the_engine(sample):
    pytest.importorskip("rlgym.rocket_league")
    from rlgym.rocket_league.sim import RocketSimEngine

    import replicar.rlgym

    _, f = sample
    a = f.arrays()
    row = int(np.flatnonzero(a["segment"] >= 0)[300])
    state = replicar.rlgym.game_state(f, row, agent_ids={p["index"]: p["name"] for p in f.players})
    assert set(state.cars) == {p["name"] for p in f.players if bridge.has_car(f, row, p["index"])}
    out = RocketSimEngine(rlbot_delay=False).set_state(state, {})
    np.testing.assert_allclose(out.ball.position, a["ball_position"][row], atol=1e-3)
    for p in f.players:
        if p["name"] in out.cars:
            car = out.cars[p["name"]]
            np.testing.assert_allclose(car.physics.position, a["car_position"][row, p["index"]], atol=1e-3)
            np.testing.assert_allclose(car.physics.rotation_mtx, state.cars[p["name"]].physics.rotation_mtx, atol=1e-5)
    # The engine indexes pads in its arena's order; the timers come back where they were put.
    np.testing.assert_allclose(out.boost_pad_timers, state.boost_pad_timers, atol=1e-5)
    assert sorted(state.boost_pad_timers) == pytest.approx(sorted(np.maximum(a["pad_cooldown"][row], 0)), abs=1e-5)
