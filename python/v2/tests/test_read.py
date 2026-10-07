"""Unit tests of the reader on small files written with pyarrow in the replicar layout."""

import json

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import pytest

import replicar


def write(path, columns, header=None, metadata=None):
    fields = []
    arrays = []
    for name, (array, field_metadata) in columns.items():
        fields.append(pa.field(name, array.type, metadata=field_metadata))
        arrays.append(array)
    table = pa.Table.from_arrays(arrays, schema=pa.schema(fields))
    header = {"format_version": 1, "groups": ["state", "game"], "precision": "float32",
              "players": [{"index": 0}, {"index": 1}], **(header or {})}
    table = table.replace_schema_metadata({"replicar": json.dumps(header)})
    pq.write_table(table, path)


def plain(values, kind):
    return pa.array(values, kind), None


def test_vectors_and_players_are_stacked_and_unknowns_are_nan_or_minus_one(tmp_path):
    path = tmp_path / "x.parquet"
    columns = {"frame": plain([3, 4], pa.uint32())}
    for p, offset in [(0, 0.0), (1, 10.0)]:
        for k, axis in enumerate("xyz"):
            values = [offset + k, None] if p == 1 else [offset + k, offset + k + 1]
            columns[f"car_{p}_position_{axis}"] = plain(values, pa.float32())
        columns[f"car_{p}_update_tick"] = plain([7, None], pa.uint32())
    columns["pad_0_cooldown"] = plain([0.0, 4.0], pa.float32())
    columns["pad_1_cooldown"] = plain([1.0, 0.0], pa.float32())
    write(path, columns)
    a = replicar.read(path).arrays()
    assert a["car_position"].shape == (2, 2, 3)
    assert a["car_position"].dtype == np.float32
    np.testing.assert_array_equal(a["car_position"][0, 1], [10, 11, 12])
    assert np.isnan(a["car_position"][1, 1]).all()
    np.testing.assert_array_equal(a["car_update_tick"], [[7, 7], [-1, -1]])
    np.testing.assert_array_equal(a["pad_cooldown"], [[0, 1], [4, 0]])


def test_quantized_columns_are_decoded_with_their_scale(tmp_path):
    path = tmp_path / "q.parquet"
    scale = {b"scale": b"1e-2", b"group": b"state"}
    write(path, {
        "frame": plain([0, 1], pa.uint32()),
        "ball_position_x": (pa.array([123, -5], pa.int32()), scale),
        "ball_position_y": (pa.array([0, None], pa.int32()), scale),
        "ball_position_z": (pa.array([9275, 9275], pa.int32()), scale),
    }, header={"precision": "quantized"})
    a = replicar.read(path).arrays()
    np.testing.assert_allclose(a["ball_position"][0], [1.23, 0.0, 92.75], rtol=1e-6)
    assert np.isnan(a["ball_position"][1, 1])


def test_names_and_flags(tmp_path):
    path = tmp_path / "n.parquet"
    names = pa.array(["running", None, "kickoff"]).dictionary_encode()
    write(path, {
        "frame": plain([0, 1, 2], pa.uint32()),
        "clock_phase": (names, None),
        "ball_updated": plain([True, False, True], pa.bool_()),
        "car_0_updated": plain([True, None, False], pa.bool_()),
    })
    a = replicar.read(path).arrays()
    assert list(a["clock_phase"]) == ["running", "", "kickoff"]
    assert a["ball_updated"].dtype == bool
    np.testing.assert_array_equal(a["car_updated"][:, 0], [1, -1, 0])


def test_records_are_one_row_per_record_with_their_frame(tmp_path):
    path = tmp_path / "r.parquet"
    event = pa.struct([("kind", pa.string()), ("scoring_team", pa.uint8())])
    events = pa.array([[], [{"kind": "goal", "scoring_team": 1}, {"kind": "goal", "scoring_team": 0}], []],
                      pa.list_(event))
    write(path, {"frame": plain([5, 6, 7], pa.uint32()), "events": (events, None)})
    f = replicar.read(path)
    assert "events" not in f.arrays()
    records = f.records("events").to_pylist()
    assert records == [{"frame": 6, "kind": "goal", "scoring_team": 1},
                       {"frame": 6, "kind": "goal", "scoring_team": 0}]


def test_other_files_are_refused(tmp_path):
    path = tmp_path / "plain.parquet"
    pq.write_table(pa.table({"a": [1]}), path)
    with pytest.raises(ValueError, match="not a replicar file"):
        replicar.read(path)
    later = tmp_path / "later.parquet"
    write(later, {"frame": plain([0], pa.uint32())}, header={"format_version": 99})
    with pytest.raises(ValueError, match="format version 99"):
        replicar.read(later)


def test_a_file_without_states_needs_the_native_extra(tmp_path, monkeypatch):
    import sys
    monkeypatch.setitem(sys.modules, "replicar_native", None)
    path = tmp_path / "s.parquet"
    write(path, {"frame": plain([0], pa.uint32())}, header={"groups": ["game", "resimulation"]})
    with pytest.raises(ImportError, match="native extra"):
        replicar.read(path, replay="x.replay")


def test_players_table_and_long_views(tmp_path):
    path = tmp_path / "p.parquet"
    write(path, {
        "frame": plain([3, 4], pa.uint32()),
        "car_0_boost": plain([10.0, 20.0], pa.float32()),
        "car_1_boost": plain([30.0, None], pa.float32()),
        "pad_0_cooldown": plain([0.0, 1.0], pa.float32()),
    }, header={"players": [
        {"index": 0, "name": "a", "team": 0, "hitbox": "octane", "key": "k0", "final_stats": {"goal": 2}},
        {"index": 1, "name": "b", "team": 1, "hitbox": "dominus", "key": "k1", "final_stats": {"save": 1}},
    ]})
    f = replicar.read(path)
    players = f.players_table().to_pylist()
    assert players[0]["name"] == "a" and players[0]["final_goal"] == 2 and players[0]["final_save"] is None
    assert players[1]["final_save"] == 1
    long = f.long("car").to_pylist()
    assert [(r["frame"], r["player"], r["car_boost"]) for r in long] == [(3, 0, 10.0), (3, 1, 30.0), (4, 0, 20.0),
                                                                         (4, 1, None)]
