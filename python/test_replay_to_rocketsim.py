import gzip
import json
import tempfile
import unittest
from pathlib import Path

from replay_columnar import (
    iter_columnar_frames,
    load_columnar_numpy,
    read_columnar_header,
    write_columnar,
)
from replay_to_rocketsim import iter_frames, load_numpy, read_header


class LoaderTest(unittest.TestCase):
    def test_stream_and_dense_arrays(self):
        header = {
            "record_type": "header",
            "schema_version": 1,
            "car_slots": [{"slot": 3, "player_key": "player", "team": 0}],
        }
        frame = {
            "record_type": "frame",
            "frame": 0,
            "replay_time": 1.5,
            "timeline_tick": 10,
            "state": {
                "arena_tick": 8,
                "ball": {"physics": {
                    "position": [1, 2, 3],
                    "rotation_columns": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                    "linear_velocity": [4, 5, 6],
                    "angular_velocity": [0, 0, 1],
                }},
                "cars": [{
                    "slot": 3,
                    "physics": {
                        "position": [7, 8, 9],
                        "rotation_columns": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                        "linear_velocity": [1, 0, 0],
                        "angular_velocity": [0, 0, 2],
                    },
                    "controls": {
                        "throttle": 1.0, "steer": -0.5, "pitch": 0.0,
                        "yaw": 0.0, "roll": 0.0,
                        "jump": False, "boost": True, "handbrake": False,
                    },
                    "boost": 33.3,
                    "is_demoed": False,
                }],
                "boost_pads": [{"position": [10, 20, 0], "is_big": True, "is_active": False, "cooldown": 2.0}],
            },
            "observations": {
                "team_scores": [{"value": 2, "frame": 0, "source": "replay"}, None],
                "seconds_remaining": None,
            },
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.jsonl"
            path.write_text("\n".join(map(json.dumps, (header, frame))) + "\n", encoding="utf-8")
            self.assertEqual(read_header(path)["schema_version"], 1)
            self.assertEqual(list(iter_frames(path)), [frame])
            try:
                arrays = load_numpy(path)
            except ImportError:
                self.skipTest("NumPy is not installed")
            self.assertEqual(arrays["car_position"].shape, (1, 1, 3))
            self.assertEqual(arrays["car_position"][0, 0].tolist(), [7, 8, 9])
            self.assertEqual(arrays["ball_angular_velocity"][0].tolist(), [0, 0, 1])
            self.assertEqual(arrays["car_rotation_columns"].shape, (1, 1, 3, 3))
            self.assertEqual(arrays["control_axes_order"], ("throttle", "steer", "pitch", "yaw", "roll"))
            self.assertTrue(arrays["control_buttons"][0, 0, 1])
            self.assertEqual(arrays["boost_pad_position"].tolist(), [[10, 20, 0]])
            self.assertFalse(arrays["boost_pad_active"][0, 0])
            self.assertEqual(arrays["scores"][0, 0], 2)
            self.assertTrue(arrays["car_present"][0, 0])
            self.assertTrue(__import__("numpy").isnan(arrays["scores"][0, 1]))
            compressed = Path(directory) / "sample.jsonl.gz"
            with gzip.open(compressed, "wt", encoding="utf-8") as output:
                output.write(path.read_text(encoding="utf-8"))
            self.assertEqual(read_header(compressed), header)
            self.assertEqual(list(iter_frames(compressed)), [frame])
            for key, value in load_numpy(compressed).items():
                if isinstance(value, __import__("numpy").ndarray):
                    __import__("numpy").testing.assert_equal(value, arrays[key])
                else:
                    self.assertEqual(value, arrays[key])
            try:
                import pyarrow  # noqa: F401
            except ImportError:
                return
            import numpy as np

            for suffix in ("arrow", "parquet"):
                columnar = Path(directory) / f"sample.{suffix}"
                self.assertEqual(write_columnar(path, columnar, batch_size=1), 1)
                self.assertEqual(read_columnar_header(columnar), header)
                self.assertEqual(list(iter_columnar_frames(columnar)), [frame])
                loaded = load_columnar_numpy(columnar)
                self.assertEqual(loaded.keys(), arrays.keys())
                for key, expected in arrays.items():
                    if isinstance(expected, np.ndarray):
                        np.testing.assert_equal(loaded[key], expected)
                    else:
                        self.assertEqual(loaded[key], expected)

    def test_rejects_unknown_schema(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.jsonl"
            path.write_text('{"record_type":"header","schema_version":2}\n', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unsupported schema version"):
                read_header(path)


if __name__ == "__main__":
    unittest.main()
