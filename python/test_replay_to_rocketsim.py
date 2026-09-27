import json
import tempfile
import unittest
from pathlib import Path

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
            "replay_time": 1.5,
            "timeline_tick": 10,
            "state": {
                "arena_tick": 8,
                "ball": {"physics": {"position": [1, 2, 3], "linear_velocity": [4, 5, 6]}},
                "cars": [{"slot": 3, "physics": {"position": [7, 8, 9], "linear_velocity": [1, 0, 0]}, "boost": 33.3}],
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
            self.assertEqual(arrays["scores"][0, 0], 2)
            self.assertTrue(arrays["car_present"][0, 0])
            self.assertTrue(__import__("numpy").isnan(arrays["scores"][0, 1]))

    def test_rejects_unknown_schema(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.jsonl"
            path.write_text('{"record_type":"header","schema_version":2}\n', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unsupported schema version"):
                read_header(path)


if __name__ == "__main__":
    unittest.main()
