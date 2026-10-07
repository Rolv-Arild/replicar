# replicar (Python reader)

Reads replicar files: Rocket League replays reconstructed as RocketSim states, one Parquet file per replay. Reading
needs only pyarrow and NumPy; the files are written by the `replicar` command (`crates/replicar-cli`).

```python
import replicar

f = replicar.read("match.parquet")
f.header["players"]          # name, team, hitbox of each player index
a = f.arrays()               # a["car_position"]: (frames, players, 3) float32, NaN when unknown
f.records("ball_contacts")   # one row per contact, with its frame
```

The words are defined in `docs/glossary.md`. Tests: `PYTHONPATH=src python -m pytest tests` (Python 3.12 or newer).
