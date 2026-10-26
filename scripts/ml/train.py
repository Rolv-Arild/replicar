"""Trains a per-tick input model on `scripts/ml/dataset.py` samples, scores it against replicar's own controls on the
held-out recordings, and exports it to ONNX.

The five analog controls (throttle, steer, pitch, yaw, roll) are classified into 21 bins each (a tick whose input is
ambiguous gets a spread of bins rather than an average of the candidates); their value is the median of the bin
distribution, which minimises the absolute error. The three buttons (jump, boost, handbrake) are logits. Recordings
are sampled with equal weight, so that the long bot match does not dominate.

usage: python scripts/ml/train.py <out dir> --train a.npz b.npz ... --test c.npz d.npz ...
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import torch
from torch import nn

BINS = 21
CENTERS = np.linspace(-1.0, 1.0, BINS, dtype=np.float32)
# With replicar's controls as input, the model classifies the change to them: an exact "no change" bin and fine bins
# near it, so a value the replay sends passes through unless the model is confident it is wrong.
_STEPS = np.array([0.02, 0.05, 0.1, 0.2, 0.3, 0.45, 0.6, 0.8, 1.0, 1.5, 2.0], dtype=np.float32)
DELTA_CENTERS = np.concatenate([-_STEPS[::-1], [0.0], _STEPS]).astype(np.float32)
ANALOG = ("throttle", "steer", "pitch", "yaw", "roll")
BUTTONS = ("jump", "boost", "handbrake")


class InputModel(nn.Module):
    def __init__(self, features: int, mean: np.ndarray, std: np.ndarray, width: int = 256, delta: bool = False,
                 baseline_at: int = 0):
        super().__init__()
        self.delta = delta
        self.baseline_at = baseline_at
        centers = DELTA_CENTERS if delta else CENTERS
        self.bins = len(centers)
        self.register_buffer("mean", torch.from_numpy(mean))
        self.register_buffer("std", torch.from_numpy(std))
        self.register_buffer("centers", torch.from_numpy(centers))
        self.body = nn.Sequential(
            nn.Linear(features, width), nn.GELU(),
            nn.Linear(width, width), nn.GELU(),
            nn.Linear(width, width), nn.GELU(),
        )
        self.analog = nn.Linear(width, len(ANALOG) * self.bins)
        self.buttons = nn.Linear(width, len(BUTTONS))

    def logits(self, x):
        h = self.body((x - self.mean) / self.std)
        return self.analog(h).view(-1, len(ANALOG), self.bins), self.buttons(h)

    def forward(self, x):
        """Analog values (n, 5) in -1..1 (the median bin) and button probabilities (n, 3)."""
        analog, buttons = self.logits(x)
        cdf = torch.cumsum(torch.softmax(analog, dim=-1), dim=-1)
        median = (cdf < 0.5).sum(dim=-1).clamp(max=self.bins - 1)
        value = self.centers[median]
        if self.delta:
            value = (x[:, self.baseline_at:self.baseline_at + len(ANALOG)] + value).clamp(-1.0, 1.0)
        return value, torch.sigmoid(buttons)


def load(paths, with_replicar):
    parts = [np.load(p) for p in paths]
    joined = lambda d: np.concatenate([d["x"], d["baseline"], d["extra"]], axis=1) if with_replicar else d["x"]
    return [(Path(p).stem, joined(d), d["y"], d["baseline"], d["meta"]) for p, d in zip(paths, parts)]


def scores(y, pred, meta):
    out = {}
    air = meta[:, 2] == 0
    for k, name in enumerate(ANALOG):
        out[name] = float(np.abs(pred[:, k] - y[:, k]).mean())
        out[f"{name} air"] = float(np.abs(pred[air, k] - y[air, k]).mean()) if air.any() else None
    for k, name in enumerate(BUTTONS):
        j = len(ANALOG) + k
        out[name] = float(((pred[:, j] > 0.5) != (y[:, j] > 0.5)).mean())
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("out")
    parser.add_argument("--train", nargs="+", required=True)
    parser.add_argument("--test", nargs="+", required=True)
    parser.add_argument("--epochs", type=int, default=12)
    parser.add_argument("--width", type=int, default=256)
    parser.add_argument("--with-replicar", action="store_true",
                        help="also give the model replicar's own controls of the tick, their sources and the flags")
    args = parser.parse_args()
    torch.manual_seed(0)
    np.random.seed(0)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    train, test = load(args.train, args.with_replicar), load(args.test, args.with_replicar)
    x = np.concatenate([t[1] for t in train])
    y = np.concatenate([t[2] for t in train])
    weight = np.concatenate([np.full(len(t[1]), 1.0 / len(t[1])) for t in train])
    mean, std = x.mean(0), x.std(0) + 1e-3
    # With replicar's view, its controls follow the replay features (`load`).
    baseline_at = np.load(args.train[0])["x"].shape[1] if args.with_replicar else 0
    model = InputModel(x.shape[1], mean.astype(np.float32), std.astype(np.float32), args.width, args.with_replicar,
                       baseline_at)
    if args.with_replicar:
        change = y[:, :len(ANALOG)] - x[:, baseline_at:baseline_at + len(ANALOG)]
        target_bins = np.abs(change[:, :, None] - DELTA_CENTERS).argmin(-1)
    else:
        target_bins = np.abs(y[:, :len(ANALOG), None] - CENTERS).argmin(-1)
    target_buttons = (y[:, len(ANALOG):] > 0.5).astype(np.float32)
    xt, bt, ut = torch.from_numpy(x), torch.from_numpy(target_bins), torch.from_numpy(target_buttons)
    optimizer = torch.optim.AdamW(model.parameters(), lr=2e-3, weight_decay=1e-4)
    steps_per_epoch = len(x) // 4096
    schedule = torch.optim.lr_scheduler.OneCycleLR(optimizer, max_lr=2e-3, total_steps=args.epochs * steps_per_epoch)
    p = torch.from_numpy(weight / weight.sum())
    for epoch in range(args.epochs):
        start, total = time.time(), 0.0
        model.train()
        for _ in range(steps_per_epoch):
            i = torch.multinomial(p, 4096, replacement=True)
            analog, buttons = model.logits(xt[i])
            loss = nn.functional.cross_entropy(analog.reshape(-1, model.bins), bt[i].reshape(-1)) \
                + nn.functional.binary_cross_entropy_with_logits(buttons, ut[i])
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            schedule.step()
            total += loss.item()
        print(f"epoch {epoch}: loss {total / steps_per_epoch:.4f} ({time.time() - start:.0f} s)", flush=True)
    model.eval()
    report = {}
    with torch.no_grad():
        for name, xs, ys, base, meta in test:
            analog, buttons = model(torch.from_numpy(xs))
            pred = np.concatenate([analog.numpy(), buttons.numpy()], axis=1)
            report[name] = {"samples": len(xs), "model": scores(ys, pred, meta), "replicar": scores(ys, base, meta)}
    (out / "report.json").write_text(json.dumps(report, indent=1))
    for name, r in report.items():
        print(f"== {name} ({r['samples']} ticks)   model | replicar")
        for k in r["model"]:
            m, b = r["model"][k], r["replicar"][k]
            if m is not None:
                print(f"  {k:14} {m:.4f} | {b:.4f}")
    torch.save(model.state_dict(), out / "model.pt")
    dummy = torch.zeros(1, x.shape[1])
    torch.onnx.export(model, (dummy,), out / "input_model.onnx", input_names=["features"],
                      output_names=["analog", "buttons"], dynamic_axes={"features": {0: "n"}, "analog": {0: "n"},
                                                                        "buttons": {0: "n"}}, opset_version=17,
                      dynamo=False)
    print("exported", out / "input_model.onnx")


if __name__ == "__main__":
    main()
