"""Trains CostNet on data/g3 and exports ONNX with its model card.

    python3 train.py [--data ../../data/g3] [--out ../../data/g3/models/latest] [--no-history]

Loss: squared error on labelled cells (sim.delay_labels), weighted by the metres the probes travelled in
the cell (capped at 2 m). Augmentation: the 8 flips/rotations of the map. The checkpoint with the
lowest validation loss is exported. --no-history zeroes the frames t0-1..t0-3 in training and in
the card, so the C++ node does the same (ablation of projects/03_planning.md).
"""

import argparse
import glob
import hashlib
import json
import os
import time

import numpy as np
import onnx
import torch

import sim
from model import CostNet

HERE = os.path.dirname(os.path.abspath(__file__))
PAST_FRAMES = slice(2, 1 + sim.HISTORY)  # observed_t-1 .. observed_t-3


def load(split_dir):
    samples = []
    for path in sorted(glob.glob(os.path.join(split_dir, 'sample_*.npz'))):
        with np.load(path) as z:
            label = z['label'].astype(np.float32)
            weight = np.minimum(np.nan_to_num(z['length'].astype(np.float32)), 2.0)
            weight[~np.isfinite(label)] = 0.0
            samples.append((z['input'].astype(np.float32), np.nan_to_num(label), weight))
    if not samples:
        raise SystemExit(f'no samples in {split_dir}: run dataset.py first')
    x = torch.from_numpy(np.stack([s[0] for s in samples]))
    y = torch.from_numpy(np.stack([s[1] for s in samples]))[:, None]
    w = torch.from_numpy(np.stack([s[2] for s in samples]))[:, None]
    return x, y, w


def prepare(x, history):
    if not history:
        x = x.clone()
        x[:, PAST_FRAMES] = 0.0
    return x


def augment(x, y, w, generator):
    k = int(torch.randint(4, (1,), generator=generator))
    flip = bool(torch.randint(2, (1,), generator=generator))
    out = []
    for t in (x, y, w):
        t = torch.rot90(t, k, dims=(2, 3))
        out.append(torch.flip(t, dims=(3,)) if flip else t)
    return out


def weighted_loss(prediction, y, w):
    # Squared error: the minimiser is the expected delay, which is what an additive path cost needs
    # (an L1/Huber loss collapses to the median, 0, because most cells have no delay).
    loss = (prediction - y) ** 2
    return (loss * w).sum() / w.sum().clamp(min=1e-6)


@torch.no_grad()
def evaluate(model, x, y, w, history):
    model.eval()
    prediction = torch.cat([model(prepare(x[i:i + 16], history)) for i in range(0, len(x), 16)])
    mask = w > 0
    err = (prediction - y).abs()
    zero = y.abs()
    return {
        'loss': float(weighted_loss(prediction, y, w)),
        'mae': float((err * w).sum() / w.sum()),
        'mae_zero_predictor': float((zero * w).sum() / w.sum()),
        'mae_on_delayed': float(err[mask & (y > 0.5)].mean()) if (mask & (y > 0.5)).any() else 0.0,
        'mean_prediction_on_delayed': float(prediction[mask & (y > 0.5)].mean()) if (mask & (y > 0.5)).any() else 0.0,
        'mean_prediction_on_free': float(prediction[mask & (y == 0)].mean()),
    }


def sha256(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def unobserved_prior(x, y, w):
    """Weighted mean label of labelled cells outside the observed window: the fixed prior there."""
    outside = (x[:, -1:] == 0) & (w > 0)
    return float((y * w)[outside].sum() / w[outside].sum().clamp(min=1e-6))


def export(model, out, history, metrics, prior, args):
    model.eval().cpu()
    path = os.path.join(out, 'model.onnx')
    example = torch.zeros(1, len(sim.CHANNELS), 32, 40)
    torch.onnx.export(model, (example,), path, input_names=['input'], output_names=['extra_cost'],
                      dynamic_shapes={'x': {2: torch.export.Dim('height'), 3: torch.export.Dim('width')}},
                      opset_version=18, external_data=False)
    # Metadata read by capstone_planning::CostModel, so the C++ node needs only this file.
    onnx_model = onnx.load(path)
    for key, value in {'history': 'true' if history else 'false', 'unobserved_prior': repr(prior),
                       'max_extra_cost': repr(sim.MAX_LABEL), 'block': str(sim.BLOCK),
                       'history_period_s': repr(sim.HISTORY_PERIOD), 'channels': str(len(sim.CHANNELS))}.items():
        entry = onnx_model.metadata_props.add()
        entry.key, entry.value = key, value
    onnx.save(onnx_model, path)
    # Golden pair for the C++ test (small, random but reproducible).
    g = torch.Generator().manual_seed(7)
    golden_in = torch.rand(1, len(sim.CHANNELS), 6, 7, generator=g)
    golden_in[:, -1, :, :3] = 0.0  # part of the window unobserved
    if not history:
        golden_in[:, PAST_FRAMES] = 0.0
    with torch.no_grad():
        golden_out = model(golden_in)
    card = {
        'name': 'g3_learned_cost', 'architecture': 'CostNet', 'width': args.width,
        'parameters': sum(p.numel() for p in model.parameters()),
        'channels': sim.CHANNELS, 'history': history, 'history_frames': sim.HISTORY,
        'history_period_s': sim.HISTORY_PERIOD, 'block': sim.BLOCK, 'resolution': sim.RESOLUTION,
        'output': 'extra cost per metre, >= 0', 'max_extra_cost': sim.MAX_LABEL,
        'unobserved_prior': prior, 'epochs': args.epochs, 'seed': args.seed,
        'validation': metrics, 'onnx_sha256': sha256(path),
        'golden': {'shape': list(golden_in.shape), 'input': golden_in.flatten().tolist(),
                   'output': golden_out.flatten().tolist()},
    }
    with open(os.path.join(out, 'model_card.json'), 'w') as f:
        json.dump(card, f, indent=1)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--data', default=os.path.join(HERE, '..', '..', 'data', 'g3'))
    parser.add_argument('--out', default=None)
    parser.add_argument('--epochs', type=int, default=60)
    parser.add_argument('--batch-size', type=int, default=16)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--width', type=int, default=16)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--no-history', action='store_true')
    args = parser.parse_args()
    history = not args.no_history
    out = args.out or os.path.join(args.data, 'models', 'latest' if history else 'no_history')
    os.makedirs(out, exist_ok=True)
    torch.manual_seed(args.seed)
    generator = torch.Generator().manual_seed(args.seed)

    x, y, w = load(os.path.join(args.data, 'train'))
    vx, vy, vw = load(os.path.join(args.data, 'val'))
    print(f'train {len(x)} samples, val {len(vx)}; history={history}')
    model = CostNet(args.width)
    with torch.no_grad():  # start from the mean label: softplus(b) = mean
        mean = float((y * w).sum() / w.sum())
        model.head.bias.fill_(float(np.log(np.expm1(max(mean, 1e-3)))))
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, args.epochs)
    best = None
    for epoch in range(args.epochs):
        model.train()
        started = time.time()
        order = torch.randperm(len(x), generator=generator)
        total = 0.0
        for i in range(0, len(x), args.batch_size):
            idx = order[i:i + args.batch_size]
            bx, by, bw = augment(prepare(x[idx], history), y[idx], w[idx], generator)
            loss = weighted_loss(model(bx), by, bw)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total += float(loss.detach()) * len(idx)
        scheduler.step()
        metrics = evaluate(model, vx, vy, vw, history)
        print(f"epoch {epoch:3d} train {total / len(x):.4f} val {metrics['loss']:.4f} "
              f"mae {metrics['mae']:.4f} (zero {metrics['mae_zero_predictor']:.4f}) {time.time() - started:.1f}s")
        if best is None or metrics['loss'] < best[0]:
            best = (metrics['loss'], {k: v.clone() for k, v in model.state_dict().items()}, metrics, epoch)
    model.load_state_dict(best[1])
    metrics = dict(best[2], epoch=best[3])
    path = export(model, out, history, metrics, unobserved_prior(x, y, w), args)
    print(f'best epoch {best[3]}: {metrics}\nexported {path}')


if __name__ == '__main__':
    main()
