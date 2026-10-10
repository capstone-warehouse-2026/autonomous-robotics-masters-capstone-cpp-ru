"""Trains the method B network on recorded and labelled frames and exports it to ONNX (offline).

Train and validation frames come from the layouts of splits.json (data/g2/<layout>/); test layouts
are never read here. Augmentations change only the network input — darker images, depth noise
growing with distance, depth holes, horizontal flips — while the targets come from the clean
labels; pixels whose depth an augmentation removes become "unknown", as at runtime.

Outputs in <out>/: model.onnx (fixed input 1x5x240x320, see preprocess.py), best.pt, metrics.csv
(learning curves) and model_card.json (data, parameters, metrics, SHA256). evaluate.py checks the
model on the test layouts, release_model.py places it into capstone_perception.
"""

import argparse
import csv
import glob
import hashlib
import json
import os
import time

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset

from generate_layout import load_splits
from model import SegNet
from preprocess import CLASSES, UNKNOWN_CLASS, network_input, network_target

CLASS_NAMES = ('free', 'obstacle', 'unknown')


def frame_paths(data_root, layouts):
    paths = []
    for layout in layouts:
        found = sorted(glob.glob(os.path.join(data_root, layout, 'frame_*.npz')))
        labelled = [p for p in found if os.path.exists(os.path.join(os.path.dirname(p), 'labels', os.path.basename(p)))]
        if not labelled:
            raise SystemExit(f'no labelled frames in {os.path.join(data_root, layout)}; '
                             f'run make g2-dataset LAYOUT={layout} and make g2-labels RUN={layout}')
        paths += labelled
    return paths


def augment(rgb, depth, labels, rng):
    """Input-only augmentations; returns new arrays (full 640x480 resolution)."""
    rgb = rgb.astype(np.float32)
    depth = depth.copy()
    labels = labels.copy()
    if rng.random() < 0.5:  # lighting: brightness and gamma
        rgb = 255.0 * (np.clip(rgb * rng.uniform(0.4, 1.2) / 255.0, 0, 1) ** rng.uniform(0.7, 1.5))
    if rng.random() < 0.5:  # depth noise growing with the square of the distance
        depth = depth + rng.normal(0.0, 1.0, depth.shape).astype(np.float32) * rng.uniform(0.0, 0.005) * depth ** 2
    if rng.random() < 0.5:  # holes: blobs and scattered pixels without depth
        height, width = depth.shape
        for _ in range(rng.integers(1, 7)):
            h, w = rng.integers(10, 80, 2)
            y, x = rng.integers(0, height - h), rng.integers(0, width - w)
            depth[y:y + h, x:x + w] = np.nan
        depth[rng.random(depth.shape) < rng.uniform(0.0, 0.05)] = np.nan
    if rng.random() < 0.5:
        rgb, depth, labels = rgb[:, ::-1], depth[:, ::-1], labels[:, ::-1]
    labels[~np.isfinite(depth)] = 255  # no depth -> unknown, the rule of the runtime
    return np.ascontiguousarray(np.clip(rgb, 0, 255).astype(np.uint8)), np.ascontiguousarray(depth), \
        np.ascontiguousarray(labels)


class Frames(Dataset):
    def __init__(self, paths, train, seed):
        self.paths = paths
        self.train = train
        self.seed = seed
        self.epoch = 0

    def __len__(self):
        return len(self.paths)

    def __getitem__(self, index):
        path = self.paths[index]
        frame = np.load(path)
        labels = np.load(os.path.join(os.path.dirname(path), 'labels', os.path.basename(path)))['labels']
        rgb, depth = frame['rgb'], frame['depth']
        if self.train:
            rng = np.random.default_rng([self.seed, self.epoch, index])
            rgb, depth, labels = augment(rgb, depth, labels, rng)
        return torch.from_numpy(network_input(rgb, depth)), torch.from_numpy(network_target(labels))


def confusion(prediction, target):
    index = target.flatten() * CLASSES + prediction.flatten()
    return torch.bincount(index, minlength=CLASSES * CLASSES).reshape(CLASSES, CLASSES).cpu()


def scores(matrix):
    """IoU per class, mean IoU and the false-free rate (obstacle pixels predicted free / obstacle pixels)."""
    matrix = matrix.double()
    tp = matrix.diag()
    iou = tp / (matrix.sum(0) + matrix.sum(1) - tp).clamp(min=1)
    false_free = matrix[1, 0] / matrix[1].sum().clamp(min=1)
    return {**{f'iou_{name}': float(iou[i]) for i, name in enumerate(CLASS_NAMES)},
            'miou': float(iou.mean()), 'false_free': float(false_free),
            'accuracy': float(tp.sum() / matrix.sum().clamp(min=1))}


def run_epoch(model, loader, device, criterion, optimizer=None):
    model.train(optimizer is not None)
    total, count = 0.0, 0
    matrix = torch.zeros(CLASSES, CLASSES, dtype=torch.long)
    with torch.set_grad_enabled(optimizer is not None):
        for inputs, targets in loader:
            inputs, targets = inputs.to(device, non_blocking=True), targets.to(device, non_blocking=True)
            logits = model(inputs)
            loss = criterion(logits, targets)
            if optimizer is not None:
                optimizer.zero_grad(set_to_none=True)
                loss.backward()
                optimizer.step()
            total += float(loss) * inputs.shape[0]
            count += inputs.shape[0]
            matrix += confusion(logits.argmax(1), targets)
    return total / max(1, count), scores(matrix)


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def export(model, sample, out):
    """ONNX with a fixed 1x5x240x320 input, checked against PyTorch with ONNX Runtime."""
    import onnxruntime

    model = model.cpu().eval()
    path = os.path.join(out, 'model.onnx')
    torch.onnx.export(model, (sample,), path, input_names=['input'], output_names=['logits'], opset_version=17,
                      dynamo=True, external_data=False)
    with torch.no_grad():
        reference = model(sample).numpy()
    session = onnxruntime.InferenceSession(path, providers=['CPUExecutionProvider'])
    logits = session.run(['logits'], {'input': sample.numpy()})[0]
    difference = float(np.abs(logits - reference).max())
    if difference > 1e-3:
        raise SystemExit(f'ONNX output differs from PyTorch by {difference}')
    return path, difference


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--data-root', default='../../data/g2')
    parser.add_argument('--out', default='../../data/g2/models/latest')
    parser.add_argument('--train-layouts', nargs='*', help='default: the train split of splits.json')
    parser.add_argument('--val-layouts', nargs='*', help='default: the val split of splits.json')
    parser.add_argument('--epochs', type=int, default=30)
    parser.add_argument('--batch-size', type=int, default=16)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--width', type=int, default=16, help='channels of the first encoder stage')
    parser.add_argument('--obstacle-weight', type=float, default=2.0, help='loss weight of the obstacle class')
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--seed', type=int, default=0, help='training seed')
    args = parser.parse_args()

    splits = load_splits()
    train_layouts = args.train_layouts or list(splits['train'])
    val_layouts = args.val_layouts or list(splits['val'])
    if set(train_layouts) & set(splits['test']) or set(val_layouts) & set(splits['test']):
        raise SystemExit('test layouts must not be used for training or model selection')
    torch.manual_seed(args.seed)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    os.makedirs(args.out, exist_ok=True)

    train_set = Frames(frame_paths(args.data_root, train_layouts), train=True, seed=args.seed)
    val_set = Frames(frame_paths(args.data_root, val_layouts), train=False, seed=args.seed)
    loader = dict(batch_size=args.batch_size, num_workers=args.workers, pin_memory=device.type == 'cuda')
    train_loader = DataLoader(train_set, shuffle=True, drop_last=len(train_set) > args.batch_size,
                              generator=torch.Generator().manual_seed(args.seed), **loader)
    val_loader = DataLoader(val_set, shuffle=False, **loader)

    model = SegNet(args.width).to(device)
    parameters = sum(p.numel() for p in model.parameters())
    weights = torch.tensor([1.0, args.obstacle_weight, 0.5], device=device)
    criterion = nn.CrossEntropyLoss(weight=weights)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    schedule = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs)
    print(f'device {device}, {parameters} parameters, {len(train_set)} train / {len(val_set)} val frames', flush=True)

    best, best_epoch, started = None, -1, time.time()
    with open(os.path.join(args.out, 'metrics.csv'), 'w', newline='') as stream:
        writer = None
        for epoch in range(args.epochs):
            train_set.epoch = epoch
            train_loss, train_scores = run_epoch(model, train_loader, device, criterion, optimizer)
            val_loss, val_scores = run_epoch(model, val_loader, device, criterion)
            schedule.step()
            row = {'epoch': epoch, 'train_loss': train_loss, 'val_loss': val_loss,
                   **{f'train_{k}': v for k, v in train_scores.items()}, **{f'val_{k}': v for k, v in val_scores.items()}}
            if writer is None:
                writer = csv.DictWriter(stream, fieldnames=list(row))
                writer.writeheader()
            writer.writerow(row)
            stream.flush()
            print(f'epoch {epoch:3d}  loss {train_loss:.3f}/{val_loss:.3f}  val mIoU {val_scores["miou"]:.3f}  '
                  f'obstacle IoU {val_scores["iou_obstacle"]:.3f}  false-free {val_scores["false_free"]:.4f}', flush=True)
            if best is None or val_scores['miou'] > best['miou']:  # model selection on validation only
                best, best_epoch = val_scores, epoch
                torch.save(model.state_dict(), os.path.join(args.out, 'best.pt'))

    model.load_state_dict(torch.load(os.path.join(args.out, 'best.pt'), map_location='cpu'))
    onnx_path, difference = export(model, val_set[0][0][None], args.out)
    card = {
        'model': 'SegNet (compact U-Net), ml/perception/model.py',
        'input': 'float32 1x5x240x320, ml/perception/preprocess.py', 'output': 'logits 1x3x240x320: free, obstacle, unknown',
        'parameters': parameters, 'train_layouts': train_layouts, 'val_layouts': val_layouts,
        'train_frames': len(train_set), 'val_frames': len(val_set), 'arguments': vars(args),
        'device': str(device), 'training_seconds': round(time.time() - started),
        'best_epoch': best_epoch, 'validation': best, 'onnx_sha256': sha256(onnx_path),
        'onnx_max_difference_to_pytorch': difference,
        'git_commit': os.environ.get('GIT_COMMIT', 'unknown'),
    }
    with open(os.path.join(args.out, 'model_card.json'), 'w') as stream:
        json.dump(card, stream, indent=2)
    print(json.dumps({'best_epoch': best_epoch, **best, 'onnx': onnx_path}, indent=2))


if __name__ == '__main__':
    main()
