#!/usr/bin/env python3
"""Построить проверяемую физическую траекторию Webots поверх реальной карты."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml


def read_rows(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def draw_map(axis, path):
    specification = yaml.safe_load(path.read_text())
    image_path = path.parent / specification['image']
    pixels = plt.imread(image_path)
    height, width = pixels.shape[:2]
    x, y, yaw = specification['origin']
    if abs(yaw) > 1e-8:
        raise ValueError('Эта визуализация поддерживает карты без поворота origin')
    resolution = specification['resolution']
    extent = [x, x + width * resolution, y, y + height * resolution]
    # Верхний ряд PGM соответствует верхнему краю карты, как в Nav2 map_server.
    axis.imshow(pixels, cmap='gray', origin='upper', extent=extent, vmin=0, vmax=255,
                interpolation='nearest', alpha=0.55, zorder=0)
    axis.set_xlim(extent[:2])
    axis.set_ylim(extent[2:])


def draw_episode(axis, episode, trajectory, plans, checks):
    truth = [row for row in trajectory
             if row['episode_id'] == episode and row['source'] == 'ground_truth']
    if len(truth) < 2:
        raise ValueError(f'Недостаточно физического ground truth для {episode}')
    paths = {}
    for row in plans:
        if row['episode_id'] == episode:
            paths.setdefault(row['plan_id'], []).append((float(row['x']), float(row['y'])))
    for index, points in enumerate(paths.values()):
        x, y = zip(*points)
        axis.plot(x, y, '--', color='#9aa4b2', linewidth=1, alpha=0.65,
                  label='Планы Nav2' if index == 0 else None, zorder=1)
    x = [float(row['x']) for row in truth]
    y = [float(row['y']) for row in truth]
    axis.plot(x, y, color='#1261a0', linewidth=2, label='Физическая траектория', zorder=3)
    axis.scatter(x[0], y[0], marker='>', s=95, color='#217a3c', edgecolor='white',
                 label='Начало', zorder=5)
    axis.scatter(x[-1], y[-1], marker='s', s=75, color='#a32929', edgecolor='white',
                 label='Остановка', zorder=5)
    targets = set()
    for index, check in enumerate(checks):
        actual = check['actual_xy']
        target = check['target_xy']
        targets.add(tuple(target))
        axis.scatter(*actual, s=35, color='#e07a18', marker='o',
                     label='Погрузка / доставка: факт' if index == 0 else None, zorder=6)
        axis.plot([target[0], actual[0]], [target[1], actual[1]],
                  color='#e07a18', linewidth=1, zorder=4)
    if targets:
        target_x, target_y = zip(*sorted(targets))
        axis.scatter(target_x, target_y, marker='x', color='black', s=65,
                     label='Станции: цель', zorder=5)
    axis.set_title(episode, fontsize=11)
    axis.set_xlabel('x, м')
    axis.set_ylabel('y, м')
    axis.set_aspect('equal')
    axis.grid(alpha=0.15)
    axis.legend(loc='upper right', fontsize=8, framealpha=0.95)


def main():
    plt.switch_backend('Agg')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path)
    parser.add_argument('--map', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    audit = yaml.safe_load((args.run / 'physical' / 'audit.yaml').read_text())
    trajectory = read_rows(args.run / 'physical' / 'trajectory.csv')
    plans = read_rows(args.run / 'physical' / 'plans.csv')
    episodes = audit.get('episodes', [])
    if not audit.get('success') or not episodes:
        raise ValueError('Публикационный график требует успешно проверенный physical audit')
    figure, axes = plt.subplots(1, len(episodes), figsize=(6.5 * len(episodes), 5.8),
                                squeeze=False, constrained_layout=True)
    for axis, episode in zip(np.ravel(axes), episodes):
        draw_map(axis, args.map)
        draw_episode(axis, episode['episode_id'], trajectory, plans,
                     episode.get('service_position_checks', []))
    provenance_path = args.run / 'provenance.json'
    method = json.loads(provenance_path.read_text()).get('method', '')
    figure.suptitle(f'G5 + Nav2 в Webots: {method}', fontsize=14)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180)
    plt.close(figure)
    print(args.output)


if __name__ == '__main__':
    main()
