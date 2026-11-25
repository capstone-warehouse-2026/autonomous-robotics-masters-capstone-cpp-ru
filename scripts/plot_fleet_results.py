#!/usr/bin/env python3
"""Сопоставить исходные, согласованные и физически выполненные маршруты флота."""
import argparse
import csv
import itertools
import math
from pathlib import Path

from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from PIL import Image
import yaml


COLORS = ['#1764a3', '#d66b18', '#278545', '#8c4aa2', '#b43c55', '#567d86']


def read_rows(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def load_paths(run):
    plans = {}
    for row in read_rows(run / 'runtime' / 'plans.csv'):
        groups = plans.setdefault(row['plan_id'], {}).setdefault(row['variant'], {})
        path = groups.setdefault(row['robot_id'], {})
        step = int(row['step'])
        point = (float(row['x']), float(row['y']))
        if step in path or not all(map(math.isfinite, point)):
            raise ValueError('Некорректная точка сохранённого плана')
        path[step] = point
    for variants in plans.values():
        for robots in variants.values():
            for robot, steps in robots.items():
                if sorted(steps) != list(range(len(steps))):
                    raise ValueError('Пропущен индекс шага плана')
                robots[robot] = [steps[step] for step in range(len(steps))]
    return plans


def load_truth(run, robots):
    truth = {robot: {} for robot in robots}
    for row in read_rows(run / 'physical' / 'trajectory.csv'):
        if row['source'] != 'ground_truth':
            continue
        robot = row['robot_id']
        if robot not in truth:
            raise ValueError('Ground truth содержит неизвестного робота')
        stamp, x, y = float(row['sim_time']), float(row['x']), float(row['y'])
        if not all(map(math.isfinite, (stamp, x, y))):
            raise ValueError('Нечисловое физическое измерение')
        key = round(stamp * 1e9)
        if key in truth[robot]:
            raise ValueError('Повторное физическое измерение в один момент времени')
        truth[robot][key] = (x, y)
    if any(len(samples) < 2 for samples in truth.values()):
        raise ValueError('Недостаточно физической траектории каждого робота')
    return truth


def load_map(path):
    specification = yaml.safe_load(path.read_text())
    x, y, yaw = specification['origin']
    if abs(yaw) > 1e-8:
        raise ValueError('Поворот карты origin для этой визуализации не поддерживается')
    pixels = np.asarray(Image.open(path.parent / specification['image']).convert('L'))
    height, width = pixels.shape
    resolution = specification['resolution']
    return pixels, (x, x + width * resolution, y, y + height * resolution)


def execution_window(events, plan_id, truth):
    planning = []
    mission_start = None
    for row in events:
        if row['event'] == 'fleet_started':
            mission_start = float(row['sim_time'])
        if row['event'] == 'planning_started':
            detail = dict(field.split('=', 1) for field in row['detail'].split(';')
                          if '=' in field)
            planning.append((detail['plan'], float(row['sim_time'])))
    if mission_start is None:
        raise ValueError('В журнале нет начала миссии')
    for index, (current, start) in enumerate(planning):
        if current != plan_id:
            continue
        end = (planning[index + 1][1] if index + 1 < len(planning)
               else min(max(samples) for samples in truth.values()) * 1e-9)
        return start, end, mission_start
    raise ValueError('Не найдено исполнение выбранного совместного плана')


def draw_background(axis, pixels, extent, focus):
    axis.imshow(pixels, cmap='gray', vmin=0, vmax=255, extent=extent, origin='upper',
                interpolation='nearest', alpha=0.65, zorder=0)
    axis.set_xlim(focus[:2])
    axis.set_ylim(focus[2:])
    axis.set_aspect('equal')
    axis.set_xlabel('x, м')
    axis.set_ylabel('y, м')
    axis.grid(alpha=0.18, linewidth=0.6)


def draw_route(axis, points, color, index, direction=True):
    x, y = zip(*points)
    axis.plot(x, y, color=color, linewidth=2.8 if index == 0 else 2.0,
              linestyle='-' if index % 2 == 0 else '--', alpha=0.95, zorder=3 + index)
    if direction:
        moving = [(start, end) for start, end in zip(points, points[1:]) if start != end]
        for start, end in moving[::2]:
            dx, dy = end[0] - start[0], end[1] - start[1]
            axis.annotate('', xy=(start[0] + 0.7 * dx, start[1] + 0.7 * dy),
                          xytext=(start[0] + 0.3 * dx, start[1] + 0.3 * dy),
                          arrowprops={'arrowstyle': '-|>', 'color': color, 'lw': 1.6},
                          zorder=6)


def mark_endpoints(axis, points, color):
    axis.scatter(*points[0], s=90, marker='o', facecolor='white', edgecolor=color,
                 linewidth=2, zorder=8)
    axis.scatter(*points[-1], s=95, marker='X', color=color, edgecolor='white',
                 linewidth=0.7, zorder=9)


def draw_separation(axis, truth, robots, mission_start, reported_minimum):
    common = sorted(set.intersection(*(set(truth[robot]) for robot in robots)))
    if len(common) < 2:
        raise ValueError('Нет синхронных физических измерений роботов')
    observed_minimum = math.inf
    plotted = False
    for index, (first, second) in enumerate(itertools.combinations(robots, 2)):
        distances = [math.dist(truth[first][stamp], truth[second][stamp]) for stamp in common]
        observed_minimum = min(observed_minimum, min(distances))
        selected = [(stamp * 1e-9 - mission_start, distance)
                    for stamp, distance in zip(common, distances)
                    if stamp * 1e-9 >= mission_start]
        if not selected:
            continue
        times, values = zip(*selected)
        axis.plot(times, values, color=COLORS[index % len(COLORS)], linewidth=1.8,
                  label=f'{first} ↔ {second}')
        plotted = True
    if not plotted or abs(observed_minimum - reported_minimum) > 1e-4:
        raise ValueError('Исходные измерения не совпадают с минимумом из physical audit')
    axis.axhline(0.60, color='#b42328', linestyle='--', linewidth=1.5,
                 label='Порог проверки: 0,60 м')
    axis.axhspan(0, 0.56, color='#b42328', alpha=0.08,
                 label='Сумма радиусов: 0,56 м')
    axis.set_ylim(bottom=0)
    axis.set_xlim(left=0)
    axis.set_xlabel('Время от начала миссии, с симуляции')
    axis.set_ylabel('Между центрами, м')
    axis.set_title(f'Физическое расстояние между роботами · минимум {observed_minimum:.3f} м',
                   loc='left', fontsize=11)
    axis.grid(alpha=0.2)
    axis.legend(loc='upper right', fontsize=8, ncol=2)


def main():
    plt.switch_backend('Agg')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path)
    parser.add_argument('--map', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    audit = yaml.safe_load((args.run / 'physical' / 'audit.yaml').read_text())
    if audit.get('success') is not True:
        raise ValueError('График результата требует успешный physical/audit.yaml')
    robots = audit['robots']
    if len(robots) < 2:
        raise ValueError('Для сравнения нужны как минимум два робота')
    paths = load_paths(args.run)
    truth = load_truth(args.run, robots)
    selected = max(audit['plans'], key=lambda item: item['initial_conflicts'])
    plan_id = str(selected['plan_id'])
    variants = paths[plan_id]
    metadata = yaml.safe_load((args.run / 'runtime' / 'plans' /
                               f'plan_{plan_id}.yaml').read_text())
    events = read_rows(args.run / 'runtime' / 'events.csv')
    start, end, mission_start = execution_window(events, plan_id, truth)
    actual = {robot: [point for stamp, point in sorted(samples.items())
                      if start <= stamp * 1e-9 <= end]
              for robot, samples in truth.items()}
    if any(len(points) < 2 for points in actual.values()):
        raise ValueError('Нет фактического движения за интервал выбранного плана')
    pixels, extent = load_map(args.map)
    all_points = [point for robot in robots for variant in variants.values()
                  for point in variant[robot]] + [point for points in actual.values()
                                                  for point in points]
    x, y = zip(*all_points)
    focus = (max(extent[0], min(x) - 1.2), min(extent[1], max(x) + 1.2),
             max(extent[2], min(y) - 1.2), min(extent[3], max(y) + 1.2))
    figure = plt.figure(figsize=(16, 6.2), layout='constrained')
    layout = figure.add_gridspec(2, 3, height_ratios=[1.35, 1])
    spatial_axes = [figure.add_subplot(layout[0, column]) for column in range(3)]
    titles = [f'Независимые A*: {selected["initial_conflicts"]} конфл.',
              f'{metadata["method"].upper()}: согласованный план', 'Webots: выполненное движение']
    for axis, title in zip(spatial_axes, titles):
        draw_background(axis, pixels, extent, focus)
        axis.set_title(title, loc='left', fontsize=11)
    for index, robot in enumerate(robots):
        color = COLORS[index % len(COLORS)]
        for axis, variant in zip(spatial_axes[:2], ['independent', 'resolved']):
            draw_route(axis, variants[variant][robot], color, index)
            mark_endpoints(axis, variants[variant][robot], color)
        planned = variants['resolved'][robot]
        px, py = zip(*planned)
        spatial_axes[2].plot(px, py, '--', linewidth=1, color=color, alpha=0.3, zorder=1)
        draw_route(spatial_axes[2], actual[robot], color, index, direction=False)
        mark_endpoints(spatial_axes[2], planned, color)
        spatial_axes[2].scatter(*actual[robot][-1], marker='s', s=35, color=color, zorder=10)
    legend = [Line2D([0], [0], color=COLORS[index % len(COLORS)], lw=2, label=robot)
              for index, robot in enumerate(robots)]
    legend.extend([
        Line2D([0], [0], marker='o', markerfacecolor='white', color='#444444', linestyle='',
               label='Начало плана'),
        Line2D([0], [0], marker='X', color='#444444', linestyle='', label='Цель плана'),
        Line2D([0], [0], marker='s', color='#444444', linestyle='', label='Фактическая остановка'),
    ])
    spatial_axes[1].legend(handles=legend, loc='upper center', bbox_to_anchor=(0.5, -0.27),
                           ncol=len(legend), fontsize=8, frameon=False)
    draw_separation(figure.add_subplot(layout[1, :]), truth, robots, mission_start,
                    audit['minimum_robot_separation_m'])
    terminal = 'доставки завершены' if audit['terminal'] == 'completed' else 'отмена подтверждена'
    figure.suptitle(f'Флот в Webots · {metadata["method"].upper()} · {terminal}\n'
                    f'Пространственные панели: совместный план №{plan_id}; '
                    'нижний график: весь физический прогон', fontsize=13)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180)
    plt.close(figure)
    print(args.output)


if __name__ == '__main__':
    main()
