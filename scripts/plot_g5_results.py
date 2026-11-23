#!/usr/bin/env python3
"""Графики готовых C++ агрегатов и журнала обучения; первичные метрики не пересчитывает."""

import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
MODELS = ROOT / 'ros2_ws/src/capstone_mission/models'
RESULTS = ROOT / 'experiments/g5/isolated-v1'


def read_rows(path):
    with path.open(encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def main():
    figures = RESULTS / 'figures'
    figures.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False, 'svg.hashsalt': 'capstone-g5'})
    figure, axes = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    for axis, prefix, title in zip(axes, ['learning_seed_', 'learning_no_congestion_seed_'],
                                   ['DQN: все признаки', 'DQN: без congestion']):
        for seed in range(3):
            rows = read_rows(MODELS / f'{prefix}{seed}.csv')
            validation = [row for row in rows if row['validation_return']]
            axis.plot([int(row['episode']) for row in validation],
                      [float(row['validation_return']) for row in validation],
                      label=f'seed {seed}')
        axis.set(title=title, xlabel='Эпизоды обучения')
        axis.grid(alpha=0.2)
        axis.legend()
    axes[0].set_ylabel('Средняя validation return (50 эпизодов)')
    figure.tight_layout()
    figure.savefig(figures / 'validation.svg', metadata={'Date': None})
    figure.savefig(figures / 'validation.png', dpi=150)
    plt.close(figure)

    rows = read_rows(RESULTS / 'summary.csv')
    families = ['low', 'high', 'bottleneck', 'blocked', 'unequal']
    figure, axis = plt.subplots(figsize=(9, 4))
    for offset, method, label in [(-0.12, 'classical', 'EDD'), (0.12, 'seed2', 'DQN seed 2')]:
        selected = [next(row for row in rows if row['family'] == family
                         and row['method'] == method) for family in families]
        means = [float(row['on_time_mean']) * 100 for row in selected]
        low = [mean - float(row['ci95_low']) * 100 for row, mean in zip(selected, means)]
        high = [float(row['ci95_high']) * 100 - mean for row, mean in zip(selected, means)]
        axis.errorbar([index + offset for index in range(5)], means,
                      yerr=[low, high], fmt='o', capsize=4, label=label)
    axis.set(xticks=range(5), xticklabels=families, ylim=(0, 102),
             ylabel='Доставлено вовремя / все поступившие, %',
             title='Изолированные таблицы: 20 эпизодов на семейство, 95% bootstrap CI')
    axis.grid(axis='y', alpha=0.2)
    axis.legend()
    figure.tight_layout()
    figure.savefig(figures / 'on_time.svg', metadata={'Date': None})
    figure.savefig(figures / 'on_time.png', dpi=150)
    plt.close(figure)


if __name__ == '__main__':
    main()
