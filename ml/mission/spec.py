"""Единый контракт признаков и экспортируемой DQN; runtime реализован на C++."""

import numpy as np

INPUT_SIZE = 32
HIDDEN_SIZE = 64
OUTPUT_SIZE = 5
WAIT_ACTION = 4
FEATURE_VERSION = 'g5-dispatch-v1'


def masked_argmax(values, mask):
    """Не допускаем неизвестное действие даже при повреждённом результате сети."""
    values = np.asarray(values)
    mask = np.asarray(mask, dtype=bool)
    if values.shape != mask.shape or values.shape != (OUTPUT_SIZE,):
        raise ValueError('expected five Q values and five mask entries')
    if not np.isfinite(values).all() or not mask.any():
        raise ValueError('nonfinite Q values or empty action mask')
    return int(np.argmax(np.where(mask, values, -np.inf)))


def observation(candidates, now, robot_x, robot_y, loaded=False, congestion=True):
    """Кандидаты передаются в порядке (release_time, job_id), без будущих arrivals."""
    result = np.zeros(INPUT_SIZE, dtype=np.float32)
    mask = np.zeros(OUTPUT_SIZE, dtype=bool)
    if len(candidates) > 4:
        raise ValueError('at most four candidates')
    for slot, job in enumerate(candidates):
        duration = job['pickup_time'] + job['delivery_time'] + job['service_time']
        result[slot * 7:slot * 7 + 7] = [
            job['pickup_time'] / 120.0,
            job['delivery_time'] / 120.0,
            (job['due_time'] - now - duration) / 300.0,
            (now - job['release_time']) / 300.0,
            job['congestion'] / 3.0 if congestion else 0.0,
            job['failures'] / 2.0,
            1.0,
        ]
        mask[slot] = not loaded
    result[28:] = [robot_x / 20.0, robot_y / 20.0, float(loaded), now / 300.0]
    mask[WAIT_ACTION] = True
    if not np.isfinite(result).all():
        raise ValueError('observation contains nonfinite values')
    return result, mask
