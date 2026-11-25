#!/usr/bin/env python3
"""IoU for independently supplied, co-registered occupancy grids in replay map format."""
import argparse
import json
from pathlib import Path
import numpy as np


def read(path):
    data = np.fromstring(Path(path).read_text(), sep=' ')
    if len(data) < 6 or len(data)-6 != int(data[0])*int(data[1]):
        raise ValueError('Invalid map')
    return data[:6], data[6:]


def compare(estimated, reference):
    header, a = read(estimated); other, b = read(reference)
    if not np.allclose(header, other, atol=1e-9, rtol=0):
        raise ValueError('Map lattices differ: require predeclared co-registration, no fitted alignment')
    known = (a >= 0) & (b >= 0); occupied_a = a[known] >= 65; occupied_b = b[known] >= 65
    union = np.count_nonzero(occupied_a | occupied_b)
    return dict(occupied_iou=float(np.count_nonzero(occupied_a & occupied_b)/union) if union else None,
                jointly_known_fraction=float(known.mean()), evaluated_cells=int(known.sum()))

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('map');p.add_argument('reference');p.add_argument('--build-seconds', type=float)
    a = p.parse_args();result = compare(a.map, a.reference)
    result['map_build_seconds'] = a.build_seconds
    print(json.dumps(result, indent=2))
