"""Puts a trained model into the capstone_planning package:

    python3 release_model.py [--model ../../data/g3/models/latest]

Copies model.onnx to models/planning_cost.onnx (the default of learned.model_path), its card to
models/planning_cost.json and writes models/planning_cost_golden.txt for the C++ test
(test_learned_cost: N C H W, the input, then the PyTorch output), after checking the ONNX file
against its card and the golden output in ONNX Runtime.
"""

import argparse
import hashlib
import json
import os
import shutil

import numpy as np
import onnxruntime

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.join(HERE, '..', '..', 'ros2_ws', 'src', 'capstone_planning', 'models')


def sha256(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', default=os.path.join(HERE, '..', '..', 'data', 'g3', 'models', 'latest'))
    parser.add_argument('--package', default=PACKAGE)
    args = parser.parse_args()
    with open(os.path.join(args.model, 'model_card.json')) as f:
        card = json.load(f)
    source = os.path.join(args.model, 'model.onnx')
    if sha256(source) != card['onnx_sha256']:
        raise SystemExit('model.onnx does not match its model_card.json')
    golden = card.pop('golden')
    x = np.array(golden['input'], np.float32).reshape(golden['shape'])
    expected = np.array(golden['output'], np.float32)
    session = onnxruntime.InferenceSession(source, providers=['CPUExecutionProvider'])
    output = session.run(None, {session.get_inputs()[0].name: x})[0].reshape(-1)
    if np.abs(output - expected).max() > 1e-4:
        raise SystemExit('ONNX Runtime output differs from PyTorch')
    os.makedirs(args.package, exist_ok=True)
    target = os.path.join(args.package, 'planning_cost.onnx')
    shutil.copyfile(source, target)
    card['sha256'] = card.pop('onnx_sha256')
    with open(os.path.join(args.package, 'planning_cost.json'), 'w') as f:
        json.dump(card, f, indent=2)
        f.write('\n')
    with open(os.path.join(args.package, 'planning_cost_golden.txt'), 'w') as f:
        f.write(' '.join(str(d) for d in golden['shape']) + '\n')
        f.write(' '.join(f'{v:.9g}' for v in x.reshape(-1)) + '\n')
        f.write(' '.join(f'{v:.9g}' for v in expected) + '\n')
    print(f'{target}: SHA256 {card["sha256"]}')


if __name__ == '__main__':
    main()
