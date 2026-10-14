"""Places a trained method B model into capstone_perception together with the golden data of its C++ test.

Copies <model>/model.onnx to models/perception_cnn.onnx of the package (the default of
learned.model_path, stored in Git) and writes models/perception_cnn.json: the model card, the test
evaluation if evaluate.py was run, and the SHA256 of the file. Then runs the model through ONNX
Runtime on one test frame and writes test/data/golden_*: the frame (RGB PNG, depth PNG in mm,
0 = no measurement), the expected labels of CnnClassifier (PNG) and, every STRIDE-th pixel, the
expected network input and logits (golden.yaml, OpenCV FileStorage). test_cnn_classifier.cpp checks
that the C++ preprocessing, inference and post-processing reproduce them.
"""

import argparse
import json
import os
import shutil
import struct
import zlib

import numpy as np
import onnxruntime

from preprocess import labels_from_logits, network_input
from train import sha256

STRIDE = 8


def write_png(path, image):
    """Minimal PNG writer: HxWx3 uint8 (RGB) or HxW uint16 (grey); numpy and zlib only."""
    if image.dtype == np.uint8 and image.ndim == 3:
        depth, colour, rows = 8, 2, image
    elif image.dtype == np.uint16 and image.ndim == 2:
        depth, colour, rows = 16, 0, image.astype('>u2')
    elif image.dtype == np.uint8 and image.ndim == 2:
        depth, colour, rows = 8, 0, image
    else:
        raise ValueError(f'unsupported image {image.dtype} {image.shape}')
    raw = b''.join(b'\0' + row.tobytes() for row in rows)

    def chunk(kind, data):
        return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind + data))

    header = struct.pack('>IIBBBBB', image.shape[1], image.shape[0], depth, colour, 0, 0, 0)
    with open(path, 'wb') as stream:
        stream.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', header) + chunk(b'IDAT', zlib.compress(raw, 9))
                     + chunk(b'IEND', b''))


def opencv_matrix(name, matrix):
    flat = [f'{v:.9e}' for v in matrix.astype(np.float32).ravel()]
    values = ',\n       '.join(', '.join(flat[i:i + 8]) for i in range(0, len(flat), 8))
    return (f'{name}: !!opencv-matrix\n   rows: {matrix.shape[0]}\n   cols: {matrix.shape[1]}\n   dt: f\n'
            f'   data: [ {values} ]\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--model', default='../../data/g2/models/latest')
    parser.add_argument('--frame', default='../../data/g2/test_0/frame_000000.npz',
                        help='test frame for the golden data (a test layout: never seen in training)')
    parser.add_argument('--package', default='../../ros2_ws/src/capstone_perception')
    args = parser.parse_args()

    models = os.path.join(args.package, 'models')
    golden = os.path.join(args.package, 'test', 'data')
    os.makedirs(models, exist_ok=True)
    os.makedirs(golden, exist_ok=True)

    onnx_path = os.path.join(models, 'perception_cnn.onnx')
    shutil.copyfile(os.path.join(args.model, 'model.onnx'), onnx_path)
    with open(os.path.join(args.model, 'model_card.json')) as stream:
        card = json.load(stream)
    if card['onnx_sha256'] != sha256(onnx_path):
        raise SystemExit('model.onnx does not match its model_card.json')
    evaluation_path = os.path.join(args.model, 'evaluation.json')
    if os.path.exists(evaluation_path):
        with open(evaluation_path) as stream:
            card['test_evaluation'] = json.load(stream)
    card['sha256'] = card.pop('onnx_sha256')
    with open(os.path.join(models, 'perception_cnn.json'), 'w') as stream:
        json.dump(card, stream, indent=2)
        stream.write('\n')

    # The depth is stored in whole millimetres; the expected values are computed from the stored
    # depth, so C++ reads exactly the frame Python used.
    frame = np.load(args.frame)
    rgb = frame['rgb']
    millimetres = np.where(np.isfinite(frame['depth']), np.round(np.nan_to_num(frame['depth']) * 1000.0), 0)
    millimetres = np.clip(millimetres, 0, 65535).astype(np.uint16)
    depth = millimetres.astype(np.float32) / 1000.0
    depth[millimetres == 0] = np.nan

    tensor = network_input(rgb, depth)
    session = onnxruntime.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])
    logits = session.run(['logits'], {'input': tensor[None]})[0][0]
    labels = labels_from_logits(logits, depth)

    write_png(os.path.join(golden, 'golden_rgb.png'), rgb)
    write_png(os.path.join(golden, 'golden_depth.png'), millimetres)
    write_png(os.path.join(golden, 'golden_labels.png'), labels)
    sampled_input = tensor[:, ::STRIDE, ::STRIDE]
    sampled_logits = logits[:, ::STRIDE, ::STRIDE]
    with open(os.path.join(golden, 'golden.yaml'), 'w') as stream:
        stream.write('%YAML:1.0\n---\n')
        stream.write(f'# Written by ml/perception/release_model.py from {os.path.relpath(args.frame, "../..")}, '
                     f'model SHA256 {card["sha256"]}.\n')
        stream.write(f'# input: 5 channels x {sampled_input.shape[1]} rows, logits: 3 x {sampled_logits.shape[1]} '
                     f'rows; every {STRIDE}th row and column of the network grid.\n')
        stream.write(f'stride: {STRIDE}\n')
        stream.write(opencv_matrix('input', sampled_input.reshape(-1, sampled_input.shape[2])))
        stream.write(opencv_matrix('logits', sampled_logits.reshape(-1, sampled_logits.shape[2])))

    counts = {name: int((labels == value).sum()) for name, value in (('free', 0), ('obstacle', 1), ('unknown', 255))}
    print(f'{onnx_path}: SHA256 {card["sha256"]}')
    print(f'golden frame {args.frame}: labels {counts}')


if __name__ == '__main__':
    main()
