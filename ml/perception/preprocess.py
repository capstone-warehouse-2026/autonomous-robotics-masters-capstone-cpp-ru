"""Input of the method B network. CnnClassifier (C++) must reproduce this exactly.

From a 640x480 frame (rgb8, depth in metres with NaN = no measurement) the network gets a
5 x 240 x 320 float32 tensor (channels, rows, columns):
  0-2  R, G, B / 255, each the mean of a 2x2 pixel block   (cv::resize INTER_AREA to 320x240)
  3    depth / 8 m clipped to [0, 1], the top-left pixel of each 2x2 block, 0 where invalid
       (cv::resize INTER_NEAREST to 320x240)
  4    validity of that depth pixel: 1 measured, 0 NaN
The output is 3 x 240 x 320 logits of the classes 0 free, 1 obstacle, 2 unknown; class 2 maps to
PixelClass::kUnknown (255). Each output pixel covers the 2x2 block of the full-size image, and a
pixel without depth is unknown whatever the network says (labels_from_logits).
"""

import numpy as np

FULL_WIDTH, FULL_HEIGHT = 640, 480
INPUT_WIDTH, INPUT_HEIGHT = 320, 240
MAX_DEPTH = 8.0
CHANNELS = 5
CLASSES = 3
UNKNOWN_CLASS = 2  # network index of PixelClass::kUnknown (255 in the label images)


def network_input(rgb, depth):
    """rgb: 480x640x3 uint8, depth: 480x640 float32 (NaN = invalid) -> 5x240x320 float32."""
    if rgb.shape != (FULL_HEIGHT, FULL_WIDTH, 3) or depth.shape != (FULL_HEIGHT, FULL_WIDTH):
        raise ValueError(f'expected 640x480 frames, got rgb {rgb.shape}, depth {depth.shape}')
    colour = rgb.reshape(INPUT_HEIGHT, 2, INPUT_WIDTH, 2, 3).astype(np.float32).mean(axis=(1, 3)) / 255.0
    sampled = depth[0::2, 0::2]
    valid = np.isfinite(sampled)
    scaled = np.where(valid, np.clip(np.nan_to_num(sampled), 0.0, MAX_DEPTH) / MAX_DEPTH, 0.0)
    return np.concatenate([colour.transpose(2, 0, 1), scaled[None], valid[None]], axis=0).astype(np.float32)


def labels_from_logits(logits, depth):
    """3x240x320 logits, 480x640 depth -> 480x640 uint8 labels 0 free, 1 obstacle, 255 unknown.

    Arg-max class (the first on ties), each output pixel copied to its 2x2 block, unknown wherever
    the depth is not finite. CnnClassifier::classify returns exactly this.
    """
    classes = np.argmax(logits, axis=0).astype(np.uint8)
    classes[classes == UNKNOWN_CLASS] = 255
    labels = classes.repeat(2, axis=0).repeat(2, axis=1)
    labels[~np.isfinite(depth)] = 255
    return labels


def network_target(labels):
    """480x640 uint8 labels (0, 1, 255) -> 240x320 int64 class indices (0, 1, 2), same sampling as depth."""
    sampled = labels[0::2, 0::2].astype(np.int64)
    sampled[sampled == 255] = UNKNOWN_CLASS
    return sampled
