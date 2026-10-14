"""Compact U-Net for 3-class RGB-D segmentation (free / obstacle / unknown) of method B."""

import torch
from torch import nn

from preprocess import CHANNELS, CLASSES


def conv_block(inputs, outputs):
    return nn.Sequential(
        nn.Conv2d(inputs, outputs, 3, padding=1, bias=False), nn.BatchNorm2d(outputs), nn.ReLU(inplace=True),
        nn.Conv2d(outputs, outputs, 3, padding=1, bias=False), nn.BatchNorm2d(outputs), nn.ReLU(inplace=True))


class SegNet(nn.Module):
    """Encoder 16-32-64-128 channels with three 2x poolings, decoder with skip connections.

    Input 1x5x240x320 (preprocess.network_input), output 1x3x240x320 logits.
    """

    def __init__(self, width=16):
        super().__init__()
        widths = [width, 2 * width, 4 * width, 8 * width]
        self.encoders = nn.ModuleList()
        channels = CHANNELS
        for out in widths:
            self.encoders.append(conv_block(channels, out))
            channels = out
        self.pool = nn.MaxPool2d(2)
        self.decoders = nn.ModuleList(conv_block(widths[i + 1] + widths[i], widths[i]) for i in reversed(range(3)))
        self.head = nn.Conv2d(widths[0], CLASSES, 1)

    def forward(self, x):
        skips = []
        for index, encoder in enumerate(self.encoders):
            x = encoder(x)
            if index < len(self.encoders) - 1:
                skips.append(x)
                x = self.pool(x)
        for decoder, skip in zip(self.decoders, reversed(skips)):
            x = nn.functional.interpolate(x, scale_factor=2.0, mode='bilinear', align_corners=False)
            x = decoder(torch.cat([x, skip], dim=1))
        return self.head(x)
