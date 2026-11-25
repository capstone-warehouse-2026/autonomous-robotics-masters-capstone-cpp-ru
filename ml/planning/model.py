"""Compact fully convolutional network of the G3 learned cost (method B).

Input  N x 6 x H x W on 0.2 m cells (sim.CHANNELS): static occupancy, observed occupancy at
       t0, t0-1 s, t0-2 s, t0-3 s, observed-area mask; values in [0, 1].
Output N x 1 x H x W: non-negative extra cost per metre (softplus), any H and W.
Dilated 3x3 convolutions give a 6 m receptive field with ~10k parameters, cheap on CPU.
"""

import torch
from torch import nn

from sim import CHANNELS


class CostNet(nn.Module):

    def __init__(self, width=16):
        super().__init__()
        layers = []
        channels = len(CHANNELS)
        for dilation in (1, 2, 4, 8, 1):
            layers += [nn.Conv2d(channels, width, 3, padding=dilation, dilation=dilation), nn.ReLU(inplace=True)]
            channels = width
        self.body = nn.Sequential(*layers)
        self.head = nn.Conv2d(width, 1, 1)
        self.softplus = nn.Softplus()

    def forward(self, x):
        return self.softplus(self.head(self.body(x)))
