# Copyright (c) 2020 Ultralytics
# This file is part of ultralytics, released under the MIT license.
# https://opensource.org/licenses/MIT

import numpy as np
from pathlib import Path

import torch


def fitness(x):
    # Model fitness as a weighted combination of metrics
    w = [0.0, 0.0, 0.1, 0.9]  # weights for [P, R, mAP@0.5, mAP@0.5:0.95]
    return (x[:, :4] * w).sum(1) 