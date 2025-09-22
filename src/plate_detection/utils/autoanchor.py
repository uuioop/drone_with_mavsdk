# Copyright (c) 2020 Ultralytics
# This file is part of ultralytics, released under the MIT license.
# https://opensource.org/licenses/MIT

import numpy as np
import torch
import yaml


def check_anchor_order(m):
    # Check anchor order against stride order for YOLOv5 Detect() module m, and correct if necessary
    a = m.anchor_grid.prod(-1).view(-1)  # current anchor areas
    stride = m.stride[m.stride.argsort()][0]  # current stride
    if a.shape[0] > 1:  # number of anchors
        a = a.view(-1, 1)  # shape(n,1)
        stride = stride.view(1, -1)  # shape(1,n)
        a = a.expand(-1, stride.shape[1])  # shape(n,n)
        stride = stride.expand(a.shape[0], -1)  # shape(n,n)
        a = a.view(-1)  # shape(n*n,)
        stride = stride.view(-1)  # shape(n*n,)
    return a, stride 