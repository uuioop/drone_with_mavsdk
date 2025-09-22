#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import sys
import typing

# 修复typing._ClassVar问题
if not hasattr(typing, '_ClassVar'):
    typing._ClassVar = typing.ClassVar

print("typing补丁已应用") 