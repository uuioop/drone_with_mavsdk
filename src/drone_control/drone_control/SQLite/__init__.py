#!/usr/bin/env python3
"""
SQLite数据库模块
提供号牌数据库管理功能
"""

from .license_plate_database import LicensePlateDatabase
from .license_plate_process import LicensePlateProcessor
__all__ = [
    'LicensePlateDatabase',
    'LicensePlateProcessor'
] 