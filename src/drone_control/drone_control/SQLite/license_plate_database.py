#!/usr/bin/env python3
"""
单一号牌表实现，用于号牌管理，包含地理位置信息
"""

import sqlite3
from typing import List, Dict, Any

class LicensePlateDatabase:
    """号牌SQLite数据库类"""

    def __init__(self, db_path: str = "license_plates.db"):
        """
        初始化数据库

        Args:
            db_path: 数据库文件路径
        """
        self.db_path = db_path
        self.connection = None
        self._init_database()

    def _init_database(self):
        """初始化数据库表结构"""
        try:
            self.connection = sqlite3.connect(self.db_path, check_same_thread=False)
            cursor = self.connection.cursor()

            # 创建唯一的号牌表，包含经纬度高度信息
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS license_plates (
                    plate_number TEXT PRIMARY KEY NOT NULL,
                    latitude REAL DEFAULT 0.0,
                    longitude REAL DEFAULT 0.0,
                    altitude REAL DEFAULT 0.0,
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP    
                )
            ''')

            self.connection.commit()
            print(f"[数据库] 数据库初始化完成: {self.db_path}")

        except Exception as e:
            print(f"[数据库] 初始化失败: {e}")
            raise

    def add_license_plate(self, plate_number: str, latitude: float = 0.0,
                         longitude: float = 0.0, altitude: float = 0.0) -> bool:
        """
        添加或更新号牌到数据库

        Args:
            plate_number: 号牌号码
            latitude: 纬度
            longitude: 经度
            altitude: 高度

        Returns:
            bool: 是否添加/更新成功
        """
        try:
            cursor = self.connection.cursor()
            # 使用 INSERT OR REPLACE 来处理已存在的号牌，实现更新效果
            cursor.execute('''
                INSERT OR REPLACE INTO license_plates
                (plate_number, latitude, longitude, altitude, timestamp)
                VALUES (?, ?, ?, ?, CURRENT_TIMESTAMP)
            ''', (plate_number, latitude, longitude, altitude))

            self.connection.commit()
            print(f"[数据库] 添加/更新号牌成功: {plate_number}")
            return True

        except Exception as e:
            print(f"[数据库] 添加/更新号牌失败: {e}")
            return False

    def remove_license_plate(self, plate_number: str) -> bool:
        """
        从数据库移除号牌

        Args:
            plate_number: 号牌号码

        Returns:
            bool: 是否移除成功
        """
        try:
            cursor = self.connection.cursor()
            cursor.execute('DELETE FROM license_plates WHERE plate_number = ?', (plate_number,))

            self.connection.commit()
            print(f"[数据库] 移除号牌成功: {plate_number}")
            return True

        except Exception as e:
            print(f"[数据库] 移除号牌失败: {e}")
            return False

    def get_all_license_plates(self) -> List[Dict[str, Any]]:
        """
        获取所有号牌

        Returns:
            List[Dict]: 号牌列表
        """
        try:
            cursor = self.connection.cursor()
            cursor.execute('SELECT * FROM license_plates')

            columns = [description[0] for description in cursor.description]
            results = [dict(zip(columns, row)) for row in cursor.fetchall()]

            return results

        except Exception as e:
            print(f"[数据库] 获取号牌列表失败: {e}")
            return []

    def is_license_plate_exists(self, plate_number: str) -> bool:
        """
        检查号牌是否存在

        Args:
            plate_number: 号牌号码

        Returns:
            bool: 是否存在
        """
        try:
            cursor = self.connection.cursor()
            cursor.execute('SELECT COUNT(*) FROM license_plates WHERE plate_number = ?', (plate_number,))

            count = cursor.fetchone()[0]
            return count > 0

        except Exception as e:
            print(f"[数据库] 检查号牌存在性失败: {e}")
            return False

    def clear_database(self):
        """清空数据库"""
        try:
            cursor = self.connection.cursor()
            cursor.execute('DELETE FROM license_plates')
            self.connection.commit()
            print("[数据库] 数据库已清空")

        except Exception as e:
            print(f"[数据库] 清空数据库失败: {e}")

    def get_database_stats(self) -> Dict[str, int]:
        """
        获取数据库统计信息

        Returns:
            Dict[str, int]: 统计信息
        """
        try:
            cursor = self.connection.cursor()
            cursor.execute('SELECT COUNT(*) FROM license_plates')
            total_plates = cursor.fetchone()[0]

            return {"total_plates": total_plates}

        except Exception as e:
            print(f"[数据库] 获取统计信息失败: {e}")
            return {"total_plates": 0}

    def close(self):
        """关闭数据库连接"""
        if self.connection:
            self.connection.close()
            print("[数据库] 数据库连接已关闭")

    def __del__(self):
        """析构函数，确保关闭连接"""
        self.close()


