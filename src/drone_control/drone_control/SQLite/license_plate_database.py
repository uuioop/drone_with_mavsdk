#!/usr/bin/env python3
"""
号牌SQLite数据库类
最简单的数据库实现，用于号牌管理
"""

import sqlite3
import os
from datetime import datetime
from typing import List, Optional, Dict, Any


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
            self.connection = sqlite3.connect(self.db_path)
            cursor = self.connection.cursor()
            
            # 创建号牌表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS license_plates (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    plate_number TEXT UNIQUE NOT NULL,
                    plate_type TEXT DEFAULT 'unknown',
                    confidence REAL DEFAULT 0.0,
                    location TEXT DEFAULT '',
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    status TEXT DEFAULT 'active',
                    notes TEXT DEFAULT ''
                )
            ''')
            
            # 创建识别记录表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS recognition_records (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    plate_number TEXT NOT NULL,
                    confidence REAL DEFAULT 0.0,
                    image_path TEXT DEFAULT '',
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    location TEXT DEFAULT '',
                    notes TEXT DEFAULT ''
                )
            ''')
            
            # 创建目标号牌表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS target_plates (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    plate_number TEXT UNIQUE NOT NULL,
                    priority INTEGER DEFAULT 1,
                    status TEXT DEFAULT 'active',
                    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
                    notes TEXT DEFAULT ''
                )
            ''')
            
            self.connection.commit()
            print(f"[数据库] 数据库初始化完成: {self.db_path}")
            
        except Exception as e:
            print(f"[数据库] 初始化失败: {e}")
            raise
    
    def add_license_plate(self, plate_number: str, plate_type: str = "unknown", 
                         confidence: float = 0.0, location: str = "", notes: str = "") -> bool:
        """
        添加号牌到数据库
        
        Args:
            plate_number: 号牌号码
            plate_type: 号牌类型
            confidence: 识别置信度
            location: 位置信息
            notes: 备注
            
        Returns:
            bool: 是否添加成功
        """
        try:
            cursor = self.connection.cursor()
            cursor.execute('''
                INSERT OR REPLACE INTO license_plates 
                (plate_number, plate_type, confidence, location, notes)
                VALUES (?, ?, ?, ?, ?)
            ''', (plate_number, plate_type, confidence, location, notes))
            
            self.connection.commit()
            print(f"[数据库] 添加号牌成功: {plate_number}")
            return True
            
        except Exception as e:
            print(f"[数据库] 添加号牌失败: {e}")
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
            cursor.execute('SELECT * FROM license_plates WHERE status = "active"')
            
            columns = [description[0] for description in cursor.description]
            results = []
            
            for row in cursor.fetchall():
                results.append(dict(zip(columns, row)))
            
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
    
    def add_recognition_record(self, plate_number: str, confidence: float = 0.0, 
                             image_path: str = "", location: str = "", notes: str = "") -> bool:
        """
        添加识别记录
        
        Args:
            plate_number: 号牌号码
            confidence: 识别置信度
            image_path: 图像路径
            location: 位置信息
            notes: 备注
            
        Returns:
            bool: 是否添加成功
        """
        try:
            cursor = self.connection.cursor()
            cursor.execute('''
                INSERT INTO recognition_records 
                (plate_number, confidence, image_path, location, notes)
                VALUES (?, ?, ?, ?, ?)
            ''', (plate_number, confidence, image_path, location, notes))
            
            self.connection.commit()
            print(f"[数据库] 添加识别记录成功: {plate_number}")
            return True
            
        except Exception as e:
            print(f"[数据库] 添加识别记录失败: {e}")
            return False
    
    def set_target_plate(self, plate_number: str, priority: int = 1, notes: str = "") -> bool:
        """
        设置目标号牌
        
        Args:
            plate_number: 号牌号码
            priority: 优先级
            notes: 备注
            
        Returns:
            bool: 是否设置成功
        """
        try:
            cursor = self.connection.cursor()
            cursor.execute('''
                INSERT OR REPLACE INTO target_plates 
                (plate_number, priority, notes)
                VALUES (?, ?, ?)
            ''', (plate_number, priority, notes))
            
            self.connection.commit()
            print(f"[数据库] 设置目标号牌成功: {plate_number}")
            return True
            
        except Exception as e:
            print(f"[数据库] 设置目标号牌失败: {e}")
            return False
    
    def get_target_plates(self) -> List[Dict[str, Any]]:
        """
        获取所有目标号牌
        
        Returns:
            List[Dict]: 目标号牌列表
        """
        try:
            cursor = self.connection.cursor()
            cursor.execute('SELECT * FROM target_plates WHERE status = "active" ORDER BY priority DESC')
            
            columns = [description[0] for description in cursor.description]
            results = []
            
            for row in cursor.fetchall():
                results.append(dict(zip(columns, row)))
            
            return results
            
        except Exception as e:
            print(f"[数据库] 获取目标号牌失败: {e}")
            return []
    
    def clear_database(self):
        """清空数据库"""
        try:
            cursor = self.connection.cursor()
            cursor.execute('DELETE FROM license_plates')
            cursor.execute('DELETE FROM recognition_records')
            cursor.execute('DELETE FROM target_plates')
            
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
            
            # 获取号牌总数
            cursor.execute('SELECT COUNT(*) FROM license_plates WHERE status = "active"')
            total_plates = cursor.fetchone()[0]
            
            # 获取识别记录总数
            cursor.execute('SELECT COUNT(*) FROM recognition_records')
            total_records = cursor.fetchone()[0]
            
            # 获取目标号牌总数
            cursor.execute('SELECT COUNT(*) FROM target_plates WHERE status = "active"')
            total_targets = cursor.fetchone()[0]
            
            return {
                "total_plates": total_plates,
                "total_records": total_records,
                "total_targets": total_targets
            }
            
        except Exception as e:
            print(f"[数据库] 获取统计信息失败: {e}")
            return {"total_plates": 0, "total_records": 0, "total_targets": 0}
    
    def close(self):
        """关闭数据库连接"""
        if self.connection:
            self.connection.close()
            print("[数据库] 数据库连接已关闭")
    
    def __del__(self):
        """析构函数，确保关闭连接"""
        self.close()


# 简单的使用示例
if __name__ == "__main__":
    # 创建数据库实例
    db = LicensePlateDatabase("test_license_plates.db")
    
    # 添加一些测试数据
    db.add_license_plate("京A12345", "普通车牌", 0.95, "北京市朝阳区", "测试数据")
    db.add_license_plate("沪B67890", "普通车牌", 0.88, "上海市浦东新区", "测试数据")
    
    # 设置目标号牌
    db.set_target_plate("京A12345", 1, "主要目标")
    
    # 添加识别记录
    db.add_recognition_record("京A12345", 0.95, "/path/to/image.jpg", "北京市朝阳区", "自动识别")
    
    # 获取统计信息
    stats = db.get_database_stats()
    print(f"数据库统计: {stats}")
    
    # 关闭数据库
    db.close() 