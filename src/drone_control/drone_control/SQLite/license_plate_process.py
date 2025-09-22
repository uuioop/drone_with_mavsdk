#!/usr/bin/env python3
"""
号牌处理模块 (修改版)
"""

# 假设 modified_license_plate_database.py 与此文件在同一目录下
from drone_control.SQLite import LicensePlateDatabase

class LicensePlateProcessor:
    """号牌处理类"""
    def __init__(self, logger):
        self.logger = logger
        # 初始化号牌数据库
        self.license_plate_database = LicensePlateDatabase()
        self.current_target_plate = ""
        self.recognized_plate = ""

    def process_license_plate(self, plate_text):
        """处理号牌识别结果"""
        try:
            self.logger.info(f"[号牌处理] 收到号牌识别结果: {plate_text}")
            self.recognized_plate = plate_text
            return self._compare_license_plate()

        except Exception as e:
            self.logger.error(f"处理号牌识别结果失败: {e}")
            return False

    def _compare_license_plate(self):
        """号牌比对逻辑"""
        if not self.current_target_plate:
            self.logger.info("[号牌比对] 未设置目标号牌，跳过比对")
            return False
            
        if self.recognized_plate == self.current_target_plate:
            self.logger.info(f"[号牌比对] 匹配成功！找到目标号牌: {self.recognized_plate}")
            return True
        else:
            self.logger.info(f"[号牌比对] 识别到 '{self.recognized_plate}', 与目标 '{self.current_target_plate}' 不匹配，继续搜索...")
            return False

    def set_target_license_plate(self, plate_number: str):
        """设置目标号牌"""
        self.current_target_plate = plate_number
        self.logger.info(f"[设置目标] 目标号牌设置为: {plate_number}")

    def add_license_plate_to_database(self, plate_number: str, latitude: float, longitude: float, altitude: float):
        """
        添加号牌到数据库
        如果号牌已存在，则会更新其位置信息
        """
        # is_license_plate_exists 的检查是可选的，因为 add_license_plate 内部使用 INSERT OR REPLACE
        # 但为了日志清晰，这里保留判断
        if self.license_plate_database.is_license_plate_exists(plate_number):
             self.logger.info(f"[数据库] 号牌 '{plate_number}' 已存在，将更新其位置信息。")
        else:
             self.logger.info(f"[数据库] 添加新号牌到数据库: {plate_number}")
             
        self.license_plate_database.add_license_plate(plate_number, latitude, longitude, altitude)


    def remove_license_plate_from_database(self, plate_number: str):
        """从数据库中移除号牌"""
        if self.license_plate_database.is_license_plate_exists(plate_number):
            self.license_plate_database.remove_license_plate(plate_number)
            self.logger.info(f"[数据库] 从数据库移除号牌: {plate_number}")
        else:
            self.logger.info(f"[数据库] 号牌 '{plate_number}' 不存在，无法移除。")

    def clear_database(self):
        """清空数据库"""
        self.license_plate_database.clear_database()
        self.logger.info("[数据库] 数据库已清空")

    def get_database_size(self) -> int:
        """获取数据库大小"""
        stats = self.license_plate_database.get_database_stats()
        return stats.get("total_plates", 0)

    def is_target_plate(self, plate_number: str) -> bool:
        """检查是否为当前设定的目标号牌"""
        return plate_number == self.current_target_plate

    def is_in_database(self, plate_number: str) -> bool:
        """检查号牌是否在数据库中"""
        return self.license_plate_database.is_license_plate_exists(plate_number)

    def is_target_in_database(self) -> bool:
        """判断当前目标号牌是否在数据库中"""
        if not self.current_target_plate:
            self.logger.warning("[目标检查] 未设置目标号牌")
            return False

        is_in_db = self.is_in_database(self.current_target_plate)
        self.logger.info(f"[目标检查] 目标号牌 '{self.current_target_plate}' 在数据库中: {is_in_db}")
        return is_in_db
