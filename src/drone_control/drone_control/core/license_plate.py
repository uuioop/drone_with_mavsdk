#!/usr/bin/env python3
"""
号牌处理模块
"""

class LicensePlateProcessor:
    """号牌处理类"""
    def __init__(self, logger):
        self.logger = logger
        self.license_plate_database = []
        self.current_target_plate = ""
        self.recognized_plate = ""
        
    def process_license_plate(self, plate_text):
        """处理号牌识别结果"""
        try:
            self.logger.info(f"[号牌处理] 收到号牌识别结果: {plate_text}")
            self.recognized_plate = plate_text
            self._compare_license_plate()
        except Exception as e:
            self.logger.error(f"处理号牌识别结果失败: {e}")
            
    def _compare_license_plate(self):
        """号牌比对逻辑"""
        try:
            self.logger.info(f"[号牌比对] 开始比对号牌...")
            self.logger.info(f"[号牌比对] 识别到的号牌: {self.recognized_plate}")
            self.logger.info(f"[号牌比对] 目标号牌: {self.current_target_plate}")
            self.logger.info(f"[号牌比对] 号牌数据库: {self.license_plate_database}")
            
            # 检查是否与目标号牌匹配
            if self.current_target_plate and self.recognized_plate:
                if self.recognized_plate == self.current_target_plate:
                    self.logger.info(f"[号牌比对] 匹配成功！找到目标号牌: {self.recognized_plate}")
                    # 这里可以添加匹配成功后的动作
                else:
                    self.logger.info(f"[号牌比对] 不匹配，继续搜索...")
            
            # 检查是否在数据库中
            if self.recognized_plate in self.license_plate_database:
                self.logger.info(f"[号牌比对] 号牌在数据库中: {self.recognized_plate}")
            else:
                self.logger.info(f"[号牌比对] 号牌不在数据库中: {self.recognized_plate}")
                
        except Exception as e:
            self.logger.error(f"号牌比对处理失败: {e}")
            
    def set_target_license_plate(self, plate_number):
        """设置目标号牌"""
        self.current_target_plate = plate_number
        self.logger.info(f"[设置目标] 目标号牌设置为: {plate_number}")

    def add_license_plate_to_database(self, plate_number):
        """添加号牌到数据库"""
        if plate_number not in self.license_plate_database:
            self.license_plate_database.append(plate_number)
            self.logger.info(f"[数据库] 添加号牌到数据库: {plate_number}")
        else:
            self.logger.info(f"[数据库] 号牌已存在: {plate_number}")
            
    def remove_license_plate_from_database(self, plate_number):
        """从数据库中移除号牌"""
        if plate_number in self.license_plate_database:
            self.license_plate_database.remove(plate_number)
            self.logger.info(f"[数据库] 移除号牌: {plate_number}")
        else:
            self.logger.info(f"[数据库] 号牌不存在: {plate_number}")
            
    def clear_database(self):
        """清空数据库"""
        self.license_plate_database.clear()
        self.logger.info("[数据库] 数据库已清空")
        
    def get_database_size(self) -> int:
        """获取数据库大小"""
        return len(self.license_plate_database)
        
    def is_target_plate(self, plate_number: str) -> bool:
        """检查是否为目标号牌"""
        return plate_number == self.current_target_plate
        
    def is_in_database(self, plate_number: str) -> bool:
        """检查号牌是否在数据库中"""
        return plate_number in self.license_plate_database 