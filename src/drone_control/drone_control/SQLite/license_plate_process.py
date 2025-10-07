#!/usr/bin/env python3
"""
号牌处理模块 (修改版)
"""

# 假设 modified_license_plate_database.py 与此文件在同一目录下
from drone_control.SQLite import LicensePlateDatabase
from enum import Enum, auto
import time # 确保导入了 time 模块
import numpy as np # 确保导入了 numpy

class PlateMatchResult(Enum):
    """号牌比对结果的枚举信号"""
    MATCH = auto()              # 完全匹配
    APPROACH = auto()           # 前缀匹配，仅后两位不同，需要接近调整
    SEARCH_AGAIN = auto()       # 完全不匹配，需要重新搜索

class LicensePlateProcessor:
    """号牌处理类"""
    def __init__(self, logger, config: dict = {}):
        self.logger = logger
        # 初始化号牌数据库
        self.config = config
        self.license_plate_database = LicensePlateDatabase()
        self.current_target_plate = ""
        # 储存最近识别到的号牌
        self.recognized_plate = ""
        # 储存最近稳定识别到的号牌
        self.recognized_plate_stable = ""
        # 识别结果稳定性计数器
        self.stability_counter = 0
        self.required_stability_count = 3
        # 存放被拒绝号牌的集合
        self.rejected_plates = set()
        # 中心点坐标
        self.center_x = None
        self.center_y = None
        self.timestamp = time.time()
        self.image_frame = self.config.get('image_frame', np.array([640,480]))
        self.detection_frame = self.config.get('detection_frame', np.array([64,48]))


    def is_valid(self):
        """检查当前数据是否有效"""
        return self.center_x is not None and self.center_y is not None \
            and self.recognized_plate_stable != ""
    
    def _check_timeout(self) -> bool:
        """
        检查是否超时
        :return: 如果超时则返回 True，否则返回 False
        """
        return time.time() - self.timestamp > self.config.get("target_timeout", 1.0)


    def update_recognized_plate(self, plate_text: str, center_x: float, center_y: float):
        """更新识别到的号牌数据"""
        # 识别结果稳定再更新其他数据
        if self.is_plate_recognition_stable(plate_text):
            self.center_x = center_x
            self.center_y = center_y
            self.timestamp = time.time()
    
    def is_plate_recognition_stable(self,plate_text: str):
        """检查当前门牌识别是否稳定
        
        考虑到运动模糊和振动问题，只有当识别结果连续多次一致时，
        才认为识别结果是可靠的，避免因临时的错误识别而进入确认状态
        """
        # 过滤无效识别结果（如长度异常的字符串）
        if len(plate_text) <8 or len(plate_text) >11:
            return False
        # 检查后八位是否为数字
        if not (plate_text[-8:].isdigit()):
            return False
        # 根据识别结果更新稳定性计数器
        if plate_text == self.recognized_plate:
            self.stability_counter += 1
        else:
            self.recognized_plate = plate_text
            self.stability_counter = 1
        # 检查是否达到稳定阈值
        if self.stability_counter >= self.required_stability_count:
            self.recognized_plate_stable = self.recognized_plate
            return True
        else:
            return False
    
    def calculate_movement_from_plate_diff(self):
        """
        根据识别到的号牌后三位与目标号牌后三位的差异计算机体坐标系方向指令
        机体坐标系FRD：
        - Y轴：指向右侧（正值）- 由后两位控制（同一层不同位置）
        - Z轴：指向底部（正值表示向下，负值表示向上）- 由倒数第三位控制（不同层）
        
        Returns:
            tuple: (axis, direction) - 移动轴('y'或'z')和方向(1.0或-1.0)
                  如果没有有效的方向指令，返回(None, 0.0)
        """
        # 确保目标号牌和当前识别的号牌有八位
        if len(self.current_target_plate) < 8 or len(self.recognized_plate_stable) < 8:
            return (None, 0.0)
        
        # 检查后八位是否为数字
        if not (self.current_target_plate[-8:].isdigit() and self.recognized_plate_stable[-8:].isdigit()):
            return (None, 0.0)
        
        # 提取倒数第三位数字（控制Z轴，不同层）
        target_third_last = int(self.current_target_plate[-3])
        detected_third_last = int(self.recognized_plate_stable[-3])
        
        # 提取后两位数字（控制Y轴，同一层不同位置）
        target_last_two = int(self.current_target_plate[-2:])
        detected_last_two = int(self.recognized_plate_stable[-2:])
        
        # 计算层差异（Z轴）
        layer_diff = target_third_last - detected_third_last
        
        # 优先处理Z轴方向（垂直方向，不同层）
        if layer_diff != 0:
            # 目标层大于当前层时向上飞（负值），目标层小于当前层时向下飞（正值）
            direction = -1.0 if layer_diff > 0 else 1.0
            return ('z', direction)
        
        # Z轴无差异时，处理Y轴方向（水平方向，同一层不同位置）
        position_diff = target_last_two - detected_last_two
        
        if position_diff != 0:
            # 目标位置大于当前位置时向右飞（正值），目标位置小于当前位置时向左飞（负值）
            direction = 1.0 if position_diff > 0 else -1.0
            return ('y', direction)
        
        # 无差异时返回None
        return (None, 0.0)

    def compare_plate(self) -> bool:
        """
        对比当前识别到的号牌与目标号牌
        """
        return self.recognized_plate_stable == self.current_target_plate
        
    def set_target_license_plate(self, plate_number: str):
        """设置目标号牌"""
        self.current_target_plate = plate_number
        self.logger.info(f"[设置目标] 目标号牌设置为: {plate_number}")

    def add_license_plate_to_database(self, plate_number: str, latitude: float, longitude: float, altitude: float):
        """
        添加号牌到数据库
        如果号牌已存在，则会更新其位置信息
        """
        if self.license_plate_database.is_license_plate_exists(plate_number):
             self.logger.info(f"[数据库] 号牌 '{plate_number}' 已存在，将更新其位置信息。")
        else:
             self.logger.info(f"[数据库] 添加新号牌到数据库: {plate_number}")
             
        self.license_plate_database.add_license_plate(plate_number, latitude, longitude, altitude)

    def is_aligned(self) -> bool:
        """
        检查是否与目标号牌对齐
        Returns:
            如果中心位置在检测框内，则返回True
        """
        # 第一个为左边界，第二个为右边界
        is_x_aligned = abs(self.center_x) >= (self.image_frame[0]-self.detection_frame[0])/2 \
            and abs(self.center_x) <= (self.image_frame[0]+self.detection_frame[0])/2 + self.detection_frame[0]
        # 第一个为上边界，第二个为下边界
        is_y_aligned = abs(self.center_y) >= (self.image_frame[1]-self.detection_frame[1])/2 \
            and abs(self.center_y) <= (self.image_frame[1]+self.detection_frame[1])/2 + self.detection_frame[1]
        return is_x_aligned and is_y_aligned
    
    def calculate_velocity(self, axes =['y','z']) -> list:
        """
        计算基于机体坐标系的速度指令，使目标号牌中心点在检测框边界内
        前提是机头正对号牌，容忍一些角度误差
        机体坐标系：
        - Y轴：指向右侧
        - Z轴：指向底部
        像素坐标系：
        - X轴：指向右侧
        - Y轴：指向底部
        Args:
            axes (list, optional): 计算速度的轴. Defaults to ['y','z'].
        Returns:
            根据axes计算的速度指令列表，顺序为['y','z']
        """
        velocity_vector = []
        # 第一个为左边界，第二个为右边界
        is_x_aligned = abs(self.center_x) >= (self.image_frame[0]-self.detection_frame[0])/2 \
            and abs(self.center_x) <= (self.image_frame[0]+self.detection_frame[0])/2 + self.detection_frame[0]
        # 第一个为上边界，第二个为下边界
        is_y_aligned = abs(self.center_y) >= (self.image_frame[1]-self.detection_frame[1])/2 \
            and abs(self.center_y) <= (self.image_frame[1]+self.detection_frame[1])/2 + self.detection_frame[1]
        if 'y' in axes:
            if is_x_aligned:
                velocity_vector.append(0.0)
            else:
                if abs(self.center_x)-(self.image_frame[0]-self.detection_frame[0])/2 < 0:
                    velocity_vector.append(0.1)
                else:
                    velocity_vector.append(-0.1)
        if 'z' in axes:
            if is_y_aligned:
                velocity_vector.append(0.0)
            else:
                if abs(self.center_y)-(self.image_frame[1]-self.detection_frame[1])/2 < 0:
                    velocity_vector.append(0.1)
                else:
                    velocity_vector.append(-0.1)
        
        return velocity_vector
    
    def get_stable_license_plate(self) -> str:
        """
        获取当前稳定的号牌
        Returns:
            当前稳定的号牌，如果没有稳定的号牌则返回空字符串
        """
        return self.recognized_plate_stable
    
    def clear_rejected_plate(self):
        """
        清除已拒绝的号牌
        """
        self.rejected_plates.clear()
