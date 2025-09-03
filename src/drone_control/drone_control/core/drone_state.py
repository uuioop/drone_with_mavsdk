#!/usr/bin/env python3
"""
无人机状态管理模块
"""

from drone_control.utils.utils import calculate_ned_from_origin
import numpy as np

class DroneState:
    """无人机状态管理类"""
    def __init__(self):
        self.connected = False
        self._landed = False
        # 精准降落
        self._search_started = False
        # 确认地址
        self.confirm_started=False
        # 检测到标记
        self._tag_detected=False
        self.current_flight_mode = "UNKNOWN"
        self.current_position = None
        self.target_position = np.array([0, 0, 0, 0],dtype=float)
        self.navigation_mode = "UNKNOWN"
        self.home_position = None
        # NED坐标系下的当前位置
        self.current_position_ned = None
        self.current_velocity_ned = None

        # 四元数姿态
        self.attitude_quaternion = None
        # 欧拉角姿态
        self.attitude_euler = None
    
    def update_connection(self,connected):
        if not self.connected == connected:
            self.connected = connected

    @property
    def tag_detected(self):
        return self._tag_detected

    @tag_detected.setter
    def tag_detected(self,is_detected: bool):
        if not self._tag_detected == is_detected:
            self._tag_detected = is_detected

    @property
    def landed(self):
        return self._landed

    @landed.setter
    def landed(self,is_landed: bool):
        if not self._landed == is_landed:
            self._landed = is_landed

    @property
    def search_started(self):
        return self._search_started

    @search_started.setter
    def search_started(self,is_search_started: bool):
        if not self._search_started == is_search_started:
            self._search_started = is_search_started

    def update_confirm_start(self,confirm_started: bool):
        if not self.confirm_started == confirm_started:
            self.confirm_started = confirm_started

    def update_flight_mode(self, mode: str):
        """更新飞行模式"""
        self.current_flight_mode = mode 
        
    def update_position(self, position):
        """更新当前位置"""
        self.current_position = position
        
    def update_target_position(self, x,y,z,w):
        """更新目标位置"""
        self.target_position = np.array([x,y,z,w], dtype=float)

    def reset_target_position(self):
        """重置目标位置"""
        self.target_position = np.array([0,0,0,0], dtype=float)
    def update_home_position(self, position):
        """更新起始位置"""
        self.home_position = position
        
    def is_ready_for_flight(self) -> bool:
        """检查是否准备好飞行"""
        return self.connected and self.armed
        
    def get_status_summary(self) -> dict:
        """获取状态摘要"""
        return {
            'flight_mode': self.current_flight_mode,
            'has_position': self.current_position is not None,
            'has_target': self.target_position is not None,
            'has_home': self.home_position is not None
        } 
    
    # def calculate_ned_from_origin(self) -> tuple:
    #     """计算当前位置相对于起始位置的NED坐标"""

    #     # 计算当前位置相对于起始位置的NED坐标
    #     current_lat, current_lon, current_alt = self.current_position.latitude_deg, self.current_position.longitude_deg, self.current_position.absolute_altitude_m
    #     home_lat, home_lon, home_alt = self.home_position.latitude_deg, self.home_position.longitude_deg, self.home_position.absolute_altitude_m

    #     # 计算当前位置相对于起始位置的NED坐标
    #     return calculate_ned_from_origin(home_lat, home_lon, home_alt, current_lat, current_lon, current_alt)

    # def calculate_target_ned_from_origin(self) -> tuple:
    #     """计算目标位置相对于起始位置的NED坐标"""
    #     target_lat, target_lon, target_alt = self.target_position[0], self.target_position[1], self.target_position[2]
    #     home_lat, home_lon, home_alt = self.home_position.latitude_deg, self.home_position.longitude_deg, self.home_position.absolute_altitude_m

    #     # 计算目标位置相对于起始位置的NED坐标
    #     return calculate_ned_from_origin(home_lat, home_lon, home_alt, target_lat, target_lon, target_alt)

    def get_current_time(self) -> float:
        """获取当前时间戳（秒）"""
        import time
        return time.time()


