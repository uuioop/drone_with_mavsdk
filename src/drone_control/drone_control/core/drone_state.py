#!/usr/bin/env python3
"""
无人机状态管理模块
"""

from drone_control.utils.utils import calculate_ned_from_origin

class DroneState:
    """无人机状态管理类"""
    def __init__(self):
        self.connected = False
        self.navigating = False
        self.current_flight_mode = "UNKNOWN"
        self.current_position = None
        self.target_position = None
        self.home_position = None
        self.current_attitude = None
        self.target_attitude = None
        self.current_velocity = None
        
    
    def update_connection(self,connected):
        if not self.connected == connected:
            self.connected = connected

    def update_flight_mode(self, mode: str):
        """更新飞行模式"""
        self.current_flight_mode = mode
        
    def update_position(self, position):
        """更新当前位置"""
        self.current_position = position
        
    def update_target_position(self, position):
        """更新目标位置"""
        self.target_position = position
        
    def update_home_position(self, position):
        """更新起始位置"""
        self.home_position = position
        
    def update_attitude(self, attitude):
        """更新姿态信息"""
        self.current_attitude = attitude
        
    def update_target_attitude(self, attitude):
        """更新目标姿态"""
        self.target_attitude = attitude

    def update_velocity(self, velocity):
        """更新速度信息"""
        self.current_velocity = velocity
    
    def update_navigating(self, navigating):
        """更新导航状态"""
        self.navigating = navigating
        
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
    
    def calculate_ned_from_origin(self) -> tuple:
        """计算当前位置相对于起始位置的NED坐标"""

        # 计算当前位置相对于起始位置的NED坐标
        current_lat, current_lon, current_alt = self.current_position.latitude_deg, self.current_position.longitude_deg, self.current_position.absolute_altitude_m
        home_lat, home_lon, home_alt = self.home_position.latitude_deg, self.home_position.longitude_deg, self.home_position.absolute_altitude_m

        # 计算当前位置相对于起始位置的NED坐标
        return calculate_ned_from_origin(home_lat, home_lon, home_alt, current_lat, current_lon, current_alt)

    def calculate_target_ned_from_origin(self) -> tuple:
        """计算目标位置相对于起始位置的NED坐标"""
        target_lat, target_lon, target_alt = self.target_position.latitude_deg, self.target_position.longitude_deg, self.target_position.absolute_altitude_m
        home_lat, home_lon, home_alt = self.home_position.latitude_deg, self.home_position.longitude_deg, self.home_position.absolute_altitude_m

        # 计算目标位置相对于起始位置的NED坐标
        return calculate_ned_from_origin(home_lat, home_lon, home_alt, target_lat, target_lon, target_alt)


