#!/usr/bin/env python3
"""
无人机状态管理模块
"""

class DroneState:
    """无人机状态管理类"""
    def __init__(self):
        self.connected = False
        self.current_flight_mode = "UNKNOWN"
        self.current_position = None
        self.target_position = None
        self.home_position = None
        self.current_attitude = None
        self.target_attitude = None
    
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