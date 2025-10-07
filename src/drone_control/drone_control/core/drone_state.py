#!/usr/bin/env python3
"""
无人机状态管理模块
"""
import numpy as np

class DroneState:
    """无人机状态管理类"""
    def __init__(self):
        # 连接状态
        self.connected = False
        self.is_armed = False
        self.landed = False # 降落状态
        self.search_started = False # 精准降落
        self.confirm_started=False # 确认地址
        self.current_flight_mode = "UNKNOWN" # 当前飞行模式
        self.current_position = None # 当前GPS位置
        self.target_position = np.array([0, 0, 0, 0],dtype=float)
        self.home_position = None
        self.current_position_ned = None # NED坐标系下的当前位置
        self.home_position_ned = None # 解锁点的NED位置
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
        """ArUco标记检测状态"""
        return self._tag_detected

    @tag_detected.setter
    def tag_detected(self,is_detected: bool):
        if not self._tag_detected == is_detected:
            self._tag_detected = is_detected

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

    def get_current_altitude(self) -> float:
        """获取当前高度（基于NED坐标系）"""
        if self.current_position_ned is None or self.home_position_ned is None:
            return None
        return abs(self.home_position_ned.down_m - self.current_position_ned.down_m )


