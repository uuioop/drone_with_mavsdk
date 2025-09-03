#!/usr/bin/env python3
"""
姿态控制模块
"""

class AttitudeController:
    """姿态控制类"""
    def __init__(self, drone, logger):
        self.drone = drone
        self.logger = logger
        self.attitude_control_active = False
        self.yaw_tolerance = 10.0  # 偏航角容差（度）
        
    async def check_attitude_control(self, current_attitude, target_attitude):
        """检查姿态控制"""
        if not current_attitude or not target_attitude:
            return
            
        # 计算偏航角差
        yaw_diff = abs(current_attitude.yaw_deg - target_attitude.yaw_deg)
        
        if yaw_diff < self.yaw_tolerance:
            self.logger.info(f"[姿态控制] 达到目标姿态！偏航角差: {yaw_diff:.2f}°")
            self.attitude_control_active = False
        else:
            self.logger.info(f"[姿态控制] 偏航角差: {yaw_diff:.2f}°", throttle_duration_sec=2)
            
    def set_yaw_tolerance(self, tolerance: float):
        """设置偏航角容差"""
        self.yaw_tolerance = tolerance
        
    def start_attitude_control(self):
        """开始姿态控制"""
        self.attitude_control_active = True
        self.logger.info("[姿态控制] 开始姿态控制")
        
    def stop_attitude_control(self):
        """停止姿态控制"""
        self.attitude_control_active = False
        self.logger.info("[姿态控制] 停止姿态控制")
        
    def is_controlling_attitude(self) -> bool:
        """检查是否正在控制姿态"""
        return self.attitude_control_active 