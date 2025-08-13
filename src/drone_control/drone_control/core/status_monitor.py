#!/usr/bin/env python3
"""
无人机状态监控模块
负责监控无人机连接状态、健康状态、飞行模式、位置和姿态
"""

import math
import asyncio
from std_msgs.msg import String
from drone_control.utils import calculate_distance

class DroneStatusMonitor:
    """无人机状态监控类"""
    
    def __init__(self, drone, drone_state, logger, position_publisher=None):
        self.drone = drone
        self.drone_state = drone_state
        self.logger = logger
        self.position_tolerance = 5.0
        self.altitude_tolerance = 2.0
        self.yaw_tolerance = 10.0
        self.position_publisher = position_publisher

    def set_position_publisher(self, publisher):
        """设置位置发布者"""
        self.position_publisher = publisher
        
    async def start_monitoring(self):
        """开始监控无人机状态"""
        self.logger.info("开始并行监控无人机状态...")
        await asyncio.gather(
            self._main_monitoring_loop(),
            self._mission_progress_loop()
        )

    async def _main_monitoring_loop(self):
        """主监控循环，高频更新"""
        while True:
            if self.drone_state.connected:
                try:
                    await self._monitor_flight_mode()
                    await self._monitor_position()
                    await self._monitor_velocity()
                except Exception as e:
                    self.logger.error(f"主状态监控异常: {e}")
            else:
                self.logger.debug("等待连接...", throttle_duration_sec=1)
            await asyncio.sleep(0.1)

    async def _mission_progress_loop(self):
        """任务进度监控循环，低频更新"""
        while True:
            if self.drone_state.connected:
                try:
                    if self.drone_state.navigating:
                        await self._monitor_mission_progress(
                            self.drone_state.current_position, self.drone_state.target_position
                        )
                except Exception as e:
                    self.logger.error(f"任务进度监控异常: {e}")
            await asyncio.sleep(1.0)
    
    async def _check_connection(self):
        """检查无人机连接状态"""
        try:
            # 尝试获取系统信息来检查连接
            async for state in self.drone.core.connection_state():
                if state is not None:
                    self.logger.info(f"连接状态: {state.is_connected}")
                    return state.is_connected  # 返回 True/False
                else:
                    self.logger.warn("连接状态返回None")
                    return False
        except Exception as e:
            # 如果是Core plugin未初始化错误，说明还未连接
            if "Core plugin has not been initialized" in str(e):
                self.logger.debug("Core plugin未初始化，等待连接...")
                return False
            else:
                self.logger.warn(f"连接异常：无法获取系统状态: {e}")
                return False
    
    async def _check_armed(self):
        """检查无人机是否解锁"""
        try:
            async for armed in self.drone.telemetry.armed():
                self.logger.info(f"解锁状态: {armed}")
                return armed  # 返回 True/False
        except Exception as e:
            self.logger.warn(f"解锁状态检查失败: {e}")
            return False

    async def _is_drone_armable(self):
        """检查无人机是否可以解锁"""
        try:
            async for health in self.drone.telemetry.health():
                if not health.is_armable:
                    self.logger.warn("解锁状态不满足")
                return health.is_armable  # 返回 True/False
        except Exception as e:
            self.logger.warn(f"可解锁状态检查失败: {e}")
            return False
            
    async def _check_preflight(self):
        if not self.drone_state.connected:
            self.logger.error("[任务模式] 未连接无人机")
            return False
        async for health in self.drone.telemetry.health():
            if not health.is_armable:
                self.logger.error("[任务模式] 系统不可解锁")
                return False
            if not health.is_global_position_ok:
                self.logger.error("[任务模式] GPS未锁定")
                return False
            if not health.is_home_position_ok:
                self.logger.error("[任务模式] 家位置未锁定")
                return False
            return True

    async def _monitor_mission_progress(self, current_position, target_position):
        """检查任务进度"""
        if not current_position or not target_position:
            return
            
        # 计算距离
        distance = calculate_distance(
            current_position.latitude_deg, current_position.longitude_deg,
            target_position.latitude_deg, target_position.longitude_deg
        )
        
        # 计算高度差
        altitude_diff = abs(current_position.absolute_altitude_m - target_position.absolute_altitude_m)
        
        if distance < self.position_tolerance and altitude_diff < self.altitude_tolerance:
            self.logger.info(f"到达目标位置！距离: {distance:.2f}m, 高度差: {altitude_diff:.2f}m")
        else:
            self.logger.info(f"距离目标: {distance:.2f}m, 高度差: {altitude_diff:.2f}m", throttle_duration_sec=3)
    
    # async def _monitor_health(self):
    #     """监控系统健康状态"""
    #     try:
    #         async for health in self.drone.telemetry.health():
    #             status = f"系统状态: 可解锁={health.is_armable} GPS锁定={health.is_global_position_ok}"
    #             self.logger.info(status, throttle_duration_sec=5)
    #             break
    #     except Exception as e:
    #         self.logger.warn(f"健康状态监控异常: {e}")
    
    async def _monitor_flight_mode(self):
        """监控飞行模式"""
        try:
            async for flight_mode in self.drone.telemetry.flight_mode():
                if flight_mode != self.drone_state.current_flight_mode:
                    old_mode = self.drone_state.current_flight_mode
                    self.drone_state.update_flight_mode(flight_mode)
                    self.logger.info(f"飞行模式切换: {old_mode} → {flight_mode}")
                break
        except Exception as e:
            self.logger.warn(f"飞行模式监控异常: {e}")
    
    async def _monitor_position(self):
        """监控GPS位置"""
        try:
            async for position in self.drone.telemetry.position():
                if position != self.drone_state.current_position:
                    self.drone_state.update_position(position)
                
                # 发布实时位置信息
                await self._publish_position_info(position)
                break
        except Exception as e:
            self.logger.warn(f"位置监控异常: {e}")
    
    async def _monitor_velocity(self):
        """监控速度"""
        try:
            async for velocity in self.drone.telemetry.velocity_ned():
                self.drone_state.update_velocity(velocity)
                break
        except Exception as e:
            self.logger.warn(f"速度监控异常: {e}")

    async def _monitor_attitude(self):
        """监控姿态"""
        try:
            async for attitude in self.drone.telemetry.attitude_euler():
                self.drone_state.update_attitude(attitude)
                break
        except Exception as e:
            self.logger.warn(f"姿态监控异常: {e}")
    
    async def _publish_position_info(self, position):
        """发布无人机位置信息"""
        if self.position_publisher:
            try:
                # 发布详细位置信息
                position_msg = String()
                position_msg.data = f"纬度: {position.latitude_deg:.6f}, 经度: {position.longitude_deg:.6f}, 绝对高度: {position.absolute_altitude_m:.2f}m, 相对高度: {position.relative_altitude_m:.2f}m"
                self.position_publisher.publish(position_msg)
                
            except Exception as e:
                self.logger.error(f"发布位置信息失败: {e}")


