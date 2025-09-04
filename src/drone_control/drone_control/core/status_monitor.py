#!/usr/bin/env python3
"""
无人机状态监控模块
负责监控无人机连接状态、健康状态、飞行模式、位置和姿态
"""

import math
import asyncio
from std_msgs.msg import String
from drone_control.utils import calculate_distance
from mavsdk.telemetry import LandedState,PositionVelocityNed

class DroneStatusMonitor:
    """无人机状态监控类"""
    
    def __init__(self, drone, drone_state, logger):
        self.drone = drone
        self.drone_state = drone_state
        self.logger = logger
        self.position_tolerance = 5.0
        self.altitude_tolerance = 2.0
        self.yaw_tolerance = 10.0
        self.publisher = None

    def set_info_publisher(self, publisher):
        """设置发布者"""
        self.publisher = publisher
        
    async def start_monitoring(self):
        """开始监控无人机状态"""
        self.logger.info("开始监控无人机状态...")
        # 创建长期运行的监控任务
        tasks = [
            self._monitor_flight_mode(),
            self._monitor_global_position(),
            self._monitor_ned_position(),
            self._monitor_attitude_quaternion(),
            self._monitor_attitude_euler(),
            self._monitor_velocity(),
            self.observer_is_landed()
        ]
        await asyncio.gather(*tasks)
    
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
            self.logger.error("未连接无人机")
            return False
        async for health in self.drone.telemetry.health():
            if not health.is_armable:
                self.logger.error("系统不可解锁")
                return False
            if not health.is_global_position_ok:
                self.logger.error("GPS未锁定")
                return False
            if not health.is_home_position_ok:
                self.logger.error("家位置未锁定")
                return False
            return True


    async def _monitor_flight_mode(self):
        """监控飞行模式"""
        try:
            async for flight_mode in self.drone.telemetry.flight_mode():
                if flight_mode != self.drone_state.current_flight_mode:
                    old_mode = self.drone_state.current_flight_mode
                    self.drone_state.update_flight_mode(flight_mode)
                    self.logger.info(f"飞行模式切换: {old_mode} → {flight_mode}")
        except Exception as e:
            self.logger.warn(f"飞行模式监控异常: {e}")
    
    async def _monitor_global_position(self):
        """监控GPS位置"""
        try:
            # 监控全局位置
            async for position in self.drone.telemetry.position():
                if position != self.drone_state.current_position:
                    self.drone_state.update_position(position)
                    await self._publish_position_info(self.drone_state.current_position)                
        except Exception as e:
            self.logger.warn(f"global position监控异常: {e}")
    async def _monitor_ned_position(self):
        """监控NED位置"""
        try:        
            # 监控NED位置
            async for pv_ned in self.drone.telemetry.position_velocity_ned():
                # 更新NED位置
                if pv_ned.position != self.drone_state.current_position_ned:
                    self.drone_state.current_position_ned=pv_ned.position
                    await self._publish_position_ned_info(self.drone_state.current_position_ned)
                
        except Exception as e:
            self.logger.warn(f"ned position监控异常: {e}")
    
    async def _monitor_velocity(self):
        """监控NED速度"""
        try:
            async for pv_ned in self.drone.telemetry.position_velocity_ned():
                if pv_ned.velocity != self.drone_state.current_velocity_ned:
                    self.drone_state.current_velocity_ned=pv_ned.velocity
        except Exception as e:
            self.logger.warn(f"速度监控异常: {e}")

    async def _monitor_attitude_quaternion(self):
        """监控四元数姿态"""
        try:
            async for attitude_quaternion in self.drone.telemetry.attitude_quaternion():
                if attitude_quaternion != self.drone_state.attitude_quaternion:
                    self.drone_state.attitude_quaternion=attitude_quaternion
        except Exception as e:
            self.logger.warn(f"姿态监控异常: {e}")

    async def _monitor_attitude_euler(self):
        """监控欧拉角姿态"""
        try:
            async for attitude_euler in self.drone.telemetry.attitude_euler():
                if attitude_euler != self.drone_state.attitude_euler:
                    self.drone_state.attitude_euler=attitude_euler
        except Exception as e:
            self.logger.warn(f"姿态监控异常: {e}")
    
    async def observer_is_landed(self):
        """ 
        监控无人机是否着陆
        
        Args:
            drone: 无人机对象
            logger: 日志记录器
        """ 
        async for landed_state in self.drone.telemetry.landed_state():
            if landed_state == LandedState.ON_GROUND:
                if not self.drone_state.landed:
                    self.drone_state.landed = True
            else:
                self.drone_state.landed = False

    # async def observe_is_in_air(drone, logger):
    #     """ 
    #     监控无人机是否在空中
    #     Args:
    #         drone: 无人机对象
    #         logger: 日志记录器
    #     """
    #     was_in_air = False
    #     async for is_in_air in drone.telemetry.in_air():
    #         if is_in_air:
    #             was_in_air = is_in_air
    #             if not was_in_air:  # 刚起飞
    #                 logger.info("无人机已起飞")

    #         if was_in_air and not is_in_air:
    #             logger.info("无人机已降落，准备上锁")              
    #             # 等待一段时间确保无人机完全稳定
    #             await asyncio.sleep(2)    
    #             return

    async def _publish_position_info(self, position):
        """发布无人机信息"""
        if self.publisher:
            try:
                # 发布详细位置信息
                position_msg = String()
                position_msg.data = f"纬度: {position.latitude_deg:.6f}, 经度: {position.longitude_deg:.6f}, 绝对高度: {position.absolute_altitude_m:.2f}m, 相对高度: {position.relative_altitude_m:.2f}m"
                self.publisher.publish(position_msg)     
            except Exception as e:
                self.logger.error(f"发布无人机信息失败: {e}")

    async def _publish_position_ned_info(self,pisition_ned):
        """发布无人机信息"""
        if self.publisher:
            try:
                # 发布详细位置信息
                position_msg = String()
                position_msg.data = f"北={pisition_ned.north_m:.2f}m, 东={pisition_ned.east_m:.2f}m, 下={pisition_ned.down_m:.2f}m"
                self.publisher.publish(position_msg)                
            except Exception as e:
                self.logger.error(f"发布无人机信息失败: {e}")

    async def _publish_yaw_info(self, yaw_deg):
        """发布无人机信息"""
        if self.publisher:
            try:
                # 发布详细位置信息
                position_msg = String()
                position_msg.data = f"偏航角={yaw_deg:.2f}°"
                self.publisher.publish(position_msg)
                
            except Exception as e:
                self.logger.error(f"发布无人机信息失败: {e}")