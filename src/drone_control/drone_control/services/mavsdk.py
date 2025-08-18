#!/usr/bin/env python3
"""
MAVSDK 常用功能封装
提供无人机控制常用功能的封装，包括起飞、降落、悬停、Offboard 控制等
"""

import asyncio
from typing import Optional, Tuple
import numpy as np
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw,
                            PositionGlobalYaw, VelocityBodyYawspeed)
from mavsdk.telemetry import LandedState


class MavsdkController:
    """MAVSDK 控制器封装类"""

    def __init__(self, drone, logger):
        """
        初始化 MAVSDK 控制器
        
        Args:
            drone: MAVSDK System 实例
            logger: 日志记录器实例
        """
        self.drone = drone
        self.logger = logger 
        self._is_offboard = False

    async def arm(self, force: bool = False) -> bool:
        """解锁无人机电机
        
        Args:
            force: 是否强制解锁（忽略安全检查）
            
        Returns:
            bool: 是否成功解锁
        """
        try:
            self.logger.info("正在解锁电机...")
            await self.drone.action.arm()
            self.logger.info("电机已解锁")
            return True
        except Exception as e:
            self.logger.error(f"解锁失败: {e}")
            if force:
                try:
                    await self.drone.action.arm_force()
                    self.logger.warning("已强制解锁电机")
                    return True
                except Exception as e_force:
                    self.logger.error(f"强制解锁失败: {e_force}")
            return False

    async def disarm(self) -> bool:
        """锁定无人机电机
        
        Returns:
            bool: 是否成功锁定
        """
        try:
            self.logger.info("正在锁定电机...")
            await self.drone.action.disarm()
            self.logger.info("电机已锁定")
            return True
        except Exception as e:
            self.logger.error(f"锁定电机失败: {e}")
            return False

    async def takeoff(self, altitude: float = 2.5) -> bool:
        """起飞到指定高度
        
        Args:
            altitude: 目标高度（米）
            
        Returns:
            bool: 是否成功执行起飞
        """
        try:
            self.logger.info(f"正在起飞到 {altitude} 米...")
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.takeoff()
            
            # 等待达到目标高度
            async for position in self.drone.telemetry.position():
                altitude_relative = -position.relative_altitude_m
                if altitude_relative >= altitude * 0.9:  # 达到目标高度的90%认为成功
                    self.logger.info(f"已达到目标高度: {altitude_relative:.1f}米")
                    return True
                await asyncio.sleep(0.1)
                
        except Exception as e:
            self.logger.error(f"起飞失败: {e}")
            return False

    async def land(self) -> bool:
        """执行降落
        
        Returns:
            bool: 是否成功执行降落指令
        """
        try:
            self.logger.info("开始降落...")
            await self.drone.action.land()
            
            # 等待降落完成
            async for state in self.drone.telemetry.landed_state():
                if state == LandedState.ON_GROUND:
                    self.logger.info("已安全降落")
                    return True
                await asyncio.sleep(0.1)
                
        except Exception as e:
            self.logger.error(f"降落失败: {e}")
            return False

    async def disarm(self) -> bool:
        """锁定无人机电机
        
        Returns:
            bool: 是否成功锁定
        """
        try:
            self.logger.info("正在锁定电机...")
            await self.drone.action.disarm()
            self.logger.info("电机已锁定")
            return True
        except Exception as e:
            self.logger.error(f"锁定电机失败: {e}")
            return False


    async def hold(self, duration: float = 5.0) -> bool:
        """悬停指定时间
        
        Args:
            duration: 悬停时间（秒）
            
        Returns:
            bool: 是否成功执行悬停
        """
        try:
            self.logger.info(f"开始悬停 {duration} 秒...")
            await asyncio.sleep(duration)
            return True
        except Exception as e:
            self.logger.error(f"悬停过程中出错: {e}")
            return False

    async def start_offboard(self) -> bool:
        """启动 Offboard 模式
        
        Returns:
            bool: 是否成功进入 Offboard 模式
        """
        if self._is_offboard:
            self.logger.info("已经在 Offboard 模式中")
            return True
            
        try:
            self.logger.info("正在启动 Offboard 模式...")
            await self.drone.offboard.start()
            self._is_offboard = True
            self.logger.info("已进入 Offboard 模式")
            return True
        except OffboardError as e:
            self.logger.error(f"启动 Offboard 模式失败: {e}")
            return False

    async def stop_offboard(self) -> bool:
        """退出 Offboard 模式
        
        Returns:
            bool: 是否成功退出 Offboard 模式
        """
        if not self._is_offboard:
            self.logger.info("当前不在 Offboard 模式中")
            return True
            
        try:
            self.logger.info("正在退出 Offboard 模式...")
            await self.drone.offboard.stop()
            self._is_offboard = False
            self.logger.info("已退出 Offboard 模式")
            return True
        except OffboardError as e:
            self.logger.error(f"退出 Offboard 模式失败: {e}")
            return False

    async def set_position_ned(
        self, 
        north: float, 
        east: float, 
        down: float, 
        yaw: float = 0.0
    ) -> bool:
        """设置 NED 坐标系下的目标位置
        
        Args:
            north: 北向位置（米）
            east: 东向位置（米）
            down: 地向位置（米，正值表示向下）
            yaw: 偏航角（弧度）
            
        Returns:
            bool: 是否成功设置位置
        """
        if not self._is_offboard:
            self.logger.error("未在 Offboard 模式中，无法设置位置")
            return False
            
        try:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(north, east, down, yaw))
            return True
        except OffboardError as e:
            self.logger.error(f"设置位置失败: {e}")
            return False

    async def set_velocity_ned(
        self, 
        velocity_north: float, 
        velocity_east: float, 
        velocity_down: float, 
        yaw: float = 0.0
    ) -> bool:
        """设置 NED 坐标系下的目标速度
        
        Args:
            velocity_north: 北向速度（米/秒）
            velocity_east: 东向速度（米/秒）
            velocity_down: 地向速度（米/秒，正表示向下）
            yaw: 偏航角（弧度）
            
        Returns:
            bool: 是否成功设置速度
        """
        if not self._is_offboard:
            self.logger.error("未在 Offboard 模式中，无法设置速度")
            return False
            
        try:
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(velocity_north, velocity_east, velocity_down, yaw))
            return True
        except OffboardError as e:
            self.logger.error(f"设置速度失败: {e}")
            return False

    async def get_position(self) -> Optional[Tuple[float, float, float]]:
        """获取当前位置 (NED 坐标系)
        
        Returns:
            Optional[Tuple[float, float, float]]: (north, east, down) 位置，如果获取失败返回 None
        """
        try:
            position = await self.drone.telemetry.position().__anext__()
            return (position.north_m, position.east_m, -position.altitude_amsl_m)
        except Exception as e:
            self.logger.error(f"获取位置失败: {e}")
            return None

    async def get_attitude(self) -> Optional[Tuple[float, float, float, float]]:
        """获取当前姿态四元数
        
        Returns:
            Optional[Tuple[float, float, float, float]]: (w, x, y, z) 四元数，如果获取失败返回 None
        """
        try:
            quaternion = await self.drone.telemetry.attitude_quaternion().__anext__()
            return (quaternion.w, quaternion.x, quaternion.y, quaternion.z)
        except Exception as e:
            self.logger.error(f"获取姿态失败: {e}")
            return None

    async def is_armed(self) -> bool:
        """检查电机是否已解锁
        
        Returns:
            bool: 电机是否已解锁
        """
        try:
            return await self.drone.telemetry.armed()
        except Exception as e:
            self.logger.error(f"获取电机状态失败: {e}")
            return False

    async def is_in_air(self) -> bool:
        """检查无人机是否在空中
        
        Returns:
            bool: 是否在空中
        """
        try:
            return await self.drone.telemetry.in_air()
        except Exception as e:
            self.logger.error(f"获取飞行状态失败: {e}")
            return False


# 使用示例
async def example_usage():
    """使用示例"""
    # 创建无人机连接
    drone = System()
    await drone.connect("udp://:14540")
    
    # 等待连接
    print("等待无人机连接...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("无人机已连接!")
            break
    
    # 创建控制器
    controller = MavsdkController(drone)
    
    # 解锁并起飞
    if await controller.arm():
        if await controller.takeoff(5.0):
            # 悬停5秒
            await controller.hold(5.0)
            
            # 进入Offboard模式
            if await controller.start_offboard():
                # 向前移动5米
                current_pos = await controller.get_position()
                if current_pos:
                    target_north = current_pos[0] + 5.0
                    await controller.set_position_ned(target_north, current_pos[1], current_pos[2])
                    await asyncio.sleep(5.0)  # 等待到达目标
                
                # 退出Offboard模式
                await controller.stop_offboard()
            
            # 降落
            await controller.land()
            
            # 锁定电机
            await controller.disarm()


if __name__ == "__main__":
    asyncio.run(example_usage())
