#!/usr/bin/env python3
"""
板外模式导航控制模块 - 只使用板外模式进行导航
"""

import asyncio
import time
from mavsdk import action
from mavsdk.offboard import PositionNedYaw, VelocityBodyYawspeed, VelocityNedYaw,PositionGlobalYaw
from drone_control.utils.utils import observe_is_in_air
import numpy as np
class OffboardNavigationController:
    """板外模式导航控制类 - 只使用板外模式进行导航"""

    def __init__(self, drone, logger, drone_state):
        self.drone = drone
        self.logger = logger
        self.drone_state = drone_state
        self.target_pos=np.array([0, 0, 0],dtype=float)
        self.current_ned=None
        self.current_global=None
        self.yaw_deg=None
        # 固定速度参数
        self.fixed_velocity = 1.0  # 固定飞行速度 (m/s)
        self.velocity_yaw_rate = 0.5  # 偏航角速度 (rad/s)


    async def navigate_to_position(self):
        """板外模式导航到目标位置 - 使用位置控制"""
        self.target_pos=self.drone_state.target_position 
        self.current_ned=self.drone_state.current_position_ned
        self.current_global=self.drone_state.current_position
        self.yaw_deg=self.drone_state.attitude_euler.yaw_deg
        try:
            # 解锁无人机
            await self.drone.action.arm()
            self.logger.info("解锁成功，准备使用板外模式")

            # 启动板外模式
            await self.start_offboard_mode()

            # 分阶段导航
            await self.fly_to_target_altitude()
            await self.fly_to_target_position()
            
            # 观察四周环境
            await self.rotate_to_yaw(self.target_pos[3])
            # await self.stop_offboard_mode()
            # await self.drone.action.hold()
            # await self.drone.action.land()
            # await observe_is_in_air(self.drone, self.logger)
            self.drone_state.reset_target_position()
            self.drone_state.update_confirm_start(True)
            self.drone_state.tag_detected=False


        except Exception as e:
            self.logger.error(f"[板外导航] 导航失败: {e}")
            raise
    
    async def fly_to_target_altitude(self):
        """飞到目标高度 - 使用位置控制"""
        self.logger.info(f"[板外导航] 阶段1：飞到目标高度 {self.target_pos[2]:.2f}m")
        
        # 使用位置控制飞到目标高度
        if self.drone_state.navigation_mode=='RELATIVE':
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(self.current_ned.north_m, self.current_ned.east_m, self.target_pos[2], self.yaw_deg)
            )
        else:
            await self.drone.offboard.set_position_global(
                    PositionGlobalYaw(self.current_global.latitude_deg, self.current_global.longitude_deg, self.target_pos[2], self.yaw_deg,PositionGlobalYaw.AltitudeType.AGL)
                )
        
        # 等待到达目标高度
        # while True:
        #     altitude_diff = target_down - self.get_current_down()
        #     if abs(altitude_diff) <= 0.2:
        #         break
        #     await asyncio.sleep(0.1)  # 每0.5秒检查一次
        
        await asyncio.sleep(9)
        self.logger.info(f"[板外导航] 已到达目标高度 {self.target_pos[2]:.2f}m")

               
    async def fly_to_target_position(self):
        """飞到目标水平位置"""
        self.logger.info(
            f"[板外导航] 阶段2：飞到目标水平位置 北={self.target_pos[0]:.2f}m, 东={self.target_pos[1]:.2f}m"
        )

        # 使用位置控制飞到目标位置
    
        if self.drone_state.navigation_mode=='RELATIVE':
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(self.target_pos[0], self.target_pos[1], self.target_pos[2], self.yaw_deg)
            )
        else:
            await self.drone.offboard.set_position_global(
                PositionGlobalYaw(self.target_pos[0], self.target_pos[1], self.target_pos[2], self.yaw_deg,PositionGlobalYaw.AltitudeType.AGL)
            )
        
        # 等待到达目标位置
        # while True:
        #     # 获取当前水平位置
        #     current_north, current_east, _ = (
        #         self.drone_state.calculate_ned_from_origin()
        #     )

        #     # 计算水平距离
        #     distance_n = target_north - current_north
        #     distance_e = target_east - current_east
        #     horizontal_distance = (distance_n**2 + distance_e**2) ** 0.5

        #     if horizontal_distance <= 0.4:
        #         break
        #     await asyncio.sleep(0.5)  # 每0.5秒检查一次
        await asyncio.sleep(4)
        self.logger.info(f"[板外导航] 已到达目标位置 北={self.target_pos[0]:.2f}m, 东={self.target_pos[1]:.2f}m")

    async def start_offboard_mode(self):
        """启动板外模式"""
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, 0.0, self.yaw_deg)
        )
        try:
            await self.drone.offboard.start()
            self.logger.info("[板外导航] 板外模式已启动")
        except Exception as e:
            self.logger.error(f"[板外导航] 启动板外模式失败: {e}")
            raise

    async def rotate_to_yaw(self, target_yaw, tolerance_deg=2.0):
        """
        使用比例控制平滑地旋转至目标航向，解决摇头问题。
        Args:
            target_yaw (float): 目标航向角 (度).
            tolerance_deg (float): 到达目标的容忍误差 (度).
        """
        # --- 可调参数 ---
        # P-增益：这是最重要的参数。它决定了响应速度。
        # 太高会导致震荡，太低会导致响应缓慢。
        kp = 0.8

        # 最大/最小速度：限制旋转速度，防止过快或在误差很小时无法移动。
        max_yaw_rate_deg_s = 25.0  # 最大旋转速度 (度/秒)
        min_yaw_rate_deg_s = 2.0   # 最小旋转速度，克服静摩擦力

        while True:
            current_yaw = self.drone_state.attitude_euler.yaw_deg
            
            # 计算最短路径的角度差 (-180 to 180)
            yaw_diff = (target_yaw - current_yaw + 180) % 360 - 180
            
            # 如果在容差范围内，则完成旋转
            if abs(yaw_diff) <= tolerance_deg:
                self.logger.info(f"目标航向到达. 当前: {current_yaw:.2f}, 误差: {yaw_diff:.2f} 度")
                break

            # --- 比例控制核心 ---
            # 期望的旋转速度与误差成正比
            desired_yaw_rate = kp * yaw_diff
            
            # 限制旋转速度，确保其在最大和最小范围内
            # 使用 np.sign 来保持旋转方向
            if abs(desired_yaw_rate) > max_yaw_rate_deg_s:
                applied_yaw_rate = np.sign(desired_yaw_rate) * max_yaw_rate_deg_s
            elif abs(desired_yaw_rate) < min_yaw_rate_deg_s:
                applied_yaw_rate = np.sign(desired_yaw_rate) * min_yaw_rate_deg_s
            else:
                applied_yaw_rate = desired_yaw_rate

            # 发送指令
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, applied_yaw_rate)
            )
            
            await asyncio.sleep(0.05)  # 提高控制频率以获得更平滑的响应

        # 发送最终停止指令
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )

    async def stop_offboard_mode(self):
        """停止板外模式"""
        try:
            await self.drone.offboard.stop()
            self.logger.info("[板外导航] 板外模式已停止")
        except Exception as e:
            self.logger.error(f"[板外导航] 停止板外模式失败: {e}")
    
