#!/usr/bin/env python3
"""
板外模式导航控制模块 - 只使用板外模式进行导航
"""

import asyncio
from mavsdk import action
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityBodyYawspeed, VelocityNedYaw
from drone_control.utils.utils import observe_is_in_air
from drone_control.core.precision_land import PrecisionLand, PrecisionLandState


class OffboardNavigationController:
    """板外模式导航控制类 - 只使用板外模式进行导航"""

    def __init__(self, drone, logger, drone_state, license_plate_result):
        self.drone = drone
        self.logger = logger
        self.drone_state = drone_state
        self.license_plate_result = license_plate_result

        # 固定速度参数
        self.fixed_velocity = 1.0  # 固定飞行速度 (m/s)
        self.velocity_yaw_rate = 0.5  # 偏航角速度 (rad/s)


    async def navigate_to_position(self,target_north=None,target_east=None,target_down=None):
        """板外模式导航到目标位置 - 使用位置控制"""
        try:
            # 解锁无人机
            await self.drone.action.arm()
            self.logger.info("解锁成功，准备使用板外模式")

            # 启动板外模式
            await self.start_offboard_mode()

            # 分阶段导航
            if target_north is None or target_east is None or target_down is None:
                target_north, target_east, target_down = (
                    self.drone_state.calculate_target_ned_from_origin()
                )
            self.logger.info(
                f"target_north: {target_north}, target_east: {target_east}, target_down: {target_down}"
            )

            await self.fly_to_target_altitude(target_down)
            await self.fly_to_target_position(target_north, target_east,target_down)

            # 观察四周环境
            # await self.observe_environment()
            # await self.stop_offboard_mode()
            # await self.drone.action.hold()

        except Exception as e:
            self.logger.error(f"[板外导航] 导航失败: {e}")
            # 尝试降落
            try:
                await self.drone.action.land()
                await observe_is_in_air(self.drone, self.logger)
            except:
                pass
            raise

    async def start_offboard_mode(self):
        """启动板外模式"""
        try:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, 0.0, 0.0)
            )
            await self.drone.offboard.start()
            self.logger.info("[板外导航] 板外模式已启动")
        except Exception as e:
            self.logger.error(f"[板外导航] 启动板外模式失败: {e}")
            raise

    async def observe_environment(self):
        """观察四周环境"""
        self.logger.info("[板外导航] 开始观察四周环境")

        # 旋转180度观察
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 30.0))
        await asyncio.sleep(2)


        # 停止所有移动，确保完全静止
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(1)  # 等待1秒确保停止

        self.logger.info("[板外导航] 观察四周环境完成")

    async def fly_to_target_altitude(self, target_down):
        """飞到目标高度 - 使用位置控制"""
        self.logger.info(f"[板外导航] 阶段1：飞到目标高度 {target_down:.2f}m")
        
        # 获取当前水平位置
        current_north, current_east, _ = (
            self.drone_state.calculate_ned_from_origin()
        )

        # 使用位置控制飞到目标高度
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(current_north, current_east, target_down, 0.0)
        )
        # 等待到达目标高度
        # while True:
        #     altitude_diff = target_down - self.get_current_down()
        #     if abs(altitude_diff) <= 0.2:
        #         break
        #     await asyncio.sleep(0.1)  # 每0.5秒检查一次
        await asyncio.sleep(7)
        self.logger.info(f"[板外导航] 已到达目标高度 {target_down:.2f}m")

               
    async def fly_to_target_position(self, target_north, target_east,target_down):
        """飞到目标水平位置"""
        self.logger.info(
            f"[板外导航] 阶段2：飞到目标水平位置 北={target_north:.2f}m, 东={target_east:.2f}m"
        )

        # 使用位置控制飞到目标位置
    
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(target_north, target_east, target_down, 0.0)
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
        await asyncio.sleep(2)
        self.logger.info(f"[板外导航] 已到达目标位置 北={target_north:.2f}m, 东={target_east:.2f}m")

    def get_current_down(self):
        """获取当前下向位置"""
        _, _, down = self.drone_state.calculate_ned_from_origin()
        return down

    async def stop_offboard_mode(self):
        """停止板外模式"""
        try:
            await self.drone.offboard.stop()
            self.logger.info("[板外导航] 板外模式已停止")
        except Exception as e:
            self.logger.error(f"[板外导航] 停止板外模式失败: {e}")
    
