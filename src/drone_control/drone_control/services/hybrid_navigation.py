#!/usr/bin/env python3
"""
混合导航控制模块 - 支持任务模式和板外模式切换
"""

import asyncio
from drone_control.utils import SimpleMissionItem
from drone_control.utils import calculate_distance, observe_is_in_air
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw,VelocityBodyYawspeed
import numpy as np

class HybridNavigationController:
    """混合导航控制类 - 支持任务模式和板外模式切换"""

    def __init__(self, mavsdk_controller, logger, drone_state):
        self.mavsdk_controller = mavsdk_controller
        self.logger = logger
        self.drone_state = drone_state

    async def navigate_to_position(self):
        """混合导航：先任务模式，到达后切换板外模式"""
        # 阶段1：使用任务模式导航到目标位置
        await self._mission_navigation()
        # 阶段2：切换到板外模式进行精确控制
        await self._offboard_mode()
        self._switch_to_confirm_position()
        # await observe_is_in_air(self.drone, self.logger)


    async def _mission_navigation(self):
        """启动任务模式导航"""
        cur_lat = self.drone_state.current_position.latitude_deg
        cur_lon = self.drone_state.current_position.longitude_deg
        cur_alt = self.drone_state.current_position.absolute_altitude_m
        tar_lat = self.drone_state.target_position[0]
        tar_lon = self.drone_state.target_position[1]
        tar_alt = self.drone_state.target_position[2]
        self.logger.info(f"[任务模式] 起点: lat={cur_lat}, lon={cur_lon}, alt={cur_alt}")
        self.logger.info(f"[任务模式] 目标: lat={tar_lat}, lon={tar_lon}, alt={tar_alt}")
        # 创建任务项
        mission_items = []
        mission_items.append(
            SimpleMissionItem(
                cur_lat,   # 纬度
                cur_lon,   # 经度
                tar_alt,   # 高度
                1,         # 速度
            )
        )
        mission_items.append(
            SimpleMissionItem(
                tar_lat,   # 纬度
                tar_lon,   # 经度
                tar_alt,   # 高度
                1,         # 速度
            )
        )
        # 两个任务，一个是上升到目标高度，一个是到达目标地点，默认不返航
        await self.mavsdk_controller.upload_mission(mission_items,2)
        await self.mavsdk_controller.arm()
        self.logger.info("解锁成功，准备使用任务模式")

        # 启动任务
        await self.mavsdk_controller.start_mission()
        self.logger.info("[任务模式] 任务已启动")
        await self.mavsdk_controller.monitor_mission_progress()

    async def _offboard_mode(self):
        """开始板外模式并执行旋转"""
        # Set initial position to ensure a smooth transition to offboard
        await self.mavsdk_controller.start_offboard()
        self.logger.info("[Offboard] 板外模式启动成功")
        # Execute the rotation using the new, robust function
        target_yaw = self.drone_state.target_position[3]
        self.logger.info(f"[Offboard] 开始旋转至目标航向: {target_yaw:.2f} 度")
        await self.mavsdk_controller.rotate_to_yaw(target_yaw=target_yaw)
        self.logger.info("[Offboard] 旋转完成")
    
    def _switch_to_confirm_position(self):
        """切换到确认位置模式"""
        self.drone_state.reset_target_position()
        self.drone_state.update_confirm_start(True)
        self.drone_state.tag_detected=False
