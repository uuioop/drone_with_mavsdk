#!/usr/bin/env python3
"""
板外模式导航控制模块 - 只使用板外模式进行导航
"""

import asyncio
import numpy as np
class OffboardNavigationController:
    """板外导航控制类 - 仅使用板外模式进行导航"""

    def __init__(self, node):
        # 通过依赖注入获取所需组件
        self.mavsdk_controller = node.mavsdk_controller
        self.logger = node.get_logger()
        self.drone_state = node.drone_state
        self.confirm_license_controller = node.confirm_license_controller
        self.target_pos=np.array([0, 0, 0],dtype=float)
        self.current_ned=None
        self.current_global=None
        self.yaw_deg=None
        # 固定速度参数
        self.fixed_velocity = 1.0  # 固定飞行速度 (m/s)
        self.velocity_yaw_rate = 0.5  # 偏航角速度 (rad/s)


    async def navigate_to_position(self):
        """板外模式导航到目标位置 - 分阶段精确导航"""
        self.target_pos=self.drone_state.target_position 
        self.current_ned=self.drone_state.current_position_ned
        self.current_global=self.drone_state.current_position
        self.yaw_deg=self.drone_state.attitude_euler.yaw_deg
        try:
            # 解锁无人机
            await self.mavsdk_controller.arm()
            self.logger.info("解锁成功，准备使用板外模式")

            # 启动板外模式
            await self.mavsdk_controller.start_offboard()

            # 阶段1：垂直导航到目标高度
            await self.fly_to_target_altitude()
            # 阶段2：水平导航到目标位置
            await self.fly_to_target_position()
            # 阶段3：旋转到目标偏航角
            await self.mavsdk_controller.rotate_to_yaw(self.target_pos[3])
            # 阶段4：导航完成，切换到精准降落模式
            await self._exit()

        except Exception as e:
            self.logger.error(f"[板外导航] 导航失败: {e}")
            raise
    
    async def fly_to_target_altitude(self):
        """飞到目标高度 - 使用位置控制"""
        self.logger.info(f"[板外导航] 阶段1：飞到目标高度 {self.target_pos[2]:.2f}m")
        
        # 使用位置控制飞到目标高度
        await self.mavsdk_controller.goto_position_ned(
        self.current_ned.north_m,
        self.current_ned.east_m,
        self.target_pos[2],  # 目标高度
        self.yaw_deg
        )
        await asyncio.sleep(1)
        self.logger.info(f"[板外导航] 已到达目标高度 {self.target_pos[2]:.2f}m")

               
    async def fly_to_target_position(self):
        """飞到目标水平位置"""
        self.logger.info(
            f"[板外导航] 阶段2：飞到目标水平位置 北={self.target_pos[0]:.2f}m, 东={self.target_pos[1]:.2f}m"
        )

        # 使用位置控制飞到目标位置
        await self.mavsdk_controller.goto_position_ned(
            self.target_pos[0], 
            self.target_pos[1],
            self.target_pos[2], 
            self.yaw_deg
        )

        await asyncio.sleep(1)
        self.logger.info(f"[板外导航] 已到达目标位置 北={self.target_pos[0]:.2f}m, 东={self.target_pos[1]:.2f}m")
    
    async def _exit(self):
        self.drone_state.reset_target_position()
        self.drone_state.update_confirm_start(True)
        await self.confirm_license_controller.start()