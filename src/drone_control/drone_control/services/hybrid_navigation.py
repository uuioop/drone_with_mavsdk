#!/usr/bin/env python3
"""
混合导航控制模块 - 支持任务模式和板外模式切换
"""

from drone_control.utils import SimpleMissionItem
import numpy as np

class HybridNavigationController:
    """混合导航控制类 - 支持任务模式和板外模式切换"""

    def __init__(self, node):
        # 通过依赖注入获取所需组件
        self.mavsdk_controller = node.mavsdk_controller
        self.logger = node.get_logger()
        self.drone_state = node.drone_state
        self.confirm_license_controller = node.confirm_license_controller

    async def navigate_to_position(self):
        """混合导航：先任务模式，到达后切换板外模式"""
        # 阶段1：使用任务模式导航到目标位置（GPS坐标导航）
        await self._mission_navigation()
        # 阶段2：切换到板外模式进行精确控制（NED坐标系精对准）
        await self._offboard_mode()
        # 阶段3：导航完成，切换到精准降落模式
        await self._exit()
        
    async def _mission_navigation(self):
        """启动任务模式导航 - GPS坐标系长距离导航"""
        # 获取当前GPS位置
        cur_lat = self.drone_state.current_position.latitude_deg
        cur_lon = self.drone_state.current_position.longitude_deg
        cur_alt = self.drone_state.current_position.absolute_altitude_m
        
        # 获取目标GPS位置（从target_position数组中提取）
        tar_lat = self.drone_state.target_position[0]  # 目标纬度
        tar_lon = self.drone_state.target_position[1]  # 目标经度
        tar_alt = self.drone_state.target_position[2]  # 目标高度
        
        self.logger.info(f"[任务模式] 起点: lat={cur_lat}, lon={cur_lon}, alt={cur_alt}")
        self.logger.info(f"[任务模式] 目标: lat={tar_lat}, lon={tar_lon}, alt={tar_alt}")
        
        # 创建任务项 - 先上升到目标高度，再水平移动到目标位置
        mission_items = []
        mission_items.append(
            SimpleMissionItem(
                cur_lat,   # 纬度 - 当前位置
                cur_lon,   # 经度 - 当前位置
                tar_alt,   # 高度 - 目标高度
                1,         # 速度 - 1m/s
            )
        )
        mission_items.append(
            SimpleMissionItem(
                tar_lat,   # 纬度 - 目标位置
                tar_lon,   # 经度 - 目标位置
                tar_alt,   # 高度 - 保持目标高度
                1,         # 速度 - 1m/s
            )
        )
        
        # 上传任务到飞控（2个航点，不自动返航）
        await self.mavsdk_controller.upload_mission(mission_items, 2)
        await self.mavsdk_controller.arm()
        self.logger.info("解锁成功，准备使用任务模式")

        # 启动任务并监控执行进度
        await self.mavsdk_controller.start_mission()
        self.logger.info("[任务模式] 任务已启动")
        await self.mavsdk_controller.monitor_mission_progress()

    async def _offboard_mode(self):
        """切换到板外模式 - 使用NED坐标系进行精确控制"""
        self.logger.info("[板外模式] 准备切换")
        # 解锁并启动板外模式
        await self.mavsdk_controller.arm()
        await self.mavsdk_controller.start_offboard()
        self.logger.info("[板外模式] 已启动，开始精确控制")
        # 执行旋转至目标航向
        target_yaw = self.drone_state.target_position[3]
        self.logger.info(f"[Offboard] 开始旋转至目标航向: {target_yaw:.2f} 度")
        await self.mavsdk_controller.rotate_to_yaw(target_yaw=target_yaw)
        self.logger.info("[Offboard] 旋转完成")
    
    async def _exit(self):
        """退出导航，切换到精准降落模式 - 完成混合导航流程"""
        self.logger.info("[退出] 导航完成，切换到精准降落模式")
        # 启动精准降落控制器，开始视觉引导降落
        self.drone_state.reset_target_position()
        self.drone_state.update_confirm_start(True)
        await self.confirm_license_controller.start()
