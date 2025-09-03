#!/usr/bin/env python3
"""
混合导航控制模块 - 支持任务模式和板外模式切换
"""

import asyncio
from mavsdk.mission import MissionItem, MissionPlan
from drone_control.utils import calculate_distance, observe_is_in_air
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw,VelocityBodyYawspeed
import numpy as np

class HybridNavigationController:
    """混合导航控制类 - 支持任务模式和板外模式切换"""

    def __init__(self, drone, logger, drone_state):
        self.drone = drone
        self.logger = logger
        self.drone_state = drone_state

    async def navigate_to_position(self):
        """混合导航：先任务模式，到达后切换板外模式"""
        try:
            # 阶段1：使用任务模式导航到目标位置
            await self.start_mission_navigation()
            await self.wait_until_mission_finished()
            # 阶段2：切换到板外模式进行精确控制
            await self.start_offboard_mode()
            self.drone_state.reset_target_position()
            self.drone_state.update_confirm_start(True)
            self.drone_state.tag_detected=False

            # await observe_is_in_air(self.drone, self.logger)
        except Exception:
            raise

    async def start_mission_navigation(self):
        """启动任务模式导航"""
        try:
            cur_lat = self.drone_state.current_position.latitude_deg
            cur_lon = self.drone_state.current_position.longitude_deg
            cur_alt = self.drone_state.current_position.absolute_altitude_m
            tar_lat = self.drone_state.target_position[0]
            tar_lon = self.drone_state.target_position[1]
            tar_alt = self.drone_state.target_position[2]
            self.logger.info(
                f"[任务模式] 开始导航到: 纬度={tar_lat:.6f}, 经度={tar_lon:.6f}, 高度={tar_alt:.2f}m"
            )
           
            # 创建任务项
            mission_items = []
            mission_items.append(
                MissionItem(
                    cur_lat,
                    cur_lon,
                    tar_alt,
                    1,
                    False,  # 纬度, 经度, 高度, 速度, 接受
                    float("nan"),
                    float("nan"),  # 相机动作参数
                    MissionItem.CameraAction.NONE,
                    float("nan"),
                    float("nan"),
                    float("nan"),  # 相机参数
                    float("nan"),
                    float("nan"),  # 相机参数
                    MissionItem.VehicleAction.NONE,
                )
            )
            mission_items.append(
                MissionItem(
                    tar_lat,
                    tar_lon,
                    tar_alt,
                    1,
                    True,  # 纬度, 经度, 高度, 速度, 接受
                    float("nan"),
                    float("nan"),  # 相机动作参数
                    MissionItem.CameraAction.NONE,
                    float("nan"),
                    float("nan"),  # 相机参数
                    float("nan"),
                    float("nan"),
                    float("nan"),  # 相机参数
                    MissionItem.VehicleAction.NONE,
                )
            )
            mission_plan = MissionPlan(mission_items)
            # 设置任务完成后不自动返航
            await self.drone.mission.set_return_to_launch_after_mission(False)
            # 上传任务
            await self.drone.mission.upload_mission(mission_plan)
            self.logger.info("[任务模式] 任务已上传")

            await self.drone.action.arm()
            self.logger.info("解锁成功，准备使用任务模式")

            # 启动任务
            await self.drone.mission.start_mission()
            self.logger.info("[任务模式] 任务已启动")
        except Exception as e:
            self.logger.error(f"[任务模式] 启动失败: {e}")
            raise

    async def wait_until_mission_finished(self):
        """
        基于位置误差的任务完成检测
        当高度差和水平差都小于0.5m时结束循环
        """
        # origin_alt=self.drone_state.current_position.absolute_altitude_m
        # self.logger.info("[任务模式] 开始位置误差检测...")
        # # 位置误差阈值
        # position_tolerance = 2.5
        # altitude_tolerance = 1
        mission_started = False
        async for progress in self.drone.mission.mission_progress():
            if progress.current > 0:
                mission_started = True

            self.logger.info(f"Mission progress: {progress.current}/{progress.total}")

            # Wait until the mission has started and the current item is the last one
            if progress.current == progress.total and mission_started:
                self.logger.info("[任务模式] Mission complete!")
                break
        # while True:         
            

        #     # 获取当前位置
        #     current_position = self.drone_state.current_position
        #     current_lat = current_position.latitude_deg
        #     current_lon = current_position.longitude_deg
        #     current_alt = current_position.absolute_altitude_m

        #     target_position = self.drone_state.target_position
        #     target_lat = target_position[0]
        #     target_lon = target_position[1]
        #     target_alt = target_position[2] + origin_alt
        #     # 计算水平距离误差
        #     horizontal_error = calculate_distance(
        #         current_lat, current_lon, target_lat, target_lon
        #     )
        #     altitude_error = abs(current_alt - target_alt)
        #     # 检查是否达到目标精度
        #     if (
        #         horizontal_error <= position_tolerance
        #         and altitude_error <= altitude_tolerance
        #     ):
        #         self.logger.info(
        #             f"[任务模式] 位置精度达到要求！水平误差: {horizontal_error:.2f}m, 高度误差: {altitude_error:.2f}m"
        #         )
                
        #         self.logger.info("任务结束")             
        #         break
        #     else:
        #         self.logger.info(f"[任务模式] 位置精度未达到要求水平误差: {horizontal_error:.2f}m, 高度误差: {altitude_error:.2f}m")
        #         await asyncio.sleep(1)  # 延时1秒

    async def start_offboard_mode(self):
        """开始板外模式并执行旋转"""
        # Set initial position to ensure a smooth transition to offboard
        current_pos_ned = self.drone_state.current_position_ned
        current_yaw = self.drone_state.attitude_euler.yaw_deg
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(current_pos_ned.north_m, current_pos_ned.east_m,
                           current_pos_ned.down_m, current_yaw)
        )
        try:
            await self.drone.offboard.start()
            self.logger.info("[Offboard] 板外模式启动成功")
        except OffboardError as error:
            self.logger.error(f"板外模式启动失败: {error._result.result}")
            raise

        # Execute the rotation using the new, robust function
        target_yaw = self.drone_state.target_position[3]
        self.logger.info(f"[Offboard] 开始旋转至目标航向: {target_yaw:.2f} 度")
        await self.rotate_to_yaw(target_yaw=target_yaw)
        self.logger.info("[Offboard] 旋转完成")
        
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