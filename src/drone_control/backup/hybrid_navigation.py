#!/usr/bin/env python3
"""
混合导航控制模块 - 支持任务模式和板外模式切换
"""

import asyncio
from mavsdk.mission import MissionItem, MissionPlan
from drone_control.utils import calculate_distance, observe_is_in_air,calculate_ned_from_origin
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

class HybridNavigationController:
    """混合导航控制类 - 支持任务模式和板外模式切换"""

    def __init__(self, drone, logger, drone_state,license_plate_result):
        self.drone = drone
        self.logger = logger
        self.navigation_phase = "IDLE"  # IDLE, MISSION, OFFBOARD
        self.drone_state = drone_state
        self.license_plate_result=license_plate_result

    async def navigate_to_position(self):
        """混合导航：先任务模式，到达后切换板外模式"""
        try:
            self.navigation_phase = "MISSION"
            # 阶段1：使用任务模式导航到目标位置
            await self.start_mission_navigation()
            await self.wait_until_mission_finished()
            # 阶段2：切换到板外模式进行精确控制
            await self.start_offboard_mode()
            # 阶段3：着陆后上锁
            await self.drone.action.land()
            await observe_is_in_air(self.drone, self.logger)    
        except Exception:
            raise

    async def start_mission_navigation(self):
        """启动任务模式导航"""
        try:
            cur_pos = self.drone_state.current_position
            cur_lat = cur_pos.latitude_deg
            cur_lon = cur_pos.longitude_deg
            tar_pos = self.drone_state.target_position
            tar_lat = tar_pos.latitude_deg
            tar_lon = tar_pos.longitude_deg
            tar_alt = tar_pos.absolute_altitude_m
            self.logger.info(
                "[任务模式] 开始导航到: 纬度={tar_lat:.6f}, 经度={tar_lon:.6f}, 高度={tar_alt:.2f}m"
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
                    2.0,
                    float("nan"),
                    float("nan"),  # 相机参数
                    MissionItem.VehicleAction.NONE,
                )
            )
            mission_plan = MissionPlan(mission_items)
            # 设置任务完成后不自动返航
            # await self.drone.action.set_home_position(
            #     cur_lat,
            #     cur_lon
            #     position.absolute_altitude_m
            # )
            await self.drone.mission.set_return_to_launch_after_mission(False)
            # 上传任务
            await self.drone.mission.upload_mission(mission_plan)
            self.logger.info("[任务模式] 任务已上传")

            await self.drone.action.arm()
            self.logger.info("解锁成功，准备使用任务模式")

            # 启动任务
            await self.drone.mission.start_mission()
            # self.mission_active = True

            self.logger.info("[任务模式] 任务已启动")

        except Exception as e:
            self.logger.error(f"[任务模式] 启动失败: {e}")
            raise

    async def wait_until_mission_finished(self):
        """
        基于位置误差的任务完成检测
        当高度差和水平差都小于0.5m时结束循环
        """
        self.logger.info("[任务模式] 开始位置误差检测...")
        # 位置误差阈值
        position_tolerance = 2 
        altitude_tolerance = 0.5

        while True:
            
            # 获取当前位置
            current_position = self.drone_state.current_position
            current_lat = current_position.latitude_deg
            current_lon = current_position.longitude_deg
            current_alt = current_position.absolute_altitude_m

            target_position = self.drone_state.target_position
            target_lat = target_position.latitude_deg
            target_lon = target_position.longitude_deg
            target_alt = target_position.absolute_altitude_m

            # 计算水平距离误差
            horizontal_error = calculate_distance(
                current_lat, current_lon, target_lat, target_lon
            )
            altitude_error = abs(current_alt - target_alt)
            mission_finished = await self.drone.mission.is_mission_finished()
            if mission_finished:
                self.logger.info("任务结束")  
                break
            
            # 检查是否达到目标精度
            # if (
            #     horizontal_error <= position_tolerance
            #     and altitude_error <= altitude_tolerance
            # ):
            #     self.logger.info(
            #         f"[任务模式] 位置精度达到要求！水平误差: {horizontal_error:.2f}m, 高度误差: {altitude_error:.2f}m"
            #     )
            #     self.drone_state.target_position = None
            #     await self.drone.action.hold()
            #     self.logger.info("任务结束")             
            #     break
            # else:
            #     self.logger.info(f"[任务模式] 位置精度未达到要求水平误差: {horizontal_error:.2f}m, 高度误差: {altitude_error:.2f}m")
            #     await asyncio.sleep(1)  # 延时1秒

    async def start_offboard_mode(self):
        """开始板外模式"""
        # 初始化板外模式位置
        
        home_pos = self.drone_state.home_position
        home_lat = home_pos.latitude_deg
        home_lon = home_pos.longitude_deg
        home_alt = home_pos.absolute_altitude_m
        cur_pos = self.drone_state.current_position
        cur_lat = cur_pos.latitude_deg
        cur_lon = cur_pos.longitude_deg
        cur_alt = cur_pos.absolute_altitude_m
        north,east,down = calculate_ned_from_origin(home_lat, home_lon, home_alt, cur_lat, cur_lon, cur_alt)
        await self.drone.offboard.set_position_ned(PositionNedYaw(north, east, down, 0))
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            self.logger.error(f"板外模式启动失败: {error._result.result}")
            self.logger.error("降落")
            await self.drone.action.land()  # 降落
            raise
        await asyncio.sleep(10)
        # 观察四周环境
        self.logger.info("[混合导航] 阶段3：观察四周环境")
        # await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 10.0))
        await self.drone.offboard.set_position_ned(PositionNedYaw(north, east, down, 180.0))
        self.logger.info("[混合导航] 观察四周环境完成")
        await asyncio.sleep(10)
        await self.drone.action.hold()
        await self.drone.offboard.stop()
        await asyncio.sleep(2)
        
                 