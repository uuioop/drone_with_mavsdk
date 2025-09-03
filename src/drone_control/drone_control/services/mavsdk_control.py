import asyncio
import logging
import math
from typing import Optional, Tuple, List
from mavsdk.action import ActionError
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw,
                            VelocityBodyYawspeed)
from mavsdk.mission import MissionItem, MissionPlan, MissionError

class MavsdkController:
    """MAVSDK 控制器封装类"""

    def __init__(self,drone, logger, drone_state):
        """
        初始化 MAVSDK 控制器
        
        Args:
            drone: MAVSDK System 实例
            logger: 日志记录器实例
        """
        self.drone = drone
        self.logger = logger
        self.drone_state = drone_state

    # --- 基础动作 ---
    async def _connect_to_drone(self,system_address):
        """连接到无人机""" 
        try:      
            self.logger.info(f"尝试连接到无人机: {system_address}")
            await self.drone.connect(system_address=system_address)
            # 获取初始位置
            await self._get_home_position()
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.logger.info("成功连接到无人机!")
                    self.drone_state.update_connection(True)
                    break
        except ActionError as e:
            self.logger.error(f"连接到无人机失败: {e}")
            return False
    
    async def _get_home_position(self):
        """获取家位置"""
        try:
            async for home in self.drone.telemetry.home():
                self.drone_state.update_home_position(home)
                self.logger.info(f"家位置: {home}")
                break
        except Exception as e:
            self.logger.warn(f"家位置监控异常: {e}")

    async def arm(self):
        """解锁无人机电机"""
        try:
            await self.drone.action.arm()
            return True
        except Exception as e:
            self.logger.error(f"解锁失败: {e}")
            return False

    async def disarm(self):
        """锁定无人机电机"""
        try:
            await self.drone.action.disarm()
            return True
        except Exception as e:
            self.logger.error(f"锁定电机失败: {e}")
            return False

    async def takeoff(self, altitude: float = 2.5) -> bool:
        """起飞到指定高度"""
        try:
            self.logger.info(f"正在起飞到 {altitude} 米...")
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.takeoff()
            
            async for position in self.drone.telemetry.position():
                if position.relative_altitude_m >= altitude * 0.95:
                    self.logger.info(f"已达到目标高度: {position.relative_altitude_m:.1f}米")
                    return True
                await asyncio.sleep(0.1)
        except Exception as e:
            self.logger.error(f"起飞失败: {e}")
            return False

    async def land(self):
        """执行降落"""
        try:
            await self.drone.action.land()
            return True
        except Exception as e:
            self.logger.error(f"降落过程中出错: {e}")
            return False

    async def hold(self,duration: float = 5.0) -> bool:
        """悬停指定时间"""
        try:
            await self.drone.action.hold(duration)
            return True
        except Exception as e:
            self.logger.error(f"悬停过程中出错: {e}")
            return False

    # --- Offboard 模式管理 ---
    async def start_offboard(self) -> bool:
        """启动 Offboard 模式"""
        if self.drone_state.current_flight_mode == "OFFBOARD":
            self.logger.info("已经在 Offboard 模式中")
            return True
            
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            self.logger.info("已进入 Offboard 模式")
            return True
        except OffboardError as e:
            self.logger.error(f"启动 Offboard 模式失败: {e}")
            return False

    async def stop_offboard(self) -> bool:
        """退出 Offboard 模式"""
        if not self.drone_state.current_flight_mode == "OFFBOARD":
            self.logger.info("当前不在 Offboard 模式中")
            return True
            
        try:
            await self.drone.offboard.stop()
            return True
        except OffboardError as e:
            self.logger.error(f"退出 Offboard 模式失败: {e}")
            return False

    # --- 【关键】基于反馈的移动原语 ---
    async def goto_position_ned(self, north, east, down, yaw=0.0, tolerance=0.5):
        """
        飞行到指定的 NED 坐标点，直到误差小于 tolerance 才返回。
        这会取代所有 asyncio.sleep() 的等待。
        """
        self.logger.info(f"正在前往 NED: ({north}, {east}, {down})")
        # 确保在 Offboard 模式中
        if not self.drone_state.current_flight_mode == "OFFBOARD":
            self.logger.error("未在 Offboard 模式中，无法执行位置控制")
            return False
        
        # 内部循环，持续发送 set_position_ned 指令
        # 并检查 self.drone_state.current_position_ned 与目标的距离
        # 直到距离小于 tolerance 后退出循环
        max_attempts = 1000  # 防止无限循环
        attempt = 0
        
        while attempt < max_attempts:
            try:
                # 发送位置指令
                await self.drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))
                
                # 检查当前位置与目标的距离
                if self.drone_state.current_position_ned is not None:
                    current_n = self.drone_state.current_position_ned.north_m
                    current_e = self.drone_state.current_position_ned.east_m
                    current_d = self.drone_state.current_position_ned.down_m
                    
                    # 计算3D距离
                    distance = math.sqrt(
                        (north - current_n) ** 2 + 
                        (east - current_e) ** 2 + 
                        (down - current_d) ** 2
                    )
                    
                    self.logger.debug(f"当前位置: ({current_n:.2f}, {current_e:.2f}, {current_d:.2f}), "
                                    f"目标位置: ({north:.2f}, {east:.2f}, {down:.2f}), "
                                    f"距离: {distance:.2f}m, 容差: {tolerance}m")
                    
                    # 如果距离小于容差，则认为已到达目标点
                    if distance <= tolerance:
                        self.logger.info(f"已到达目标点，最终距离: {distance:.2f}m")
                        return True
                
                # 短暂等待后继续循环
                await asyncio.sleep(0.1)
                attempt += 1
                
            except OffboardError as e:
                self.logger.error(f"设置位置失败: {e}")
                return False
            except Exception as e:
                self.logger.error(f"位置控制过程中出错: {e}")
                return False
        
        # 如果超过最大尝试次数仍未到达目标
        self.logger.warning(f"在 {max_attempts} 次尝试后仍未到达目标点")
        return False

    async def set_velocity_ned(self, vel_n: float, vel_e: float, vel_d: float, yaw_deg: float = 0.0) -> bool:
        """设置 NED 坐标系下的目标速度"""
        if not self.drone_state.current_flight_mode == "OFFBOARD":
            self.logger.error("未在 Offboard 模式中，无法设置速度")
            return False
            
        try:
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vel_n, vel_e, vel_d, yaw_deg))
            return True
        except OffboardError as e:
            self.logger.error(f"设置速度失败: {e}")
            return False

    async def set_velocity_body(self, forward_m_s: float, right_m_s: float, down_m_s: float, yawspeed_deg_s: float) -> bool:
        """设置机体坐标系下的目标速度"""
        if not self.drone_state.current_flight_mode == "OFFBOARD":
            self.logger.error("未在 Offboard 模式中，无法设置机体速度")
            return False
        
        try:
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(forward_m_s, right_m_s, down_m_s, yawspeed_deg_s))
            return True
        except OffboardError as e:
            self.logger.error(f"设置机体速度失败: {e}")
            return False

    async def upload_mission(self, mission_items: List[MissionItem]) -> bool:
        """上传任务"""
        try:
            mission_plan = MissionPlan(mission_items)
            await self.drone.mission.upload_mission(mission_plan)
            self.logger.info("任务上传成功")
            return True
        except MissionError as e:
            self.logger.error(f"任务上传失败: {e}")
            return False

    async def start_mission(self) -> bool:
        """开始任务"""
        try:
            await self.drone.mission.start_mission()
            self.logger.info("任务已开始")
            return True
        except MissionError as e:
            self.logger.error(f"任务开始失败: {e}")
            return False

    async def monitor_mission_progress(self):
        """检查任务进度"""
        try:
            mission_started = False
            async for progress in self.drone.mission.mission_progress():
                if progress.current > 0:
                    mission_started = True

                self.logger.info(f"Mission progress: {progress.current}/{progress.total}")

                # Wait until the mission has started and the current item is the last one
                if progress.current == progress.total and mission_started:
                    self.logger.info("[任务模式] Mission complete!")
                    return True
        except MissionError as e:
            self.logger.error(f"任务进度检查失败: {e}")
            return False

# async def example_usage():
#     """使用示例"""
#     logging.basicConfig(level=logging.INFO)
#     logger = logging.getLogger("MavsdkController")
    
#     drone = System()
#     await drone.connect(system_address="udp://:14540")
    
#     controller = MavsdkController(drone, logger)
    
#     if not await controller.check_connection():
#         return

#     if not await controller.pre_flight_checks():
#         return

#     if await controller.arm():
#         if await controller.takeoff(5.0):
#             await controller.hold(5.0)

#             # Offboard 模式示例
#             if await controller.start_offboard():
#                 logger.info("向前移动 5 米...")
#                 await controller.set_velocity_body(5.0, 0.0, 0.0, 0.0)
#                 await asyncio.sleep(2)
                
#                 logger.info("停止...")
#                 await controller.set_velocity_body(0.0, 0.0, 0.0, 0.0)
#                 await asyncio.sleep(2)
                
#                 await controller.stop_offboard()

#             # 任务模式示例
#             mission_items = [
#                 MissionItem(47.3980398, 8.5455725, 25, 10, True, float('nan'), float('nan'),
#                             MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan')),
#                 MissionItem(47.3980362, 8.5450146, 25, 10, True, float('nan'), float('nan'),
#                             MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'))
#             ]
            
#             if await controller.upload_mission(mission_items):
#                 if await controller.start_mission():
#                     async for mission_progress in drone.mission.mission_progress():
#                         logger.info(f"任务进度: {mission_progress.current}/{mission_progress.total}")
#                         if mission_progress.current == mission_progress.total:
#                             logger.info("任务完成")
#                             break
            
#             await controller.land()
#             await controller.disarm()

# if __name__ == "__main__":
#     asyncio.run(example_usage())
