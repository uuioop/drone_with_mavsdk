import asyncio
import math
import numpy as np
import time
from typing import Optional, Tuple, List
from mavsdk.action import ActionError
from mavsdk.telemetry import FlightMode ,LandedState
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw,
                            VelocityBodyYawspeed)
from mavsdk.mission import MissionItem , MissionPlan, MissionError
from drone_control.utils import SimpleMissionItem

class MavsdkController:
    """MAVSDK 控制器封装类"""

    def __init__(self,node):
        """
        初始化 MAVSDK 控制器
        
        Args:
            drone: MAVSDK System 实例
            logger: 日志记录器实例
        """
        self.drone = node.drone
        self.logger = node.get_logger()
        self.drone_state = node.drone_state
        self.status_monitor = node.status_monitor

    # --- 基础动作 ---
    async def _connect_to_drone(self, system_address) -> None:
        self.logger.info(f"尝试连接到无人机: {system_address}")
        await self.drone.connect(system_address=system_address)
        await self._get_home_position()

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.logger.info("成功连接到无人机!")
                self.drone_state.update_connection(True)
                return
        raise RuntimeError("连接超时")
    
    async def _get_home_position(self):
        """获取家位置"""
        try:
            async for home in self.drone.telemetry.home():
                self.drone_state.update_home_position(home)
                self.logger.info(f"家位置: {home}")
                break
            async for attitude_euler in self.drone.telemetry.attitude_euler():
                self.drone_state.takeoff_yaw_deg = attitude_euler.yaw_deg
                self.logger.info(f"偏航角: {attitude_euler.yaw_deg}")
                break
        except Exception as e:
            self.logger.warn(f"家位置监控异常: {e}")

    async def arm(self):
        """解锁无人机电机"""
        if self.drone_state.is_armed:
            self.logger.info("无人机电机已解锁")
            return True
        try:
            await self.drone.action.arm()
            self.drone_state.is_armed = True
            self.drone_state.home_position_ned = self.drone_state.current_position_ned
            return True
        except Exception as e:
            self.logger.error(f"解锁失败: {e}")
            raise

    async def disarm(self):
        """锁定无人机电机"""
        if not self.drone_state.is_armed:
            self.logger.info("无人机电机已锁定")
            return True
        try:
            await self.drone.action.disarm()
            self.drone_state.is_armed = False
            return True
        except Exception as e:
            self.logger.error(f"锁定电机失败: {e}")
            raise

    async def takeoff(self, altitude: float = 1.0, timeout: float = 10.0) -> bool:
        """
        起飞到指定高度
        - 在循环内处理高度值为None的情况
        - 确保所有高度比较都是安全的
        """
        try:
            self.logger.info(f"正在起飞到 {altitude} 米...")
            last_time = time.time()
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.takeoff()

            while time.time() - last_time < timeout:
                # 每次循环都获取一次飞行模式和高度
                current_flight_mode = await self.status_monitor.get_current_flight_mode()
                current_altitude = self.drone_state.get_current_altitude()

                # 检查模式是否已切换到 HOLD
                if current_flight_mode == FlightMode.HOLD:
                    self.logger.info("飞行模式切换为 HOLD。起飞成功")
                    return True

                # 安全地检查高度是否达标
                # 只有在 current_altitude 不是 None 的情况下，才进行比较
                if current_altitude is not None and current_altitude >= altitude * 0.9:
                    self.logger.info(f"到达目标高度 {altitude} 米。起飞成功")
                    return True
                
                # 如果两个成功条件都不满足，则等待下一次检查
                await asyncio.sleep(0.5)

            # 如果循环结束（因为超时），则抛出异常
            raise TimeoutError("起飞超时")

        except Exception as e:
            self.logger.error(f"起飞失败: {e}")
            raise

    async def land(self):
        """执行降落"""
        try:
            await self.drone.action.land()
            return True
        except Exception as e:
            self.logger.error(f"降落过程中出错: {e}")
            raise

    async def hold(self) -> bool:
        """悬停指定时间"""
        try:
            await self.drone.action.hold()
            return True
        except Exception as e:
            self.logger.error(f"悬停过程中出错: {e}")
            raise

    # --- Offboard 模式管理 ---
    async def start_offboard(self) -> bool:
        """启动 Offboard 模式"""
        if await self.status_monitor.get_current_flight_mode() == FlightMode.OFFBOARD:
            self.logger.info("已经在 Offboard 模式中")
            return True
        
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.2)
            await self.drone.offboard.start()

            for _ in range(5):
                await self.drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                )
                await asyncio.sleep(0.1) # 10Hz发送
            return True
        except OffboardError as e:
            self.logger.error(f"启动 Offboard 模式失败: {e}")
            raise

    async def stop_offboard(self) -> bool:
        """退出 Offboard 模式"""
        if not await self.status_monitor.get_current_flight_mode() == FlightMode.OFFBOARD:
            self.logger.info("当前不在 Offboard 模式中")
            return True
            
        try:
            await self.drone.offboard.stop()
            await asyncio.sleep(1)
            return True
        except OffboardError as e:
            self.logger.error(f"退出 Offboard 模式失败: {e}")
            raise   

    # --- 【关键】基于反馈的移动原语 ---
    async def goto_position_ned(self, north, east, down, yaw=0.0, tolerance=0.5):
        """
        飞行到指定的 NED 坐标点，直到误差小于 tolerance 才返回。
        """      
        self.logger.info(f"正在前往 NED: ({north}, {east}, {down})")
        # 内部循环，持续发送 set_position_ned 指令
        # 并检查 self.drone_state.current_position_ned 与目标的距离
        # 直到距离小于 tolerance 后退出循环
        max_attempts = 1000  # 防止无限循环
        attempt = 0
        try:
            while attempt < max_attempts:
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
                        await asyncio.sleep(0.5)
                        return True
                
                # 短暂等待后继续循环
                await asyncio.sleep(0.1)
                attempt += 1  
        except OffboardError as e:
            self.logger.error(f"设置位置失败: {e}")
            raise
        except Exception as e:
            self.logger.error(f"位置控制过程中出错: {e}")
            raise
        # 如果超过最大尝试次数仍未到达目标
        self.logger.warning(f"在 {max_attempts} 次尝试后仍未到达目标点")
        return False

    async def set_velocity_ned(self, vel_n: float, vel_e: float, vel_d: float, yaw_deg: float = 0.0) -> bool:
        """设置 NED 坐标系下的目标速度"""          
        try:
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vel_n, vel_e, vel_d, yaw_deg))
            return True
        except OffboardError as e:
            self.logger.error(f"设置速度失败: {e}")
            raise

    async def set_velocity_body(self, forward_m_s: float, right_m_s: float, down_m_s: float, yawspeed_deg_s: float) -> bool:
        """设置机体坐标系下的目标速度"""        
        try:
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(forward_m_s, right_m_s, down_m_s, yawspeed_deg_s))
            return True
        except OffboardError as e:
            self.logger.error(f"设置机体速度失败: {e}")
            raise
    
    # 转到目标航向角 body
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
            await self.set_velocity_body(
                0.0, 0.0, 0.0, applied_yaw_rate
            )
            
            await asyncio.sleep(0.05)  # 提高控制频率以获得更平滑的响应

        # 发送最终停止指令
        await self.set_velocity_body(
            0.0, 0.0, 0.0, 0.0
        )

    async def land_autodisarm(self):
        """自动降落并自动上锁"""
        await self.drone.action.land()
        async for landed_state in self.drone.telemetry.landed_state():
            if landed_state == LandedState.ON_GROUND:
                await self.drone.action.disarm()
       

    async def upload_mission(self, items: List[SimpleMissionItem],nums=1,is_return=False) -> bool:
        """上传任务"""
        try:
            mission_items=[]
            for i in range(nums):
                mission_items.append(
                    MissionItem(
                        items[i].latitude,
                        items[i].longitude,
                        items[i].altitude,
                        items[i].speed,
                        False if i==0 else True,  # 纬度, 经度, 高度, 速度, 接受
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
            mission_plan = MissionPlan(mission_items)
            await self.drone.mission.set_return_to_launch_after_mission(is_return)
            await self.drone.mission.upload_mission(mission_plan)
            self.logger.info("任务上传成功")
            return True
        except MissionError as e:
            self.logger.error(f"任务上传失败: {e}")
            raise

    async def start_mission(self) -> bool:
        """开始任务"""
        try:
            await self.drone.mission.start_mission()
            self.logger.info("任务已开始")
            return True
        except MissionError as e:
            self.logger.error(f"任务开始失败: {e}")
            raise

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
            raise

