#!/usr/bin/env python3
"""
精准降落模块 - 基于ArUco标记的精准降落控制 (已優化版本)
"""

import time
import asyncio
from enum import Enum
from typing import Tuple
import numpy as np
from mavsdk.offboard import VelocityBodyYawspeed
from scipy.spatial.transform import Rotation as R

class PrecisionLandState(Enum):
    """精准降落状态枚举"""
    IDLE = "IDLE"
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    DESCEND = "DESCEND"
    FINISHED = "FINISHED"

class ArucoTag:
    """存储ArUco标记信息，包含位置、姿态和时间戳。"""
    def __init__(self, position=None, orientation=None, timestamp=0):
        self.position = np.array(position, dtype=float) if position is not None else None
        # 使用scipy的Rotation对象处理姿态，更强大和稳定
        self.orientation = orientation if orientation is not None else None
        self.timestamp = timestamp

    def is_valid(self) -> bool:
        """检查标记数据是否有效。"""
        return self.timestamp > 0 and self.position is not None and self.orientation is not None

class PrecisionLand:
    """
    精准降落控制类，负责状态管理和PID控制指令计算。
    完全基于机体坐标系 (Body Frame)，不依赖GPS/NED坐标系。
    """
    
    def __init__(self, drone, logger, drone_state, tf_buffer, tf_listener):
        self.drone = drone
        self.logger = logger
        self.drone_state = drone_state
        self.tf_buffer = tf_buffer
        self.tf_listener = tf_listener
        
        # 状态管理
        self.state = PrecisionLandState.IDLE
        self._tag = ArucoTag()

        self._target_lost_prev = False

        # --- 控制参数 (重要！需要根据实际飞机进行调试) ---
        self.params = {
            # --- 速度與閾值 ---
            'descent_vel': 0.6,       # 下降速度 (m/s)，建議值0.2-0.5，越小越穩定
            'max_vel_xy': 2.0,        # 最大水平飛行速度 (m/s)，限制过大的速度指令
            'target_timeout': 4.0,    # 目标丢失超时时间 (s)
            'delta_position': 0.2,    # 水平位置到達閾值 (m)，越小越精準但可能更難達到
            
        }

        self.logger.info("[PL] 精准降落控制器已初始化")

    async def start_precision_land(self):
        """外部调用此函数以开始精准降落流程。"""
        if self.drone.offboard.is_active():
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
        self.drone_state.search_started = True
        self._switch_to_state(PrecisionLandState.SEARCH) # 从搜索开始更安全
        self.logger.info("[PL] 开始精准降落流程，进入搜索模式")

    def process_tag_detection(self, position_cam, orientation_cam_quat):
        """
        处理来自图像节点的ArUco Tag检测结果。
        输入为相机坐标系下的位置和姿态。
        """
        # 相机到机体的转换矩阵
        # 假设相机朝下安装，且相机坐标系为：X右, Y下, Z前
        # 相机坐标 → 机体系 FRD
        R_cb = np.array([
             [0, -1, 0], # 机体X轴 = 相机-Y轴
             [1, 0, 0], # 机体Y轴 = 相机X轴 
             [0, 0, 1]  # 机体Z轴 = 相机Z轴 
        ])

        # t_cb = np.array([0, 0, -0.10])  # 相机在机下 0.1 m
        # 1. 位置转换：将相机坐标系下的位置向量左乘旋转矩阵，得到机体坐标系下的位置
        tag_body_position = R_cb @ np.array(position_cam, dtype=float)
        
        # 2. 姿态转换：将相机姿态和安装姿态结合
        R_orientation_in_cam = R.from_quat(orientation_cam_quat)
        R_body_to_cam = R.from_matrix(R_cb.T) # 注意是 body to cam
        tag_body_orientation = R_body_to_cam * R_orientation_in_cam

        # 3. 更新目标信息
        self._tag = ArucoTag(position=tag_body_position,
                            orientation=tag_body_orientation.as_matrix(),
                            timestamp=time.time())

    async def update(self):
        """主更新函数，由外部节点定期调用以驱动状态机。"""
        if not self.drone_state.search_started:
            return
        target_lost = self._check_target_timeout()

        if target_lost and not self._target_lost_prev:
            self.logger.warning("[PL] 目标丢失")
        elif not target_lost and self._target_lost_prev:
            self.logger.info("[PL] 目标重新获取")
        self._target_lost_prev = target_lost
        # --- 状态机逻辑 ---
        if self.state == PrecisionLandState.IDLE:
            # 保持不动，等待指令
            return

        elif self.state == PrecisionLandState.SEARCH:
            # 在当前位置悬停，等待Aruco Tag出现
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            if not target_lost:
                self.logger.info("[PL] 搜索到目标，切换到接近模式")
                self._switch_to_state(PrecisionLandState.APPROACH)

        elif self.state == PrecisionLandState.APPROACH:
            if target_lost:
                self.logger.warning("[PL] 在接近过程中丢失目标，返回搜索模式")
                self._switch_to_state(PrecisionLandState.SEARCH)
                return 
            
            # 计算水平速度，飞向目标正上方
            vx, vy = self._calculate_horizontal_velocity()
            # 检查是否已到达目标正上方
            if self._is_horizontally_aligned():
                self.logger.info("[PL] 已到达目标正上方，切换到下降模式")
                self._switch_to_state(PrecisionLandState.DESCEND)
            
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, 0.0, 0.0))
        
        elif self.state == PrecisionLandState.DESCEND:
            if target_lost:
                self.logger.warning("[PL] 在下降过程中丢失目标，返回搜索模式")
                self._switch_to_state(PrecisionLandState.SEARCH)
                return 
              
            # 同时计算水平对准速度，并施加一个恒定的下降速度
            vx, vy = self._calculate_horizontal_velocity()
            vz = self.params['descent_vel']
              
            # 如果检测到着陆，则任务完成
            if self.drone_state.landed:
                self.logger.info("[PL] 无人机已着陆，任务完成！")
                self._switch_to_state(PrecisionLandState.FINISHED)
                return 

            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, 0.0))
        
        elif self.state == PrecisionLandState.FINISHED:
            self.drone_state.search_started = False
            await self.drone.offboard.stop()
            await drone.action.disarm()
            self._switch_to_state(PrecisionLandState.IDLE)
        
    def _is_horizontally_aligned(self) -> bool:
        """
        检查无人机是否在目标的水平位置容差范围内。
        仅使用X和Y轴进行判断。
        """
        if not self._tag.is_valid():
            return False
        # 计算机体坐标系下，目标位置在XY平面上的距离
        horizontal_dist = np.linalg.norm(self._tag.position[0:2])
        return horizontal_dist < self.params['delta_position']
        
    def _check_target_timeout(self):
        """检查目标是否超时"""
        return (time.time() - self._tag.timestamp) > self.params['target_timeout'] or not self._tag.is_valid()

    def _calculate_horizontal_velocity(self) -> Tuple[float, float]:
        """
        根据标记在机体坐标系的位置，计算水平速度指令(vx, vy)。
        """
        if not self._tag.is_valid():
            return 0.0, 0.0

        error_xy=self._tag.position[0:2]
        
         # 简单的P控制器
        kp = 0.5  # 比例增益
        v_xy = kp * error_xy
        
        # 限制最大速度
        max_vel = self.params['max_vel_xy']
        norm = np.linalg.norm(v_xy)
        if norm > max_vel:
            v_xy = v_xy / norm * max_vel
            
        return v_xy[0], v_xy[1]  # 返回vx, vy

    # def _calculate_yaw_speed(self) -> float:
    #     """
    #     【新增】计算偏航角速度指令，用于对准目标的姿态。
    #     """
    #     if not self._tag.is_valid():
    #         return 0.0
            
    #     # 从姿态中提取偏航角（绕机体Z轴的旋转）
    #     # 'zyx' 表示先绕z轴，再绕y轴，最后绕x轴。yaw是绕z轴的第一个角度。
    #     tag_euler_angles = self._tag.orientation.as_euler('zyx', degrees=False)
    #     yaw_error_rad = tag_euler_angles[0] # 获取偏航角误差（弧度）

    #     # 简单的P控制器
    #     # yaw_error_rad > 0 表示目标姿态在当前机头的逆时针方向，需要逆时针转动机体
    #     # MAVSDK中，yawspeed > 0 为顺时针旋转。因此需要加一个负号。
    #     yaw_speed = -self.params['yaw_p_gain'] * yaw_error_rad

    #     # 简单的限幅
    #     return np.clip(yaw_speed, -0.5, 0.5) # 限制最大转速为0.5 rad/s

    def _switch_to_state(self, new_state: PrecisionLandState):
        """切换状态并记录日志。"""
        if self.state != new_state:
            self.logger.info(f"[PL] 状态切换: {self.state.value} -> {new_state.value}")
            self.state = new_state
    
        # def _get_tag_body(self, tag_cam):
    #     # 查 camera_link → base_link 的变换
    #     try:
    #         trans = self.tf_buffer.lookup_transform(
    #             'base_link', 'camera_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
    #     except Exception as ex:
    #         self.get_logger().warn(f'TF lookup failed: {ex}')
    #         return None

    #     # 旋转矩阵
    #     q = trans.transform.rotation
    #     R = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

    #     # 平移向量
    #     t = np.array([trans.transform.translation.x,
    #                   trans.transform.translation.y,
    #                   trans.transform.translation.z])

    #     # 把 tag 从 camera_link 转到 base_link
    #     tag_body_pos = R @ tag_cam + t
    #     return tag_body_pos