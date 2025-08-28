#!/usr/bin/env python3
"""
精准降落模块 - 基于ArUco标记的精准降落控制
"""

import time
import asyncio
from enum import Enum
from typing import Optional, Tuple, Callable
import numpy as np
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw
from scipy.spatial.transform import Rotation as R
from drone_control.utils import ControlMode
class PrecisionLandState(Enum):
    """精准降落状态枚举"""
    IDLE = "IDLE"
    SEARCH = "SEARCH"  # 添加搜索状态
    APPROACH = "APPROACH"
    DESCEND = "DESCEND"
    FINISHED = "FINISHED"

class ArucoTag:
    """存储ArUco标记信息，类似C++版本中的ArucoTag结构体。"""
    def __init__(self, position=None, orientation=None, timestamp=0):
        self.position = np.array(position,dtype=float) if position is not None else None
        self.orientation = R.from_quat(orientation) if orientation is not None else None
        self.timestamp = timestamp

    def is_valid(self) -> bool:
        """检查标记数据是否有效。"""
        return self.timestamp > 0

class PrecisionLand:
    """精准降落控制类，负责状态管理和控制指令计算。"""
    
    def __init__(self, drone, logger, drone_state):
        self.drone = drone
        self.logger = logger
        self.drone_state = drone_state
        # 状态管理
        self.state = PrecisionLandState.IDLE  # 初始状态设为IDLE
        self._tag = ArucoTag()  # 使用ArucoTag结构存储目标信息
        self._vel_x_integral = 0.0
        self._vel_y_integral = 0.0
        self._target_lost_prev = False
        self._approach_altitude = 0.0
        # 搜索模式相关
        self.search_waypoints = []
        self.search_waypoint_index = 0
        self.search_started = False
        
        # 控制参数 (可配置)
        self.params = {
            'descent_vel': 0.6,       # 下降速度 (m/s)
            'vel_p_gain': 1.7,             # 位置P增益
            'vel_i_gain': 0.0,            # 位置I增益
            'max_vel_xy': 3.0,        # 最大水平速度 (m/s)
            'target_timeout': 1.0,    # 目标超时时间 (s)
            'delta_position': 0.2, # 位置到达阈值 (m)
            'delta_velocity': 0.2,  # 速度阈值 (m/s)

            'search_max_radius': 2.0,  # 搜索最大半径 (m)
            'search_layer_spacing': 0.5, # 搜索层间距 (m)
            'search_points_per_layer': 16, # 每层搜索点数
            'search_min_altitude': 1.0 # 搜索最低高度 (m)          
        }

        self.logger.info("[PL] 精准降落控制器已初始化")

    def start_precision_land(self):
        """开始精准降落。"""
        self.drone_state.search_started = True
        self._switch_to_state(PrecisionLandState.SEARCH)
        self._generate_search_waypoints()
        self.logger.info(f"当前位置: { np.array([self.drone_state.current_position_ned.north_m, self.drone_state.current_position_ned.east_m, self.drone_state.current_position_ned.down_m])}")

    def _get_tag_world(self, position, orientation):
        # Convert from optical to NED
        # Optical: X right, Y down, Z away from lens
        # NED: X forward, Y right, Z away from viewer        
        tag=ArucoTag(position=position, orientation=orientation,timestamp=time.time())

        R_mat = np.array([
            [0, -1, 0],
            [1,  0, 0],
            [0,  0, 1]
        ], dtype=float)
        quat_NED = R.from_matrix(R_mat)

        # 处理 PositionNed 对象，提取 x, y, z 坐标
        
        vehicle_position = np.array([
                self.drone_state.current_position_ned.north_m,
                self.drone_state.current_position_ned.east_m,
                self.drone_state.current_position_ned.down_m
        ], dtype=float)

        attitude_quaternion = np.array([
            self.drone_state.attitude_quaternion.x,
            self.drone_state.attitude_quaternion.y,
            self.drone_state.attitude_quaternion.z,
            self.drone_state.attitude_quaternion.w
        ], dtype=float)
        
        
        vehicle_orientation = R.from_quat(attitude_quaternion)  

        # Drone transform
        drone_transform = np.eye(4)
        drone_transform[:3, :3] = vehicle_orientation.as_matrix()
        drone_transform[:3, 3] = vehicle_position

        # Camera transform
        camera_transform = np.eye(4)
        camera_transform[:3, :3] = quat_NED.as_matrix()

        # Tag transform
        tag_transform = np.eye(4)
        tag_transform[:3, :3] = tag.orientation.as_matrix()
        tag_transform[:3, 3] = tag.position

        # World transform
        tag_world_transform = drone_transform @ camera_transform @ tag_transform

        self._tag= ArucoTag(position=tag_world_transform[:3, 3],
         orientation=R.from_matrix(tag_world_transform[:3, :3]).as_quat(),
          timestamp=tag.timestamp)

    async def update(self):
        """主更新函数，由外部节点定期调用。""" 
        if not self.drone_state.search_started:
            return
        target_lost = self._check_target_timeout()

        if target_lost and not self._target_lost_prev:
            self.logger.warning("[PL] 目标丢失")
        elif not target_lost and self._target_lost_prev:
            self.logger.info("[PL] 目标重新获取")
        self._target_lost_prev = target_lost
        # --- 状态机逻辑 ---
        try:
            if self.state == PrecisionLandState.IDLE:
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0)) 
            elif self.state == PrecisionLandState.SEARCH:
                if not self._check_target_timeout() and self._tag.position is not None:
                    self._approach_altitude = self.drone_state.current_position_ned.down_m
                    self._switch_to_state(PrecisionLandState.APPROACH)
                    return 
                    
                # 获取当前目标航点
                if not self.search_waypoints: # 如果航点列表为空，则不执行后续操作
                    self.logger.warning("[PL_SEARCH] 搜索航点列表为空，无法导航。")
                    await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                    return

                target_waypoint = self.search_waypoints[self.search_waypoint_index]
                
                # 检查是否到达当前航点
                if self._position_reached(target_waypoint):
                    self.search_waypoint_index += 1
                    
                    # 如果已完成所有航点，重新开始
                    if self.search_waypoint_index >= len(self.search_waypoints):
                        self.search_waypoint_index = 0
                        self.logger.info("[PL] 完成一圈搜索，重新开始")
                    else:
                        self.logger.debug("[PL] 前往搜索航点 %d/%d" % 
                                    (self.search_waypoint_index + 1, 
                                    len(self.search_waypoints)))
                
                # 计算到当前目标航点的速度指令
                target_waypoint = self.search_waypoints[self.search_waypoint_index]
                vx,vy=self._calculate_velocity_to_waypoint(target_waypoint)
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vx, vy, 0.0, 0.0))
                # await self.drone.offboard.set_position_ned(PositionNedYaw(target_waypoint[0], target_waypoint[1], target_waypoint[2], 0.0))

            elif self.state == PrecisionLandState.APPROACH:
                if target_lost:
                    self.logger.error(f"Failed! Target lost during {self.state.value}")
                    self._switch_to_state(PrecisionLandState.SEARCH)
                    return 
                target_position = np.array([self._tag.position[0], self._tag.position[1], self._approach_altitude])
        
                # 检查是否到达目标位置
                if self._position_reached(target_position):
                    self._switch_to_state(PrecisionLandState.DESCEND)
                
                vx,vy=self._calculate_velocity_to_waypoint(target_position)
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vx, vy, 0.0, 0.0))
                # await self.drone.offboard.set_position_ned(PositionNedYaw(target_position[0], target_position[1], target_position[2], 0.0))

            elif self.state == PrecisionLandState.DESCEND:
                if target_lost:
                    self.logger.error(f"Failed! Target lost during {self.state.value}")
                    self._switch_to_state(PrecisionLandState.SEARCH)
                    return 
                
                # 计算对准目标的水平速度
                vx, vy = self._calculate_velocity_to_waypoint(self._tag.position)  # 计算XY平面速度
                vz = self.params['descent_vel']  # 下降速度
                
                # 计算偏航角（从目标标记的四元数中提取）
                # yaw = self._quaternion_to_yaw(self._tag.orientation)
                
                # 如果检测到着陆，切换到完成状态

                """降落不行"""
                if self.drone_state.landed:
                    self.logger.info("[PL] 无人机已检测到着陆，任务完成")
                    self._switch_to_state(PrecisionLandState.FINISHED)
                
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vx, vy, vz, 0.0))
                

            elif self.state == PrecisionLandState.FINISHED:
                self.drone_state.search_started = False
                self._switch_to_state(PrecisionLandState.IDLE)
                await self.drone.offboard.stop()
        except Exception as e:
            self.logger.error(f"[PL] 状态机更新异常: {e}")
        

    
    def _check_target_timeout(self):
        """检查目标是否超时"""
        return (time.time() - self._tag.timestamp) > self.params['target_timeout'] or not self._tag.is_valid()
    
    def _position_reached(self, target: np.ndarray) -> bool:
        """检查是否到达目标位置
        Args:   
            target: 目标位置，形如 [x, y, z] 的numpy数组，对应 [north, east, down]

        """
        # 获取当前位置
        pos = self.drone_state.current_position_ned
        current_pos = np.array([pos.north_m, pos.east_m, pos.down_m])
        
        # 获取当前速度
        vel = self.drone_state.current_velocity_ned
        current_vel = np.array([vel.north_m_s, vel.east_m_s, vel.down_m_s])
        
        # 计算位置差和速度的范数
        delta_pos = np.linalg.norm(target - current_pos)
        vel_norm = np.linalg.norm(current_vel)
        
        # 检查位置和速度是否都在阈值范围内
        position_ok = delta_pos < self.params['delta_position']
        velocity_ok = vel_norm < self.params['delta_velocity']
        # self.logger.info(f"目标位置{target}, 当前位置{current_pos}, 位置差{delta_pos}, 速度{vel_norm}")
        return position_ok and velocity_ok
    
    def _generate_search_waypoints(self):
        """生成螺旋搜索路径点。"""
        start_x = self.drone_state.current_position_ned.north_m
        start_y = self.drone_state.current_position_ned.east_m
        current_z = self.drone_state.current_position_ned.down_m
        min_z = self.params['search_min_altitude']
        
        max_radius = self.params['search_max_radius']
        layer_spacing = self.params['search_layer_spacing']
        points_per_layer = self.params['search_points_per_layer']
        
        waypoints = []
        
        # 计算需要的层数
        num_layers = max(1, int((current_z - min_z) / layer_spacing) // 2)
        
        for layer in range(num_layers):
            # 向外螺旋
            radius = 0.0
            for point in range(points_per_layer + 1):
                angle = 2.0 * np.pi * point / points_per_layer
                x = start_x + radius * np.cos(angle)
                y = start_y + radius * np.sin(angle)
                z = current_z
                waypoints.append(np.array([x, y, z]))
                radius += max_radius / points_per_layer
            
            # 更新高度
            current_z = max(min_z, current_z - layer_spacing)
            
            # 向内螺旋（反向）
            radius = max_radius
            for point in range(points_per_layer, -1, -1):
                angle = 2.0 * np.pi * point / points_per_layer
                x = start_x + radius * np.cos(angle)
                y = start_y + radius * np.sin(angle)
                z = current_z
                waypoints.append(np.array([x, y, z]))
                radius -= max_radius / points_per_layer
            
            # 更新高度
            current_z = max(min_z, current_z - layer_spacing)
        
        self.search_waypoints = waypoints
        self.search_waypoint_index = 0
        
    def _calculate_velocity_to_waypoint(self, target_waypoint: np.ndarray = None) -> Tuple[float, float]:
        """计算到目标航点的速度指令。"""
        current_pos = np.array([
            self.drone_state.current_position_ned.north_m,
            self.drone_state.current_position_ned.east_m,
            self.drone_state.current_position_ned.down_m
        ])
        
        # 计算误差
        error = target_waypoint - current_pos
        error_xy = error[:2]  # 只考虑水平面
        
        # 简单的P控制器
        kp = 0.5  # 比例增益
        v_xy = kp * error_xy
        
        # 限制最大速度
        max_vel = self.params['max_vel_xy']
        norm = np.linalg.norm(v_xy)
        if norm > max_vel:
            v_xy = v_xy / norm * max_vel
            
        return v_xy[0], v_xy[1]  # 返回vx, vy

    # def _calculate_velocity_setpoint_xy(self) -> Tuple[float, float]:
    #     """根据目标的相机坐标系位置，计算水平速度指令。"""
    #     if not self._tag.is_valid():
    #         return 0.0, 0.0 
    #     p_gain=self.params['vel_p_gain']
    #     i_gain=self.params['vel_i_gain']

    #     # P控制器
    #     delta_pos_x=self.drone_state.current_position_ned.north_m-self._tag.position[0]
    #     delta_pos_y=self.drone_state.current_position_ned.east_m-self._tag.position[1]
    #     # I控制器
    #     self._vel_x_integral+=delta_pos_x
    #     self._vel_y_integral+=delta_pos_y
    #     max_integral=self.params['max_vel_xy']
    #     self._vel_x_integral=np.clip(self._vel_x_integral,-max_integral,max_integral)
    #     self._vel_y_integral=np.clip(self._vel_y_integral,-max_integral,max_integral)

    #     Xp=delta_pos_x* p_gain
    #     Xi=self._vel_x_integral* i_gain
    #     Yp=delta_pos_y* p_gain
    #     Yi=self._vel_y_integral* i_gain
        
    #     # Sum P and I gains
    #     vx=-(Xp+Xi)
    #     vy=-(Yp+Yi)
        
    #     vx=np.clip(vx,-self.params['max_vel_xy'],self.params['max_vel_xy'])
    #     vy=np.clip(vy,-self.params['max_vel_xy'],self.params['max_vel_xy'])
       
    #     return vx, vy

    def _switch_to_state(self, new_state: PrecisionLandState):
        """切换状态并记录日志。"""
        if self.state != new_state:
            self.logger.info(f"[PL] 状态切换: {self.state.value} -> {new_state.value}")
            self.state = new_state
