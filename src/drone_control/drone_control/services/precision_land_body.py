import time
import asyncio
from enum import Enum
from typing import Tuple, Optional
import numpy as np
from mavsdk.offboard import VelocityBodyYawspeed
from scipy.spatial.transform import Rotation as R

class PrecisionLandState(Enum):
    """精准降落状态枚举"""
    IDLE = "IDLE"
    SEARCH = "SEARCH"
    APPROACH_GUIDE_TAG = "APPROACH_GUIDE_TAG"  # 接近引导标记
    ALIGN_ON_PLATFORM = "ALIGN_ON_PLATFORM"    # 在平台上对准
    DESCEND = "DESCEND"                        # 下降
    FAILSAFE_HOVER = "FAILSAFE_HOVER"          # 安全悬停
    INERTIAL_LAND = "INERTIAL_LAND"          # 惯性降落
    FINISHED = "FINISHED"

class ArucoTag:
    """存储ArUco标记信息，包含位置、姿态和时间戳。"""
    def __init__(self, position=None, orientation=None, timestamp=0):
        self.position = position
        self.orientation = orientation
        self.timestamp = timestamp

    def is_valid(self) -> bool:
        """检查标记数据是否有效。"""
        return self.timestamp > 0 

class PrecisionLand:
    """
    精准降落控制类，负责状态管理和PID控制指令计算。
    """
    def __init__(self, mavsdk_controller, logger, drone_state):
        self.mavsdk_controller = mavsdk_controller
        self.logger = logger
        self.drone_state = drone_state
        self.state = PrecisionLandState.SEARCH
        self._tag_down = ArucoTag()
        self._tag_front = ArucoTag()
        self._target_down_lost_prev = False
        self._target_front_lost_prev = False
        self._failsafe_entry_time = 0
        self._inertial_land_entry_time = 0
        self._timer=0

        self.params = {
            'target_timeout': 2.0,
            'delta_position': 0.15,  # 水平对准容差 (m)
            'approach_dist_x': 1.0, # 接近引导标记时的目标悬停距离 (m, 机体前向)
            'failsafe_timeout': 10.0, # Failsafe悬停超时时间 (s)
            'inertial_land_time': 3.0, # 惯性降落时间 (s)
            'inertial_land_altitude': 0.4, # 惯性降落高度 (m)
            'inertial_land_vel': 0.3, # 惯性降落速度 (m/s)
        }
        self._r_cb_front = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        self._r_cb_down = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        self.logger.info("[PL] 精准降落控制器已初始化")

    async def start_precision_land(self):
        """外部调用此函数以开始精准降落流程。"""
        self.drone_state.search_started = True
        self._switch_to_state(PrecisionLandState.SEARCH)
        self.logger.info("[PL] 开始精准降落流程")

    def process_tag_detection(self, position_cam, orientation_cam_quat, source):
        """处理来自图像节点的ArUco Tag检测结果。"""
        R_cb = self._r_cb_down if source == 'down' else self._r_cb_front
        tag_body_position = R_cb @ np.array(position_cam, dtype=float)
        R_orientation_in_cam = R.from_quat(orientation_cam_quat)
        R_body_to_cam = R.from_matrix(R_cb.T)
        tag_body_orientation = R_body_to_cam * R_orientation_in_cam

        if source == 'down':
            self._tag_down = ArucoTag(position=tag_body_position, orientation=tag_body_orientation.as_matrix(), timestamp=time.time())
        else:
            self._tag_front = ArucoTag(position=tag_body_position, orientation=tag_body_orientation.as_matrix(), timestamp=time.time())

    async def update(self):
        """主更新函数，由外部节点定期调用以驱动状态机。"""
        if not self.drone_state.search_started:
            return

        # 获取最新的有效标记信息
        is_front_valid = self._check_tag_valid(self._tag_front)
        is_down_valid = self._check_tag_valid(self._tag_down)
        if self._timer== 15:
            self._timer=0
            self.logger.info(f"[PL] 前置标记有效: {is_front_valid}, 平台标记有效: {is_down_valid},state{self.state}")
        else:
            self._timer+=1

        if not is_front_valid and not self._target_front_lost_prev:
            self.logger.warning("[PL] 前置引导标记丢失")
        elif is_front_valid and self._target_front_lost_prev:
            self.logger.info("[PL] 前置引导标记重新获取")
        self._target_front_lost_prev = not is_front_valid

        if not is_down_valid and not self._target_down_lost_prev:
            self.logger.warning("[PL] 平台标记丢失")
        elif is_down_valid and self._target_down_lost_prev:
            self.logger.info("[PL] 平台标记重新获取")
        self._target_down_lost_prev = not is_down_valid
        # --- 状态机逻辑 ---
        if self.state == PrecisionLandState.IDLE:
            if self.drone_state.search_started:
                self._switch_to_state(PrecisionLandState.SEARCH)
                return
        elif self.state == PrecisionLandState.SEARCH:
            await self.mavsdk_controller.set_velocity_body(0.0, 0.0, 0.0, 0.0)
            # 优先寻找前置引导标记
            if is_front_valid:
                self.logger.info("[PL] 发现引导标记，开始接近...")
                self._switch_to_state(PrecisionLandState.APPROACH_GUIDE_TAG)
            elif is_down_valid:
                # 如果直接看到了平台标记，也可以直接开始对准
                self.logger.info("[PL] 直接发现平台标记，开始对准...")
                self._switch_to_state(PrecisionLandState.ALIGN_ON_PLATFORM)

        elif self.state == PrecisionLandState.APPROACH_GUIDE_TAG:
            if not is_front_valid:
                self.logger.warning("[PL] 引导标记丢失，返回搜索模式")
                self._switch_to_state(PrecisionLandState.SEARCH)
                return

            # 如果此时看到了平台标记，说明已到达位置，切换到对准阶段
            if is_down_valid:
                self.logger.info("[PL] 发现平台标记，切换到平台对准模式")
                self._switch_to_state(PrecisionLandState.ALIGN_ON_PLATFORM)
                return

            # --- 核心逻辑：飞向引导标记，并保持在 'approach_dist_x' 的距离 ---
            error_x = self._tag_front.position[0] - self.params['approach_dist_x']
            error_y = self._tag_front.position[1]
            error_z = self._tag_front.position[2]
            
            vx = np.clip(0.3 * error_x, -0.3, 0.3)
            vy = np.clip(0.4 * error_y, -0.3, 0.3)
            vz = np.clip(0.3 * error_z, -0.3, 0.3)
            # 安全检查：如果离引导标记太近但仍未看到平台标记，则进入悬停
            if error_x < 0.01:
                self.logger.warning("[PL] 离引导标记过近但未发现平台标记，进入安全悬停！")
                self._failsafe_entry_time = time.time()
                self._switch_to_state(PrecisionLandState.FAILSAFE_HOVER)
                return

            await self.mavsdk_controller.set_velocity_body(vx, vy, vz, 0.0)

        elif self.state == PrecisionLandState.ALIGN_ON_PLATFORM:
            if not is_down_valid:
                self.logger.warning("[PL] 平台标记丢失，返回搜索模式")
                self._switch_to_state(PrecisionLandState.SEARCH)
                return

            # 检查是否已水平对准
            if self._is_horizontally_aligned():
                self.logger.info("[PL] 水平对准完成，开始下降")
                self._switch_to_state(PrecisionLandState.DESCEND)
                return
            
            # 计算速度以对准平台标记，但不下降 (vz=0)
            vx, vy, _ = self._calculate_velocity_for_tag(self._tag_down)
            await self.mavsdk_controller.set_velocity_body(vx, vy, 0.0, 0.0)

        elif self.state == PrecisionLandState.DESCEND:
            if not is_down_valid and self._tag_down.position[2] > self.params['inertial_land_altitude']:
                self.logger.warning("[PL] 下降过程中平台标记丢失，返回搜索模式")
                self._switch_to_state(PrecisionLandState.SEARCH)
                return
            if self._tag_down.position[2] <= self.params['inertial_land_altitude']:
                self.logger.info(f"[PL] 下降到{self.params['inertial_land_altitude']}m，切换到惯性降落")
                self._inertial_land_entry_time = time.time()
                self._switch_to_state(PrecisionLandState.INERTIAL_LAND)
                return

            # 计算速度，同时进行水平对准和垂直下降
            vx, vy, vz = self._calculate_velocity_for_tag(self._tag_down)
            await self.mavsdk_controller.set_velocity_body(vx, vy, vz, 0.0)

        elif self.state == PrecisionLandState.FAILSAFE_HOVER:
            # 安全悬停，等待超时
            await self.mavsdk_controller.set_velocity_body(0.0, 0.0, 0.0, 0.0)
            if time.time() - self._failsafe_entry_time > self.params['failsafe_timeout']:
                self.logger.info("[PL] 安全悬停超时，返回搜索模式")
                self._switch_to_state(PrecisionLandState.SEARCH)

        elif self.state == PrecisionLandState.INERTIAL_LAND:
            # 惯性降落
            await self.mavsdk_controller.set_velocity_body(0.0, 0.0, self.params['inertial_land_vel'], 0.0)
            if time.time() - self._inertial_land_entry_time > self.params['inertial_land_time']:
                self.logger.info("[PL] 惯性降落完成")
                self._switch_to_state(PrecisionLandState.FINISHED)
            
        elif self.state == PrecisionLandState.FINISHED:
            self.drone_state.search_started = False
            await self.mavsdk_controller.land_autodisarm()
            # await self.mavsdk_controller.stop_offboard()
            # await self.mavsdk_controller.disarm()
            self._switch_to_state(PrecisionLandState.IDLE)

    def _is_horizontally_aligned(self) -> bool:
        """检查无人机是否在平台标记的水平容差范围内。"""
        if not self._tag_down.is_valid():
            return False
        horizontal_dist = np.linalg.norm(self._tag_down.position[0:2])
        return horizontal_dist < self.params['delta_position']

    def _check_tag_valid(self, tag) -> bool:
        """检查标记是否有效且未超时。"""
        if tag is None:
            return False
        return (time.time() - tag.timestamp) <= self.params['target_timeout'] and tag.is_valid()

    def _calculate_velocity_for_tag(self, tag: ArucoTag) -> Tuple[float, float, float]:
        """根据指定标记的位置计算三轴速度指令。"""
        if not self._check_tag_valid(tag):
            return 0.0, 0.0, 0.0

        error_xyz = tag.position
        v_xy = 0.3 * error_xyz[0:2]
        v_z = 0.4 * error_xyz[2]

        # 限制最大水平速度
        norm_xy = np.linalg.norm(v_xy)
        if norm_xy > 0.3:
            v_xy = v_xy / norm_xy * 0.3

        # 限制最大垂直速度
        v_z = np.clip(v_z, -0.3, 0.3)

        return v_xy[0], v_xy[1], v_z

    def _switch_to_state(self, new_state: PrecisionLandState):
        """切换状态并记录日志。"""
        if self.state != new_state:
            self.logger.info(f"[PL] 状态切换: {self.state.value} -> {new_state.value}")
            self.state = new_state
