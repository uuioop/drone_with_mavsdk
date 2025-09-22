import time
import asyncio
from enum import Enum
from typing import Tuple, Optional
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque

class PrecisionLandState(Enum):
    """精准降落状态枚举"""
    IDLE = "IDLE"
    SEARCH = "SEARCH"
    APPROACH_GUIDE_TAG = "APPROACH_GUIDE_TAG"  # 接近引导标记
    ALIGN_ON_GUIDE_TAG = "ALIGN_ON_GUIDE_TAG"  # 在引导标记上对准
    ALIGN_ON_PLATFORM = "ALIGN_ON_PLATFORM"    # 在平台上对准
    DESCEND = "DESCEND"                        # 下降
    FAILSAFE_HOVER = "FAILSAFE_HOVER"          # 安全悬停
    INERTIAL_LAND = "INERTIAL_LAND"          # 惯性降落
    FINISHED = "FINISHED"

class ArucoTag:
    """存储ArUco标记信息，包含位置、姿态和时间戳。"""
    def __init__(self, position: np.ndarray=None, orientation: Optional[R]=None, timestamp=0):
        self.position = position
        self.orientation = orientation
        self.timestamp = timestamp

    def is_valid(self) -> bool:
        """检查标记数据是否有效。"""
        return self.timestamp > 0 and self.position is not None and self.orientation is not None

class PrecisionLand:
    """
    精准降落控制类，负责状态管理和PID控制指令计算。
    """
    def __init__(self, mavsdk_controller, logger, drone_state):
        self.mavsdk_controller = mavsdk_controller
        self.logger = logger
        self.drone_state = drone_state
        self.state = PrecisionLandState.IDLE
        self._tag_down = ArucoTag()
        self._tag_front = ArucoTag()
        self._target_down_lost_prev = False
        self._target_front_lost_prev = False
        self._failsafe_entry_time = 0
        self._inertial_land_entry_time = 0
        self._timer=0
        # 新增：初始化滤波后的偏航误差
        self._filtered_yaw_error_deg = 0.0
        self.filter_alpha = 0.1  # 新增：定义滤波系数 alpha
        self.yaw_error_history = deque(maxlen=5) # 存储最近5个值

        self._previous_error = np.zeros(2)
        self._integral_error = np.zeros(2)
        self._last_update_time = 0
        self.pid_gains = {
            'kp': 0.5,
            'ki': 0.01,
            'kd': 0.1
        }
        self.params = {
            'target_timeout': 4.0,
            'delta_position': 0.1,  # 对准容差 (m)
            'approach_dist_x': 1.8, # 接近引导标记时的目标悬停距离 (m, 机体前向)
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
        if np.linalg.norm(orientation_cam_quat) == 0:          # 零范数
            self.logger.warning(f"[PL] {source} 四元数全零，丢弃")
            return
        R_cb = self._r_cb_down if source == 'down' else self._r_cb_front
        tag_body_position = R_cb @ np.array(position_cam, dtype=float)
        R_orientation_in_cam = R.from_quat(orientation_cam_quat)
        R_cam_to_body = R.from_matrix(R_cb)
        tag_body_orientation = R_cam_to_body * R_orientation_in_cam

        if source == 'down':
            self._tag_down = ArucoTag(position=tag_body_position, orientation=tag_body_orientation, timestamp=time.time())
        else:
            self._tag_front = ArucoTag(position=tag_body_position, orientation=tag_body_orientation, timestamp=time.time())

    async def update(self):
        """主更新函数，由外部节点定期调用以驱动状态机。"""
        if not self.drone_state.search_started:
            return
        yaw_rate = 0.0
        # 获取最新的有效标记信息
        is_front_valid = self._check_tag_valid(self._tag_front)
        is_down_valid = self._check_tag_valid(self._tag_down)

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
                await self.mavsdk_controller.start_offboard()
                await self.mavsdk_controller.arm()
                self._switch_to_state(PrecisionLandState.SEARCH)
        elif self.state == PrecisionLandState.SEARCH:
            if is_front_valid:
                yaw_rate = self._rotate_to_yaw_error(self._tag_front)
            await self.mavsdk_controller.set_velocity_body(0.0, 0.0, -0.8, yaw_rate)

            # 优先寻找前置引导标记
            if is_front_valid and abs(self._tag_front.position[2]) <= 0.6:
                self.logger.info("[PL] 发现引导标记，开始对准...")
                self._switch_to_state(PrecisionLandState.ALIGN_ON_GUIDE_TAG)
            # 后面写判断是否角度误差大，才能直接进入平台对准
            # elif is_down_valid:
            #     # 如果直接看到了平台标记，也可以直接开始对准
            #     self.logger.info("[PL] 直接发现平台标记，开始对准...")
            #     self._switch_to_state(PrecisionLandState.ALIGN_ON_PLATFORM)

        elif self.state == PrecisionLandState.ALIGN_ON_GUIDE_TAG:
            if not is_front_valid:
                self.logger.warning("[PL] 引导标记丢失，返回搜索模式")
                self._switch_to_state(PrecisionLandState.SEARCH)
                return
            
            # if is_down_valid:
            #     self.logger.info("[PL] 发现平台标记，切换到平台对准模式")
            #     self._switch_to_state(PrecisionLandState.ALIGN_ON_PLATFORM)
            #     return
            
            error_y = self._tag_front.position[1]
            error_z = self._tag_front.position[2]
            yaw_rate = self._rotate_to_yaw_error(self._tag_front)
            # 计算速度
            _,vy,vz = self._calculate_velocity_for_tag(self._tag_front,np.array([0,error_y,error_z]))
            # self.logger.info(f"[PL] y={error_y:.2f} z={error_z:.2f} ")
            # 检查是否对准（位置 + 姿态）
            if abs(error_y) <= self.params['delta_position']*1.5 and abs(error_z) <= self.params['delta_position']*1.5:
                self.logger.info("[PL] 引导标记对准完成（含姿态），开始接近")
                self._switch_to_state(PrecisionLandState.APPROACH_GUIDE_TAG)
                return
            
            await self.mavsdk_controller.set_velocity_body(0.0, vy, vz, yaw_rate)


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
            vx,vy,vz = self._calculate_velocity_for_tag(self._tag_front,np.array([error_x,error_y,error_z]))
            yaw_rate = self._rotate_to_yaw_error(self._tag_front)
            # 安全检查：如果离引导标记太近但仍未看到平台标记，则进入悬停
            if error_x < -0.5:
                self.logger.warning("[PL] 离引导标记过近但未发现平台标记，进入安全悬停！")
                self._failsafe_entry_time = time.time()
                self._switch_to_state(PrecisionLandState.FAILSAFE_HOVER)
                return

            await self.mavsdk_controller.set_velocity_body(vx, vy, vz, yaw_rate)

        elif self.state == PrecisionLandState.ALIGN_ON_PLATFORM:
            if not is_down_valid:
                self.logger.warning("[PL] 平台标记丢失，返回搜索模式")
                self._switch_to_state(PrecisionLandState.SEARCH)
                return

            # 检查是否已水平对准
            if self._is_aligned(self._tag_down,'platform'):
                self.logger.info("[PL] 水平对准完成，开始下降")
                self._switch_to_state(PrecisionLandState.DESCEND)
                return
            
            # 计算速度以对准平台标记，但不下降 (vz=0)
            vx, vy, _ = self._calculate_velocity_for_tag(self._tag_down)
            # vx,vy=self._calculate_velocity_for_tag_pid(self.pid_gains)
            await self.mavsdk_controller.set_velocity_body(vx, vy, 0.0, 0.0)

        elif self.state == PrecisionLandState.DESCEND:
            # if not is_down_valid and self._tag_down.position[2] > self.params['inertial_land_altitude']:
            #     self.logger.warning("[PL] 下降过程中平台标记丢失，返回搜索模式")
            #     self._switch_to_state(PrecisionLandState.SEARCH)
            #     return
            # if self._tag_down.position[2] <= self.params['inertial_land_altitude']:
            #     self.logger.info(f"[PL] 下降到{self.params['inertial_land_altitude']}m，切换到惯性降落")
            #     self._inertial_land_entry_time = time.time()
            #     self._switch_to_state(PrecisionLandState.INERTIAL_LAND)
            #     return

            # # 计算速度，同时进行水平对准和垂直下降
            # vx, vy, vz = self._calculate_velocity_for_tag(self._tag_down)
            # await self.mavsdk_controller.set_velocity_body(vx, vy, vz, 0.0)
            if not is_down_valid:
                await self.mavsdk_controller.land()
                self._switch_to_state(PrecisionLandState.FINISHED)
                return
            vz = 0.5
            vx,vy,_ = self._calculate_velocity_for_tag(self._tag_down)
            # vx,vy=self._calculate_velocity_for_tag_pid(self.pid_gains)
            await self.mavsdk_controller.set_velocity_body(vx, vy, vz, 0.0)


        elif self.state == PrecisionLandState.FAILSAFE_HOVER:
            # 安全悬停，等待超时
            await self.mavsdk_controller.set_velocity_body(0.0, 0.0, 0.0, 0.0)
            if time.time() - self._failsafe_entry_time > self.params['failsafe_timeout']:
                self.logger.info("[PL] 安全悬停超时，返回搜索模式")
                self._switch_to_state(PrecisionLandState.SEARCH)
            
        elif self.state == PrecisionLandState.FINISHED:
            if self.drone_state.landed:
                self.drone_state.search_started = False
                await self.mavsdk_controller.disarm()
                self._switch_to_state(PrecisionLandState.IDLE)

    def _is_aligned(self, tag: ArucoTag, mode: str = 'platform') -> bool:
        """通用对准检查
        mode = 'platform'  -> 检查 x-y 平面误差（下视）
        mode = 'guide'     -> 检查 y-z 平面误差（前置引导）
        """
        if not tag.is_valid():
            return False

        if mode == 'platform':
            error = np.array([tag.position[0], tag.position[1]])  # x, y
        elif mode == 'guide':
            error = np.array([tag.position[1], tag.position[2]])  # y, z
        else:
            self.logger.warning(f"[PL] 未知对准模式: {mode}")
            return False

        return np.linalg.norm(error) < self.params['delta_position']

    def _check_tag_valid(self, tag) -> bool:
        """检查标记是否有效且未超时。"""
        if tag is None:
            return False
        return (time.time() - tag.timestamp) <= self.params['target_timeout'] and tag.is_valid()

    def _calculate_velocity_for_tag(self, tag: ArucoTag,error: np.ndarray= None) -> Tuple[float, float, float]:
        """根据指定标记的位置计算三轴速度指令。"""
        if not self._check_tag_valid(tag):
            return 0.0, 0.0, 0.0
        # 非特殊处理
        if error is None:
            error = tag.position
        v_x = 0.6 * error[0]
        v_y = 0.6 * error[1]
        v_z = 0.5 * error[2]

        # 限制最大水平速度
        vx = np.clip(v_x, -0.6, 0.6)
        vy = np.clip(v_y, -0.6, 0.6)    

        # 限制最大垂直速度
        vz = np.clip(v_z, -0.5, 0.5)

        return vx, vy, vz

    def _calculate_velocity_for_tag_pid(self,pid_gains:dict) -> Tuple[float, float]:
        """根据指定标记的位置计算三轴速度指令。"""
        if not self._check_tag_valid(self._tag_down):
            return 0.0, 0.0
        if self._last_update_time == 0:
            self._last_update_time = time.time()
            self._previous_error = np.array([self._tag_down.position[0], self._tag_down.position[1]])
            return 0.0, 0.0
        _error = np.array([self._tag_down.position[0], self._tag_down.position[1]])
        current_time = time.time()
        dt = current_time - self._last_update_time
        if dt > 0.5 or dt <= 0:
            dt = 0.1
        # 等待合适值
        # if abs(self._integral_error[0])<10 and abs(self._integral_error[1])<10:
        #     self._integral_error = np.array([0,0])

        self._integral_error += _error * dt
        derivative = (_error - self._previous_error) / dt
        v_x = pid_gains['kp'] * _error[0]+pid_gains['ki']*self._integral_error[0]+pid_gains['kd']*derivative[0]
        v_y = pid_gains['kp'] * _error[1]+pid_gains['ki']*self._integral_error[1]+pid_gains['kd']*derivative[1]

        # 限制最大水平速度
        vx = np.clip(v_x, -0.6, 0.6)
        vy = np.clip(v_y, -0.6, 0.6)    
        self.last_error = _error
        self._last_update_time=current_time
        return vx, vy

    def _rotate_to_yaw_error(self, tag: ArucoTag, tolerance_deg=3.0):
        kp = 0.5
        max_yaw_rate_deg_s = 15.0
        min_yaw_rate_deg_s = 2.0
        mark_x = tag.orientation.apply([0, 0, -1])
        yaw_error_rad = np.arctan2(mark_x[1], mark_x[0])
        yaw_error_deg = np.degrees(yaw_error_rad)
        self._filtered_yaw_error_deg = (self.filter_alpha * yaw_error_deg) + \
                                       (1 - self.filter_alpha) * self._filtered_yaw_error_deg
        
        # 3. 使用滤波后的平滑值进行后续所有计算
        yaw_diff = (self._filtered_yaw_error_deg + 180) % 360 - 180
        # self.logger.info(f"原始误差: {yaw_error_deg:.2f}, 滤波后: {self._filtered_yaw_error_deg:.2f}")
        if abs(yaw_diff) <= tolerance_deg:
            return 0.0
        desired_yaw_rate = kp * yaw_diff
        if abs(desired_yaw_rate) > max_yaw_rate_deg_s:
            applied_yaw_rate = np.sign(desired_yaw_rate) * max_yaw_rate_deg_s
        elif abs(desired_yaw_rate) < min_yaw_rate_deg_s:
            applied_yaw_rate = np.sign(desired_yaw_rate) * min_yaw_rate_deg_s
        else:
            applied_yaw_rate = desired_yaw_rate
        return applied_yaw_rate
        
    def _switch_to_state(self, new_state: PrecisionLandState):
        """切换状态并记录日志。"""
        if self.state != new_state:
            self.logger.info(f"[PL] 状态切换: {self.state.value} -> {new_state.value}")
            self.state = new_state
