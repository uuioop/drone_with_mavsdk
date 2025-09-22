# drone_control/utils/tag_processor.py
import time
import numpy as np
from typing import Tuple
from scipy.spatial.transform import Rotation as R

from .ArucoTag import ArucoTag # 假設 ArucoTag 在這裡

class ArucoTagProcessor:
    """
    一個專門用於處理 ArUco Tag 相關計算的輔助類別。
    它封裝了 PID 控制、對準檢查、速度計算等通用邏輯。
    """
    def __init__(self, config: dict, logger):
        """
        初始化處理器。
        :param config: 包含設定參數的字典 (例如 PID 增益, 容忍度等)
        """
        self.config = config
        self.logger = logger
        self.tag = ArucoTag() 
        # 该标记是否有时效
        self.is_valid_now=False
        # 用于追踪状态变化的“记忆”属性
        self._was_previously_valid = False
        # 对准状态计数器
        self._align_timer=0
        # PID 控制所需的內部狀態
        # self._last_update_time = 0.0
        # self._previous_error = np.array([0.0, 0.0])
        self._integral_error = np.zeros(3)
        self._filtered_yaw_error_deg = 0.0
        self.filter_alpha = 0.3 # 濾波器 alpha 值

    def update_and_log_validity(self):
        tag_name = self.config.get('tag_name', "未知标记")
        # 检查标记是否有效且未超时
        self.is_valid_now = self.tag.is_valid() and not self._check_tag_timeout()
        
        if not self.is_valid_now and self._was_previously_valid:
            self.logger.warning(f"[PL] {tag_name} 丢失")
        elif self.is_valid_now and not self._was_previously_valid:
            self.logger.info(f"[PL] {tag_name} 重新获取")
        
        self._was_previously_valid = self.is_valid_now
    
    def is_valid(self) -> bool:
        """
        检查当前标记是否有时效。
        :return: 如果标记有时效则返回 True，否则返回 False
        """
        return self.is_valid_now
    def process_tag_detection(self, position_cam, orientation_cam_quat):
        """
        將相機座標系下的偵測結果轉換為主體座標系下的 ArucoTag 物件。
        """
        try:
            # 1. 使用一个小的容差来检查范数是否接近于零
            if np.linalg.norm(orientation_cam_quat) < 1e-6:
                # 只有在tag从有效变为无效时才记录警告，避免日志刷屏
                if self.tag.is_valid():
                    self.logger.warning("[TagProcessor] 收到一个范数为零的无效四元数，已忽略。")
                self.tag = ArucoTag() # 重置tag为无效状态
                return # 提前返回

            # 2. 只有在检查通过后才进行转换
            r_cb = self.config.get('rotation_matrix', np.eye(3))
            tag_body_position = r_cb @ np.array(position_cam, dtype=float)
            
            R_orientation_in_cam = R.from_quat(orientation_cam_quat)
            R_cam_to_body = R.from_matrix(r_cb)
            tag_body_orientation = R_cam_to_body * R_orientation_in_cam
            self.tag = ArucoTag(position=tag_body_position, orientation=tag_body_orientation, timestamp=time.time())

        except ValueError as e:
            # 3. --- 这是关键的修复 ---
            #    使用 ROS2 兼容的方式记录异常信息。
            #    我们直接将异常对象 e 转换成字符串并附加到日志消息中。
            self.logger.error(f"[TagProcessor] 处理Tag时发生数学错误: {e}")
            self.logger.debug(f"  > 原始输入 position_cam: {position_cam}")
            self.logger.debug(f"  > 原始输入 orientation_cam_quat: {orientation_cam_quat}")
            self.tag = ArucoTag() # 发生任何错误都将tag置为无效

    def is_aligned(self) -> bool:
            """通用对准检查，使用配置中定义的轴"""
            # 从配置中获取需要检查的轴
            axes = self.config.get('alignment_axes', ['x', 'y'])
            error_vector = []
            if 'x' in axes:
                error_vector.append(self.tag.position[0])
            if 'y' in axes:
                error_vector.append(self.tag.position[1])
            if 'z' in axes:
                error_vector.append(self.tag.position[2])
                
            if np.linalg.norm(error_vector) < self.config.get('aligned_tolerance', 0.2):
                self._align_timer+=1
            else:
                self._align_timer=0
            return self._align_timer>=15

    def _check_tag_timeout(self) -> bool:
        """检查标记是否超时未更新。"""
        return time.time() - self.tag.timestamp > self.config['target_timeout'] 
    
    def calculate_velocity_command(self,is_yaw:bool = True) -> Tuple[float, float, float,float]:
        """
        通用的速度计算方法，它会考虑目标偏移量。
        """
        # 从配置中获取目标偏移量
        target_offset = self.config.get('target_offset', np.array([0.0, 0.0, 0.0]))
        
        # 核心：误差 = 当前位置 - 期望的目标位置
        error = self.tag.position - target_offset
        self._integral_error += error
        self._integral_error = np.clip(self._integral_error, -2, 2)
        # 这里的 kp, max_speed 等也应该从 self.config 中获取
        kp = self.config.get('kp', 0.5)
        ki = self.config.get('ki', 0.0)
        max_speed = self.config.get('max_speed', 0.6)

        # 根据误差计算速度 (这里可以用简单的P控制器或完整的PID)
        vx = kp * error[0] + ki * self._integral_error[0]
        vy = kp * error[1] + ki * self._integral_error[1]
        vz = kp * error[2] + ki * self._integral_error[2]
        
        vx=np.clip(vx,-max_speed,max_speed)
        vy=np.clip(vy,-max_speed,max_speed)
        vz=np.clip(vz,-max_speed,max_speed)
        yaw_rate = self._calculate_yaw_rate() if is_yaw else 0.0

        return vx, vy, vz, yaw_rate

    def _calculate_yaw_rate(self):
        kp = self.config.get('kp', 0.5)
        max_yaw_rate_deg_s = self.config.get('max_yaw_rate_deg_s', 15.0)
        min_yaw_rate_deg_s = self.config.get('min_yaw_rate_deg_s', 2.0)
        mark_x = self.tag.orientation.apply([0, 0, -1])
        yaw_error_rad = np.arctan2(mark_x[1], mark_x[0])
        yaw_error_deg = np.degrees(yaw_error_rad)
        self._filtered_yaw_error_deg = (self.filter_alpha * yaw_error_deg) + \
                                    (1 - self.filter_alpha) * self._filtered_yaw_error_deg
    
        # 3. 使用滤波后的平滑值进行后续所有计算
        yaw_diff = (self._filtered_yaw_error_deg + 180) % 360 - 180
        # self.logger.info(f"原始误差: {yaw_error_deg:.2f}, 滤波后: {self._filtered_yaw_error_deg:.2f}")
        if abs(yaw_diff) <= self.config.get('yaw_tolerance_deg', 0.3):
            return 0.0
        desired_yaw_rate = kp * yaw_diff
        if abs(desired_yaw_rate) > max_yaw_rate_deg_s:
            applied_yaw_rate = np.sign(desired_yaw_rate) * max_yaw_rate_deg_s
        elif abs(desired_yaw_rate) < min_yaw_rate_deg_s:
            applied_yaw_rate = np.sign(desired_yaw_rate) * min_yaw_rate_deg_s
        else:
            applied_yaw_rate = desired_yaw_rate
        return applied_yaw_rate
    
    def update_config(self,config:dict):
        self.config=config