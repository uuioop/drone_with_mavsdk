import numpy as np
from typing import Optional, Tuple
from scipy.spatial.transform import Rotation as R
import time

class ArucoTag:
    """存储ArUco标记信息，包含位置、姿态和时间戳。"""
    def __init__(self, position: np.ndarray=None, orientation: Optional[R]=None, timestamp=0):
        self.position = position
        self.orientation = orientation
        self.timestamp = timestamp

    def is_valid(self) -> bool:
        """检查标记数据是否有效。"""
        return self.timestamp > 0 and self.position is not None and self.orientation is not None

    def process_tag_detection(self, position_cam: np.ndarray, orientation_cam_quat:np.ndarray, r_cb: np.ndarray):
        """处理来自图像节点的ArUco Tag检测结果。"""
        if orientation_cam_quat is None:
            self.logger.warning(f"四元数为 None，丢弃")
            return
        if np.linalg.norm(orientation_cam_quat) < 1e-6:          # 零范数
            self.logger.warning(f"四元数全零，丢弃")
            return
        tag_body_position = r_cb @ np.array(position_cam, dtype=float)
        R_orientation_in_cam = R.from_quat(orientation_cam_quat)
        R_cam_to_body = R.from_matrix(r_cb)
        tag_body_orientation = R_cam_to_body * R_orientation_in_cam
        self.position = tag_body_position
        self.orientation = tag_body_orientation
        self.timestamp = time.time()

    def is_tag_aligned(self, aligned_axis: str="xy", threshold: float = 0.1) -> bool:
        """检查标记姿态是否与期望对齐。"""
        if not self.is_valid():
            return False
        if aligned_axis == "xy":
            error = np.array([self.position[0], self.position[1]])
        elif aligned_axis == "yz":
            error = np.array([self.position[1], self.position[2]])
        elif aligned_axis == "zx":
            error = np.array([self.position[2], self.position[0]])
        else:
            print("aligned_axis must be 'xy', 'yz' or 'zx'")
            return False
        return np.linalg.norm(error) < threshold

    def _check_tag_timeout(self, timeout: float = 1.0) -> bool:
        """检查标记是否超时未更新。"""
        return time.time() - self.timestamp > timeout
