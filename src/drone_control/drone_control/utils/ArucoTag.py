import numpy as np
from typing import Optional
from scipy.spatial.transform import Rotation as R

class ArucoTag:
    """存储ArUco标记信息，包含位置、姿态和时间戳。"""
    def __init__(self, position: np.ndarray=None, orientation: Optional[R]=None, timestamp=0):
        self.position = position
        self.orientation = orientation
        self.timestamp = timestamp

    def is_valid(self) -> bool:
        """检查标记数据是否有效。"""
        return self.timestamp > 0 and self.position is not None and self.orientation is not None
