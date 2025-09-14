from drone_control.utils import ArucoTag
import numpy as np
import time

class PIDController:
    """一个简单的PID控制器"""
    def __init__(self, drone_state, pid_gains,max_speed):
        self.drone_state = drone_state
        self._previous_error = np.zeros(3)
        self._integral_error = np.zeros(3)
        self._last_update_time = 0
        self.pid_gains = pid_gains
        self.max_speed = max_speed
        self._r_cb=np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        self._tag = ArucoTag()

    def update(self):
        if not self._tag.is_valid():
            return 0.0, 0.0, 0.0, 0.0    

        if self._last_update_time == 0:
            self._last_update_time = time.time()
            self._previous_error = np.array([self._tag.position[0] - 2, self._tag.position[1], self._tag.position[2]])
            return 0.0, 0.0, 0.0, 0.0
        
        error = np.array([self._tag.position[0] - 2, self._tag.position[1], self._tag.position[2]])
        current_time = time.time()
        dt = current_time - self._last_update_time
        if dt > 0.5 or dt <= 0:
            dt = 0.1

        self._integral_error += error * dt
        derivative = (error - self._previous_error) / dt

        output = (self.pid_gains['p'] * error[0]+ self.pid_gains['i'] * self._integral_error[0]+ self.pid_gains['d'] * derivative[0],
                  self.pid_gains['p'] * error[1]+ self.pid_gains['i'] * self._integral_error[1]+ self.pid_gains['d'] * derivative[1],
                  self.pid_gains['p'] * error[2]+ self.pid_gains['i'] * self._integral_error[2]+ self.pid_gains['d'] * derivative[2])

        vx = np.clip(output[0], -self.max_speed, self.max_speed)
        vy = np.clip(output[1], -self.max_speed, self.max_speed)
        
        mark_x = self._tag.orientation.apply([1, 0, 0])
        yaw_error = np.arctan2(mark_x[1], mark_x[0])
        yaw_rate = self._rotate_to_yaw_error(yaw_error)
        
        self.node.get_logger().info(f"current:{self.drone_state.attitude_euler.yaw_deg}, yaw_error:{yaw_error}", throttle_duration_sec=1)
        
        self._previous_error = error
        self._last_update_time = current_time
        return 0.0, vy, 0.0, yaw_rate

    def _rotate_to_yaw_error(self, yaw_error, tolerance_deg=2.0):
        kp = 0.8
        max_yaw_rate_deg_s = 15.0
        min_yaw_rate_deg_s = 2.0
        yaw_diff = (yaw_error * 180 / np.pi + 180) % 360 - 180
        
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
