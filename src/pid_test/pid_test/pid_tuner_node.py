#!/usr/bin/env python3
from collections import deque
import rclpy
from rclpy.node import Node
import time
import numpy as np
import threading
import asyncio
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# 假设这些核心模块与 drone_control_node.py 中的定义一致
# 如果它们在不同的包中，请确保这里的导入路径是正确的
from pid_test.core import DroneState, DroneStatusMonitor

class ArucoTag:
    """存储ArUco标记信息，包含位置、姿态和时间戳。"""
    def __init__(self, position: np.ndarray=None, orientation: R=None, timestamp=0):
        self.position = position
        self.orientation = orientation
        self.timestamp = timestamp

    def is_valid(self) -> bool:
        return self.timestamp > 0 and self.position is not None and self.orientation is not None

class PIDController:
    """一个简单的PID控制器"""
    def __init__(self, node, drone_state: DroneState, pid_gains,max_speed):
        self.node = node
        self.drone_state = drone_state
        self._previous_error = np.zeros(3)
        self._integral_error = np.zeros(3)
        self._last_update_time = 0
        self.pid_gains = pid_gains
        self.max_speed = max_speed
        self._tag = ArucoTag()
        # 新增：初始化滤波后的偏航误差
        self._filtered_yaw_error_deg = 0.0
        self.filter_alpha = 0.1  # 新增：定义滤波系数 alpha
        self.yaw_error_history = deque(maxlen=5) # 存储最近5个值

    def process_tag_detection(self, position_cam, orientation_cam_quat):
        if orientation_cam_quat is None:
            self.node.get_logger().warning(f"[PL] 四元数为 None，丢弃")
            return
        quat = np.asarray(orientation_cam_quat, dtype=float)
        if np.linalg.norm(quat) < 1e-6:
            self.node.get_logger().warning(f"[PL]  四元数全零，丢弃")
            return
        # error= np.array([-0.1,0.05,0])
        R_cb = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        tag_body_position = R_cb @ np.array(position_cam, dtype=float)
        R_cam_to_body = R.from_matrix(R_cb)
        R_orientation_in_cam = R.from_quat(orientation_cam_quat)
        tag_body_orientation = R_cam_to_body * R_orientation_in_cam
        self._tag = ArucoTag(position=tag_body_position, orientation=tag_body_orientation, timestamp=time.time())

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
        # 标记正对无人机——取负（标记使用相机的坐标系Z轴)
        mark_x = self._tag.orientation.apply([0, 0, -1])
        yaw_error_rad = np.arctan2(mark_x[1], mark_x[0])
        yaw_rate = self._rotate_to_yaw_error(yaw_error_rad)
        
        self._previous_error = error
        self._last_update_time = current_time
        return 0.0, vy, 0.0, yaw_rate

    def _rotate_to_yaw_error(self, yaw_error_rad, tolerance_deg=3.0):
        kp = 0.5
        max_yaw_rate_deg_s = 15.0
        min_yaw_rate_deg_s = 2.0
        yaw_error_deg = np.degrees(yaw_error_rad)
        self._filtered_yaw_error_deg = (self.filter_alpha * yaw_error_deg) + \
                                       (1 - self.filter_alpha) * self._filtered_yaw_error_deg
                                       
        # 3. 使用滤波后的平滑值进行后续所有计算
        yaw_diff = (self._filtered_yaw_error_deg + 180) % 360 - 180
        self.node.get_logger().info(f"原始误差: {yaw_error_deg:.2f}, 滤波后: {self._filtered_yaw_error_deg:.2f}")
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

class PIDTunerNode(Node):
    def __init__(self):
        super().__init__('pid_tuner_node')
        self.get_logger().info("PID Tuner 节点启动中...")

        # 1. 初始化核心组件
        self.drone = System()
        self.drone_state = DroneState()
        # 核心修改：添加状态监控器
        self.status_monitor = DroneStatusMonitor(self.drone, self.drone_state, self.get_logger())

        # 聲明參數並設定預設值
        self.declare_parameter('kp', 1.0)  # Kp for y-axis
        self.declare_parameter('ki', 0.0)  # Ki for y-axis
        self.declare_parameter('kd', 0.0)  # Kd for y-axis
        self.declare_parameter('max_speed', 1.5)

        # 在您的控制循環中，獲取這些參數的當前值
        kp = self.get_parameter('kp').get_parameter_value().double_value
        ki = self.get_parameter('ki').get_parameter_value().double_value
        kd = self.get_parameter('kd').get_parameter_value().double_value
        max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        pid_gains = {
            'p': kp,
            'i': ki,
            'd': kd
        }
        self.pid_controller = PIDController(self, self.drone_state, pid_gains,max_speed)
        self.get_logger().info(f"kp: {kp}, ki: {ki}, kd: {kd}, max_speed: {max_speed}")
        # 2. 状态机
        self.state = "INITIALIZING"  # 初始状态
        self.ascent_speed = -1.2  # m/s, MAVSDK FRD 坐标系，向上为负

        # 3. 设置ROS接口
        qos = rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            PoseStamped, '/infra_tracker/target_pose', self.pose_callback, qos)

        # 4. 核心修改：创建并启动独立的 asyncio 事件循环线程
        self.loop = asyncio.new_event_loop()
        self._start_event_loop()
        self.get_logger().info("MAVSDK 事件循环线程已启动。")
        self.get_logger().info("节点初始化完成。")

    def _start_event_loop(self):
        """启动MAVSDK事件循环线程"""
        def run_loop():
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self._run_all())

        self.loop_thread = threading.Thread(target=run_loop, daemon=True)
        self.loop_thread.start()
        # 定时器用于周期性地触发PID计算和指令发送
        self.control_timer = self.create_timer(0.1, self.control_timer_callback)

    async def _run_all(self):
        """先连接无人机，再开始持续监控"""
        await self._setup_and_start_offboard()
        await self.status_monitor.start_monitoring()

    async def _setup_and_start_offboard(self):
        """执行一次性的起飞和模式切换任务。"""
        self.get_logger().info("正在连接无人机...")
        await self.drone.connect(system_address="udp://:14540")

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("无人机已连接！")
                break
        self.get_logger().info("等待无人机GPS位置稳定...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("GPS位置已稳定。")
                break

        self.get_logger().info("解锁无人机...")
        await self.drone.action.arm()

        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        self.get_logger().info("启动 Offboard 模式。")
        try:
            await self.drone.offboard.start()
            self.state = "ASCENDING" # 进入Offboard模式成功后，切换到上升状态
            self.get_logger().info(f"Offboard 模式已启动。切换状态到: {self.state}")
        except OffboardError as error:
            self.get_logger().error(f"启动 Offboard 模式失败: {error}")
            await self.drone.action.disarm()
            self.state = "ERROR"
            return

    async def _send_velocity_command(self, vx, vy, vz, yaw_speed):
        """一个专门用于发送速度指令的协程。"""
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, yaw_speed))

    def pose_callback(self, msg: PoseStamped):
        """ROS订阅回调函数，处理来自图像节点的位姿信息。"""
        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.pid_controller.process_tag_detection(position, orientation)
        
        if self.state == "ASCENDING" and self.pid_controller._tag.is_valid():
            # 在机体坐标系下，相机朝下，Z轴指向地面，所以tag_z_position为正值
            tag_z_position_body = self.pid_controller._tag.position[2]
            if abs(tag_z_position_body) < 0.8:
                self.get_logger().info(f"目标进入追踪范围 (Z轴距离: {tag_z_position_body:.2f} m)。切换状态到 TRACKING。")
                self.state = "TRACKING"

    def control_timer_callback(self):
        """
        ROS定时器回调函数。此函数在ROS线程中执行。
        它负责计算控制指令，然后将MAVSDK的发送任务提交给asyncio线程。
        """
        vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0

        if self.state == "ASCENDING":
            vx, vy, vz, yaw_rate = 0.0, 0.0, self.ascent_speed, 0.0

        elif self.state == "TRACKING":
            vx, vy, vz, yaw_rate = self.pid_controller.update()

        else: # 包括 INITIALIZING, ERROR 等状态
            vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0
            # 添加这行日志来了解当前状态

        # 核心修改：安全地将异步任务提交到 MAVSDK 的事件循环线程
        asyncio.run_coroutine_threadsafe(
            self._send_velocity_command(vx, vy, vz, yaw_rate), 
            self.loop
        )

    def destroy_node(self):
        """节点关闭时调用的清理函数。"""
        self.get_logger().info("正在关闭节点...")
        if self.loop.is_running():
            # 安全地停止循环
            self.loop.call_soon_threadsafe(self.loop.stop)
        # 等待线程结束
        self.loop_thread.join(timeout=2)
        super().destroy_node()


def main(args=None):
    """标准的ROS 2节点入口函数。"""
    rclpy.init(args=args)
    pid_tuner_node = PIDTunerNode()
    
    # 使用rclpy.spin()，它会自动处理所有ROS回调，比手动循环更高效
    try:
        rclpy.spin(pid_tuner_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 确保节点被正确销毁
        pid_tuner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()