#!/usr/bin/env python3
"""
无人机控制节点主文件 - 支持板外模式导航
使用模块化结构和自定义Nav.srv服务
"""

import math
import threading
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from mavsdk import System
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import numpy as np
# 导入自定义服务类型
from drone_control.srv import Nav, Pos

# 导入核心模块
from drone_control.core import DroneState, LicensePlateProcessor, DroneStatusMonitor,  PrecisionLand
from drone_control.core.offboard_navigation import OffboardNavigationController
from drone_control.utils.utils import validate_gps_coordinates



class DroneControlNode(Node):
    """无人机控制节点主类 - 支持板外模式导航和ArUco标记跟踪"""
    def __init__(self):
        super().__init__('drone_control_node')
        self.get_logger().info("无人机控制节点启动中...")
        
        # 初始化组件
        # self.drone = System(mavsdk_server_address='localhost', port=50051)
        self.drone = System()
        self.drone_state = DroneState()
        self.system_address = "udp://0.0.0.0:14540"
        
        # 板外模式状态
        self.offboard_active = False

        # 初始化号牌识别结果
        self.license_plate_result = False
        
        # 初始化控制器
        self.precision_land_controller = PrecisionLand(
            self.drone, 
            self.get_logger(), 
            self.drone_state
        )

        self.offboard_navigation_controller = OffboardNavigationController(
            self.drone, self.get_logger(), self.drone_state, self.license_plate_result
        )
        self.license_plate_processor = LicensePlateProcessor(self.get_logger())
        
        self.takeoff_alt=2.0

        # 创建独立事件循环
        self.loop = asyncio.new_event_loop()
        
        # 设置ROS接口
        self._setup_services()
        self._setup_subscriptions()
        
        # 初始化状态监控器
        self.status_monitor = DroneStatusMonitor(self.drone, self.drone_state, self.get_logger())
        self.status_monitor.set_position_publisher(self.position_pub)
        
        # 启动MAVSDK事件循环线程
        self._start_event_loop()


        self.get_logger().info("节点初始化完成")

    def _setup_services(self):
        """初始化ROS服务"""
        self.navigation_srv = self.create_service(
            Pos, 'drone/navigation', self._sync_navigation_callback)
        # self.absolute_navigation_srv = self.create_service(
        #     Nav, 'drone/absolute_navigation', self._sync_absolute_navigation_callback)

    def _setup_subscriptions(self):
        """设置订阅和发布"""
        # 创建订阅者和发布者
        qos = rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        
        # 发布
        self.position_pub = self.create_publisher(String, 'drone/position', 10)

        # 精准降落订阅
        self.target_pose_sub = self.create_subscription(
            PoseStamped, '/target_pose', self._target_pose_callback, qos)

    def _start_event_loop(self):
        """启动MAVSDK事件循环线程"""
        def run_loop():
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(
                asyncio.gather(
                    self._connect_to_drone(),
                    self.status_monitor.start_monitoring()
                )
            )
        self.loop_thread = threading.Thread(target=run_loop, daemon=True)
        self.loop_thread.start()

        # # 创建精准降落主循环定时器
        self.precision_land_timer = self.create_timer(0.2, self._precision_land_update_callback) # 10Hz

    def _target_pose_callback(self, msg):
        """目标位姿回调函数"""
        # 检查无人机状态是否就绪
        if self.drone_state.current_position_ned is None or self.drone_state.attitude_quaternion is None:
            self.get_logger().warn("无人机位置或姿态数据尚未就绪，无法更新目标位姿。")
            return
        
        if self.drone_state.search_started or True:
            # 调用精准降落控制器的方法，传入所有必需的数据
            self.precision_land_controller._get_tag_world(
            position=np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float), 
            orientation=np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype=float)
            )

    def _precision_land_update_callback(self):
        """精准降落状态机主更新循环。"""
        if self.drone_state.search_started:
            asyncio.run_coroutine_threadsafe(
            self.precision_land_controller.update(), self.loop
        )

    async def _connect_to_drone(self):
        """连接到无人机"""       
        self.get_logger().info(f"尝试连接到无人机: {self.system_address}")
        self.get_logger().info("等待无人机连接...")
        await self.drone.connect(system_address=self.system_address)
        # await self.drone.connect()
        # 获取初始位置
        await self.status_monitor._get_home_position()
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("成功连接到无人机!")
                self.drone_state.update_connection(True)
                break

    async def _navigation_callback(self, request):
        """处理相对位置导航请求 - 使用NED坐标系"""
        response = Pos.Response()

        if self.offboard_active:
            response.success = False
            response.message = "已有板外导航任务在执行中。"
            return response

        # 检查解锁状态
        if not await self.status_monitor._check_preflight():
            response.success = False
            response.message = "无人机起飞状态不满足"
            return response
        
        try:
            # 获取当前位置
            current_position = self.drone_state.current_position
            if current_position is None:
                response.success = False
                response.message = "无法获取当前位置，请等待GPS信号稳定"
                self.get_logger().error("当前位置为空，请检查GPS连接")
                return response
            # 从Pos.srv消息中获取NED相对偏移
            self.drone_state.update_target_position(request.x,request.y,request.z)           
            self.drone_state.navigation_mode = "RELATIVE" if request.is_ned else "ABSOLUTE"

            self.get_logger().info(f"[导航] 模式: {self.drone_state.navigation_mode}")

            # 使用板外导航控制器进行相对位置导航
            self.offboard_active = True
            await self.offboard_navigation_controller.navigate_to_position()
            # self.precision_land_controller.start_precision_land()
            self.offboard_active = False
            response.success = True
            response.message = f"导航完成"
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"导航错误: {str(e)}"
            self.get_logger().error(response.message)
        return response

    # ========= 同步桥接方法 =========

    def _sync_navigation_callback(self, request, response):
        """导航的同步回调函数"""
        try:
            future = asyncio.run_coroutine_threadsafe(
                self._navigation_callback(request), self.loop)
            result = future.result(timeout=60)  # 相对导航需要更长时间
            
            # 确保响应对象被正确设置
            if result is not None:
                response.success = result.success
                response.message = result.message
            else:
                response.success = False
                response.message = "导航回调返回空结果"
            if response.success:
                pass
            return response
        except Exception as e:
            response.success = False
            response.message = f"导航超时: {str(e)}，应急降落"        
           
            asyncio.run_coroutine_threadsafe(self.drone.action.land(), self.loop)
            return response

    def destroy_node(self):
        """优雅关闭节点"""
        self.get_logger().info("正在关闭无人机控制节点...")
        
        try:
            # 停止事件循环
            if hasattr(self, 'loop') and self.loop:
                self.loop.call_soon_threadsafe(self.loop.stop)
            
            # 等待线程结束
            if hasattr(self, 'loop_thread') and self.loop_thread:
                self.loop_thread.join(timeout=5)
                
            # 断开MAVSDK连接
            if hasattr(self, 'drone') and self.drone:
                asyncio.run_coroutine_threadsafe(
                    self.drone.disconnect(), self.loop)
                    
        except Exception as e:
            self.get_logger().error(f"关闭节点时出错: {e}")
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DroneControlNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()