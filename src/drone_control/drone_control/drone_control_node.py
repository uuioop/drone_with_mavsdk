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
# import tf2_ros
# import tf_transformations
# from geometry_msgs.msg import TransformStamped
from mavsdk import System
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import numpy as np
# 导入自定义服务类型
from drone_control.srv import Pos

# 导入核心模块
from drone_control.core import DroneState, DroneStatusMonitor 
from drone_control.services import HybridNavigationController, PrecisionLand, OffboardNavigationController,ConfirmPositionController,MavsdkController
from drone_control.SQLite import LicensePlateProcessor
from drone_control.utils.utils import validate_gps_coordinates,observe_is_in_air

class DroneControlNode(Node):
    """无人机控制节点主类 - 支持板外模式导航和ArUco标记跟踪"""
    def __init__(self):
        super().__init__('drone_control_node')
        self.get_logger().info("无人机控制节点启动中...")
        # 初始化组件
        self.drone = System()
        self.drone_state = DroneState()
        self.mavsdk_controller = MavsdkController(self.drone, self.get_logger(),self.drone_state)
        self.system_address = "udpin://0.0.0.0:14540"
        # self.system_address="serial:///dev/ttyACM0:115200"
        # tf2
        # self.tf_buffer   = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # 初始化号牌数据库
        self.license_plate_processor = LicensePlateProcessor(self.get_logger())
        # 板外模式状态
        self.offboard_active = False

        # 初始化号牌识别结果
        self.license_plate_result = False
        
        # 初始化控制器
        self.precision_land_controller = PrecisionLand(
            self.mavsdk_controller, 
            self.get_logger(), 
            self.drone_state
        )

        self.offboard_navigation_controller = OffboardNavigationController(
            self.drone, self.get_logger(), self.drone_state
        )
        self.license_plate_processor = LicensePlateProcessor(self.get_logger())

        self.hybrid_navigation_controller = HybridNavigationController(
            self.drone, self.get_logger(), self.drone_state
        )
        self.confirm_position_controller = ConfirmPositionController(
            self.drone, self.get_logger(), self.drone_state, self.license_plate_processor
        )

        # 创建独立事件循环
        self.loop = asyncio.new_event_loop()
        
        # 设置ROS接口
        self._setup_services()
        self._setup_subscriptions()
        
        # 初始化状态监控器
        self.status_monitor = DroneStatusMonitor(self.drone, self.drone_state, self.get_logger())
        self.status_monitor.set_info_publisher(self.drone_info_publisher)
        
        # 启动MAVSDK事件循环线程
        self._start_event_loop()


        self.get_logger().info("节点初始化完成")

    def _setup_services(self):
        """初始化ROS服务"""
        self.navigation_srv = self.create_service(
            Pos, 'drone/navigation', self._sync_navigation_callback)
        self.hybrid_navigation_srv = self.create_service(
            Pos, 'drone/hybrid_navigation', self._sync_hybrid_navigation_callback)

    def _setup_subscriptions(self):
        """设置订阅和发布"""
        qos = rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        
        # 发布
        self.drone_info_publisher = self.create_publisher(String, 'drone/info', 10)

        # 订阅
        self.license_plate_sub = self.create_subscription(
            String, '/license_detection_result', self._license_plate_callback, qos)
        # 精准降落订阅
        self.target_pose_sub_front = self.create_subscription(
            PoseStamped, '/mono_tracker/target_pose', lambda msg: self._target_pose_callback(msg, 'down'), qos)
        self.target_pose_sub_down = self.create_subscription(
            PoseStamped, '/infra_tracker/target_pose', lambda msg: self._target_pose_callback(msg, 'front'), qos)
    def _start_event_loop(self):
        """启动MAVSDK事件循环线程"""
        def run_loop():
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self._run_all())

        self.loop_thread = threading.Thread(target=run_loop, daemon=True)
        self.loop_thread.start()

        # 创建精准降落主循环定时器
        self.precision_land_timer = self.create_timer(0.1, self._precision_land_update_callback) # 10Hz
        self.confirm_position_timer = self.create_timer(0.2, self._confirm_position_update_callback) # 5Hz

    async def _run_all(self):
        """先连接无人机，再开始持续监控"""
        await self.mavsdk_controller._connect_to_drone(self.system_address)
        await self.status_monitor.start_monitoring()
    # def _start_event_loop(self):
    #     """启动MAVSDK事件循环线程"""
    #     self.loop.run_until_complete(
    #         self.mavsdk_controller._connect_to_drone(self.system_address)
    #     )
    #     def run_loop():
    #         asyncio.set_event_loop(self.loop)
    #         self.loop.run_until_complete(
    #             self.status_monitor.start_monitoring()
    #         )
    #     self.loop_thread = threading.Thread(target=run_loop, daemon=True)
    #     self.loop_thread.start()


        
    def _license_plate_callback(self, msg: String):
        """号牌识别回调函数"""
        self.license_plate_result = msg.data
    def _target_pose_callback(self, msg: PoseStamped, source: str):
        """目标位姿回调函数"""
        # 检查是否开始搜索      
        if self.drone_state.confirm_started and source=='front':
            self.get_logger().info("确认地址，检测到标记")
            self.drone_state.tag_detected=True
        if self.drone_state.search_started:
            # 根据来源调用不同的处理函数
            if source == 'down':
                # 调用精准降落控制器的方法，传入所有必需的数据
                self.precision_land_controller.process_tag_detection(
                position_cam=np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float), 
                orientation_cam_quat=np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype=float),
                source=source
                )       
            else:
                self.precision_land_controller.process_tag_detection(
                position_cam=np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float), 
                orientation_cam_quat=np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype=float),
                source=source
                )       

    def _precision_land_update_callback(self):
        """精准降落状态机主更新循环。"""
        if self.drone_state.search_started:
            asyncio.run_coroutine_threadsafe(
            self.precision_land_controller.update(), self.loop
        )

    def _confirm_position_update_callback(self):
        """号牌确认状态机主更新循环。"""
        if self.drone_state.confirm_started:
            asyncio.run_coroutine_threadsafe(
            self.confirm_position_controller.update(), self.loop
        )
    # async def _connect_to_drone(self):
    #     """连接到无人机"""       
    #     self.get_logger().info(f"尝试连接到无人机: {self.system_address}")
    #     self.get_logger().info("等待无人机连接...")
    #     await self.drone.connect(system_address=self.system_address)
    #     # 获取初始位置
    #     await self.status_monitor._get_home_position()
    #     async for state in self.drone.core.connection_state():
    #         if state.is_connected:
    #             self.get_logger().info("成功连接到无人机!")
    #             self.drone_state.update_connection(True)
    #             break
    
    async def _hybrid_navigation_callback(self, request):
        """处理混合导航请求"""
        response = Pos.Response()
        if not await self.status_monitor._check_preflight():
            response.success = False
            response.message = "无人机起飞状态不满足"
            return response
        
        try:
            if not validate_gps_coordinates(request.x, request.y, request.z):
                response.success = False
                response.message = "计算后的坐标参数无效"
                return response
            self.drone_state.update_target_position(request.x,request.y,request.z,request.w)
            await self.hybrid_navigation_controller.navigate_to_position()
            
            response.success = True
            response.message = f"混合导航完成"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"混合导航错误: {str(e)}"
            self.drone_state.reset_target_position()
            await observe_is_in_air(self.drone, self.get_logger())
            self.get_logger().error(response.message)
        return response

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
            current_position_ned = self.drone_state.current_position_ned
            if current_position_ned is None:
                response.success = False
                response.message = "无法获取当前位置，请等待GPS信号稳定"
                self.get_logger().error("当前位置为空，请检查GPS连接")
                return response
            # 从Pos.srv消息中获取NED相对偏移
            if request.is_ned:
                self.drone_state.update_target_position(request.x+current_position_ned.north_m,
                request.y+current_position_ned.east_m,
                request.z+current_position_ned.down_m,request.w)           
            else:
                self.drone_state.update_target_position(request.x,request.y,request.z,request.w)           
            self.drone_state.navigation_mode = "RELATIVE" if request.is_ned else "ABSOLUTE"

            self.get_logger().info(f"[导航] 模式: {self.drone_state.navigation_mode}")

            # 使用板外导航控制器进行相对位置导航
            await self.offboard_navigation_controller.navigate_to_position()
            response.success = True
            response.message = f"导航完成"
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"导航错误: {str(e)}"
            self.drone_state.reset_target_position()
            await observe_is_in_air(self.drone, self.get_logger())
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
            return response
        except Exception as e:
            response.success = False
            response.message = f"导航超时: {str(e)}，应急降落"                   
            return response

    def _sync_hybrid_navigation_callback(self, request, response):
        """混合导航的同步回调函数"""
        try:
            future = asyncio.run_coroutine_threadsafe(
                self._hybrid_navigation_callback(request), self.loop)
            result = future.result(timeout=60)  # 相对导航需要更长时间
            
            # 确保响应对象被正确设置
            if result is not None:
                response.success = result.success
                response.message = result.message
            else:
                response.success = False
                response.message = "导航回调返回空结果"
            return response
        except Exception as e:
            response.success = False
            response.message = f"导航超时: {str(e)}，应急降落"        
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