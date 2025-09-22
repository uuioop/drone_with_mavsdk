#!/usr/bin/env python3
"""
无人机控制节点主文件 - 支持板外模式导航
使用模块化结构和自定义Nav.srv服务
"""
import time
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
from drone_control_interfaces.srv import Pos

# 导入核心模块
from drone_control.core import DroneState, DroneStatusMonitor,MavsdkController
from drone_control.precision_land import PrecisionLandController
from drone_control.confirm_license import ConfirmLicenseController
from drone_control.SQLite import LicensePlateProcessor
from drone_control.utils import validate_gps_coordinates

class DroneControlNode(Node):
    """无人机控制节点主类 - 支持板外模式导航和ArUco标记跟踪"""
    def __init__(self):
        super().__init__('drone_control_node')
        self.get_logger().info("无人机控制节点启动中...")
        # 初始化组件
        self.drone = System()
        self.drone_state = DroneState()
        self.mavsdk_controller = MavsdkController(self)
        self.system_address = "udpin://0.0.0.0:14540"
        # self.system_address="serial:///dev/ttyACM0:115200"
        # 初始化号牌数据库
        self.license_plate_processor = LicensePlateProcessor(self.get_logger())
        # 板外模式状态
        self.offboard_active = False

        # 初始化号牌识别结果
        self.license_plate_result = False
        
        # 聲明參數並設定預設值
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('dist', 0.02)

        pid_config={}
        pid_config['output_amplitude'] = self.get_parameter('speed').get_parameter_value().double_value
        pid_config['input_tolerance'] = self.get_parameter('dist').get_parameter_value().double_value

        # 初始化控制器
        self.precision_land_controller = PrecisionLandController(self,pid_config)
        self.confirm_license_controller = ConfirmLicenseController(self)

        # self.offboard_navigation_controller = OffboardNavigationController(
        #     self.mavsdk_controller, self.get_logger(), self.drone_state
        # )
        # self.license_plate_processor = LicensePlateProcessor(self.get_logger())

        # self.hybrid_navigation_controller = HybridNavigationController(
        #     self.mavsdk_controller, self.get_logger(), self.drone_state
        # )

        # 创建独立事件循环
        self.loop = asyncio.new_event_loop()
        
        # 设置ROS接口
        self._setup_services()
        self._setup_subscriptions()
        
        # 初始化状态监控器
        self.status_monitor = DroneStatusMonitor(self)
        self.status_monitor.set_info_publisher(self.drone_info_publisher)
        
        # 启动MAVSDK事件循环线程
        self._start_event_loop()
        # self.license_plate_processor.add_license_plate_to_database("闽DA01010231")
        # self.license_plate_processor.set_target_license_plate("闽DA01010231")

        self.get_logger().info("节点初始化完成")

    def _setup_services(self):
        """初始化ROS服务"""
        # self.navigation_srv = self.create_service(
        #     Pos, 'drone/navigation', self._sync_navigation_callback)
        # self.hybrid_navigation_srv = self.create_service(
        #     Pos, 'drone/hybrid_navigation', self._sync_hybrid_navigation_callback)
        self.autotune_x_srv = self.create_service(
            Trigger, 'drone/start_autotune_x', self._sync_autotune_x_callback)
        self.autotune_y_srv = self.create_service(
            Trigger, 'drone/start_autotune_y', self._sync_autotune_y_callback)
        self.precision_srv = self.create_service(
            Trigger, 'drone/precision_land', self._sync_precision_land_callback)

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

        # 创建状态机主循环定时器
        self.fsm_update_timer = self.create_timer(0.1, self._fsm_update_callback) # 10Hz

    async def _run_all(self):
        """先连接无人机，再开始持续监控"""
        try:
            await self.mavsdk_controller._connect_to_drone(self.system_address)
        except Exception as e:
            self.get_logger().error(f"连接到无人机失败: {e}")
        # await self.precision_land_controller.start_autotune('x')
        await self.status_monitor.start_monitoring()

    def _license_plate_callback(self, msg: String):
        """号牌识别回调函数"""
        self.license_plate_result = msg.data
    def _target_pose_callback(self, msg: PoseStamped, source: str):
        """目标位姿回调函数"""
        # 检查是否开始搜索      
        # if self.drone_state.confirm_started and source=='front':
        #     self.get_logger().info("确认地址，检测到标记")
        #     self.drone_state.tag_detected=True
        # if self.drone_state.search_started:
        # 根据来源调用不同的处理函数
        if source == 'down':
            # 调用精准降落控制器的方法，传入所有必需的数据
            self.precision_land_controller.update_down_tag(
            position_cam=np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float), 
            orientation_cam_quat=np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype=float)
            )       
        else:
            self.precision_land_controller.update_front_tag(
            position_cam=np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float), 
            orientation_cam_quat=np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype=float)
            )       

    def _fsm_update_callback(self):
        """状态机主更新循环。"""
        def _handle_task_result(task):
            try:
                task.result()
            except Exception as e:
                self.get_logger().error(f"FSM update loop encountered an exception: {e}", exc_info=True)

        if self.drone_state.search_started:
            future = asyncio.run_coroutine_threadsafe(
                self.precision_land_controller.update(), self.loop
            )
            future.add_done_callback(_handle_task_result)

        if self.drone_state.confirm_started:
            future = asyncio.run_coroutine_threadsafe(
                self.confirm_license_controller.update(), self.loop
            )
            future.add_done_callback(_handle_task_result)

    async def _autotune_callback(self, axis: str):
        """处理自动调参请求的异步核心函数"""
        # 检查无人机状态是否适合开始调参
        if not await self.status_monitor._check_preflight():
            return False, "无人机飞行前检查未通过，无法开始调参。"
        
        # 调用控制器的调参启动方法
        await self.precision_land_controller.start_autotune(axis)

        return True, f"{axis.upper()} 轴自动调参已成功启动。"

    async def _precision_land_callback(self):
        """精准降落的同步回调函数"""
        # 检查无人机状态是否适合精准降落
        if not await self.status_monitor._check_preflight():
            return False, "无人机飞行前检查未通过，无法精准降落。"
        # 启动精准降落
        await self.precision_land_controller.start()
        return True, "精准降落已成功启动。"

    def _sync_autotune_x_callback(self, request, response):
        """X轴自动调参的同步回调函数"""
        self.get_logger().info("收到X轴自动调参请求...")
        return self._sync_autotune_callback('x', response)

    def _sync_autotune_y_callback(self, request, response):
        """Y轴自动调参的同步回调函数"""
        self.get_logger().info("收到Y轴自动调参请求...")
        return self._sync_autotune_callback('y', response)

    def _sync_autotune_callback(self, axis, response):
        """通用的同步回调桥接函数"""
        try:
            future = asyncio.run_coroutine_threadsafe(
                self._autotune_callback(axis), self.loop)
            success, message = future.result(timeout=45) # 调参过程有超时
            
            response.success = success
            response.message = message
            return response
        except asyncio.TimeoutError:
            response.success = False
            response.message = "自动调参超时！"
            self.get_logger().error(response.message)
            asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land(), self.loop)
            return response
        except Exception as e:
            response.success = False
            response.message = f"自动调参过程中发生未知错误: {e}"
            self.get_logger().error(response.message)
            asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land(), self.loop)
            return response

    def _sync_precision_land_callback(self, request, response):
        """精准降落的同步回调函数"""
        self.get_logger().info("收到精准降落请求...")
        try:
            future = asyncio.run_coroutine_threadsafe(
                self._precision_land_callback(), self.loop)
            success, message = future.result(timeout=120) 
            
            response.success = success
            response.message = message
            return response

        except asyncio.TimeoutError:
            response.success = False
            response.message = "精准降落超时！"
            self.get_logger().error(response.message)
            asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land(), self.loop)
            return response
        except Exception as e:
            response.success = False
            response.message = f"精准降落过程中发生未知错误: {e}"
            self.get_logger().error(response.message)
            asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land(), self.loop)
            return response
    # async def _takeoff_callback(self, request):
    #     """处理起飞请求的异步核心函数"""
    #     response = Trigger.Response()
    #     # 检查无人机状态是否适合起飞
    #     if not await self.status_monitor._check_preflight():
    #         response.success = False
    #         response.message = "无人机飞行前检查未通过，无法起飞。"
    #         return response

    #     self.get_logger().info("正在执行起飞操作...")
    #     try:
    #         await self.mavsdk_controller.takeoff()
    #         response.success = True
    #         response.message = "起飞成功。"
    #         self.get_logger().info(response.message)
    #     except Exception as e:
    #         response.success = False
    #         response.message = f"起飞失败: {e}"
    #         self.get_logger().error(response.message)
    #     return response

    # def _sync_takeoff_callback(self, request, response):
    #     """起飞的同步回调函数"""
    #     self.get_logger().info("收到起飞请求...")
    #     try:
    #         future = asyncio.run_coroutine_threadsafe(
    #             self._takeoff_callback(request), self.loop)
    #         result = future.result(timeout=30)  # 起飞操作设置超时
            
    #         if result is not None:
    #             response.success = result.success
    #             response.message = result.message
    #         else:
    #             response.success = False
    #             response.message = "起飞回调返回空结果"
    #         return response
    #     except asyncio.TimeoutError:
    #         response.success = False
    #         response.message = "起飞超时！"
    #         self.get_logger().error(response.message)
    #         asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land(), self.loop)
    #         return response
    #     except Exception as e:
    #         response.success = False
    #         response.message = f"起飞过程中发生未知错误: {e}"
    #         self.get_logger().error(response.message)
    #         asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land(), self.loop)
    #         return response

    # async def _hybrid_navigation_callback(self, request):
    #     """处理混合导航请求"""
    #     response = Pos.Response()
    #     if not await self.status_monitor._check_preflight():
    #         response.success = False
    #         response.message = "无人机起飞状态不满足"
    #         return response

    #     if not validate_gps_coordinates(request.x, request.y, request.z):
    #         response.success = False
    #         response.message = "计算后的坐标参数无效"
    #         return response
    #     self.drone_state.update_target_position(request.x,request.y,request.z,request.w)
    #     await self.hybrid_navigation_controller.navigate_to_position()
    #     response.success = True
    #     response.message = f"混合导航完成"
    #     self.get_logger().info(response.message)
    #     return response

    # async def _navigation_callback(self, request):
    #     """处理相对位置导航请求 - 使用NED坐标系"""
    #     response = Pos.Response()

    #     if self.offboard_active:
    #         response.success = False
    #         response.message = "已有板外导航任务在执行中。"
    #         return response

    #     # 检查解锁状态
    #     if not await self.status_monitor._check_preflight():
    #         response.success = False
    #         response.message = "无人机起飞状态不满足"
    #         return response
        
    #     # 获取当前位置
    #     current_position_ned = self.drone_state.current_position_ned
    #     if current_position_ned is None:
    #         response.success = False
    #         response.message = "无法获取当前位置，请等待GPS信号稳定"
    #         self.get_logger().error("当前位置为空，请检查GPS连接")
    #         return response
    #     # 从Pos.srv消息中获取NED相对偏移
    #     if request.is_ned:
    #         self.drone_state.update_target_position(request.x+current_position_ned.north_m,
    #         request.y+current_position_ned.east_m,
    #         request.z+current_position_ned.down_m,request.w)           
    #     else:
    #         self.drone_state.update_target_position(request.x,request.y,request.z,request.w)           
    #     self.drone_state.navigation_mode = "RELATIVE" if request.is_ned else "ABSOLUTE"

    #     self.get_logger().info(f"[导航] 模式: {self.drone_state.navigation_mode}")

    #     # 使用板外导航控制器进行相对位置导航
    #     await self.offboard_navigation_controller.navigate_to_position()
    #     response.success = True
    #     response.message = f"导航完成"
    #     self.get_logger().info(response.message)
            
    #     return response

    # # ========= 同步桥接方法 =========

    # def _sync_navigation_callback(self, request, response):
    #     """导航的同步回调函数"""
    #     try:
    #         future = asyncio.run_coroutine_threadsafe(
    #             self._navigation_callback(request), self.loop)
    #         result = future.result(timeout=60)  # 相对导航需要更长时间
            
    #         # 确保响应对象被正确设置
    #         if result is not None:
    #             response.success = result.success
    #             response.message = result.message
    #         else:
    #             response.success = False
    #             response.message = "导航回调返回空结果"
    #         return response
    #     except asyncio.TimeoutError as e:               # 超时
    #         response.success = False
    #         response.message = f"板外导航超时: {e}，已启动应急降落"
    #         self.get_logger().error(response.message)
    #         asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land_autodisarm(), self.loop)
    #     except Exception as e:                          # 业务异常
    #         response.success = False
    #         response.message = f"板外导航失败: {e}"
    #         self.get_logger().error(response.message)
    #         asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land_autodisarm(), self.loop)   

    # def _sync_hybrid_navigation_callback(self, request, response):
    #     """混合导航的同步回调函数"""
    #     try:
    #         future = asyncio.run_coroutine_threadsafe(
    #             self._hybrid_navigation_callback(request), self.loop)
    #         result = future.result(timeout=60)  # 相对导航需要更长时间
            
    #         # 确保响应对象被正确设置
    #         if result is not None:
    #             response.success = result.success
    #             response.message = result.message
    #         else:
    #             response.success = False
    #             response.message = "导航回调返回空结果"
    #         return response
    #     except asyncio.TimeoutError as e:               # 超时
    #         response.success = False
    #         response.message = f"混合导航超时: {e}，已启动应急降落"
    #         self.get_logger().error(response.message)
    #         asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land_autodisarm(), self.loop)
    #     except Exception as e:                          # 业务异常
    #         response.success = False
    #         response.message = f"混合导航失败: {e}"
    #         self.get_logger().error(response.message)
    #         asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land_autodisarm(), self.loop)

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