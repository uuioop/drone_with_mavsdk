#!/usr/bin/env python3
"""
无人机控制节点主文件 - 支持板外模式导航和ArUco标记跟踪

功能概述：
1. 集成MAVSDK与ROS2，提供无人机控制接口
2. 支持板外模式导航（相对位置控制）
3. 支持精准降落（基于ArUco标记的视觉引导）
4. 支持门牌识别与确认功能
5. 提供混合导航服务（GPS+视觉）
6. 状态机管理多个飞行任务

主要组件：
- DroneState: 无人机状态管理
- DroneStatusMonitor: 状态监控器
- MavsdkController: MAVSDK控制接口
- PrecisionLandController: 精准降落控制器
- ConfirmLicenseController: 门牌确认控制器
- OffboardNavigationController: 板外导航控制器
- LicensePlateProcessor: 号牌数据库管理

服务接口：
- /drone/navigation: 相对位置导航服务（NED坐标系）
- /drone/hybrid_navigation: 混合导航服务（GPS+视觉）
- /drone/precision_land: 精准降落服务
- /drone/confirm_license: 门牌确认服务

订阅话题：
- /license_detection_result: 门牌识别结果
- /mono_tracker/target_pose: 前置ArUco标记位姿
- /infra_tracker/target_pose: 下方ArUco标记位姿
"""
import time
import math
import os
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
from drone_control_interfaces.srv import Pos,Nav
# 导入自定义消息类型
from drone_control_interfaces.msg import LicenseInfo
# 导入核心模块
from drone_control.core import DroneState, DroneStatusMonitor,MavsdkController
from drone_control.precision_land import PrecisionLandController
from drone_control.confirm_license import ConfirmLicenseController
from drone_control.services import OffboardNavigationController,HybridNavigationController
from drone_control.SQLite import LicensePlateProcessor
from drone_control.utils import validate_gps_coordinates

class DroneControlNode(Node):
    """
    无人机控制节点主类 - 支持板外模式导航和ArUco标记跟踪
    
    功能：
    - 集成MAVSDK与ROS2，提供完整的无人机控制接口
    - 管理多个飞行任务状态机（精准降落、门牌确认等）
    - 提供板外模式导航服务（相对位置控制）
    - 处理ArUco标记检测结果，支持视觉引导功能
    - 管理号牌识别数据库和验证流程
    
    主要属性：
        drone: MAVSDK系统实例
        drone_state: 无人机状态管理器
        status_monitor: 状态监控器
        mavsdk_controller: MAVSDK控制接口
        precision_land_controller: 精准降落控制器
        confirm_license_controller: 门牌确认控制器
        offboard_navigation_controller: 板外导航控制器
        license_plate_processor: 号牌数据库管理器
    """
    def __init__(self):
        """
        初始化无人机控制节点
        
        初始化内容：
        - 创建MAVSDK系统实例和无人机状态管理器
        - 初始化状态监控器和MAVSDK控制器
        - 检测串口设备并设置连接地址（支持仿真和实飞）
        - 初始化号牌数据库和添加测试数据
        - 创建各种任务控制器实例
        - 设置ROS服务、订阅和发布
        - 启动MAVSDK事件循环线程
        """
        super().__init__('drone_control_node')  # 初始化ROS2节点
        # 初始化组件
        self.drone = System()  # 创建MAVSDK系统实例
        self.drone_state = DroneState()  # 初始化无人机状态管理器
        # 初始化状态监控器
        self.status_monitor = DroneStatusMonitor(self)  # 创建状态监控器实例
        # 初始化MAVSDK控制器
        self.mavsdk_controller = MavsdkController(self)  # 创建MAVSDK控制器实例
        # --- 动态检查串口设备并设置 system_address ---        
        # 使用字典存储可用的串口设备信息，可根据实际情况添加或删除
        serial_ports = {
            'ACM0': '/dev/ttyACM0',
            'ACM1': '/dev/ttyACM1',
            'ACM2': '/dev/ttyACM2'
        }
        
        # 遍历字典查找可用的串口设备
        self.system_address = None
        for port_name, port_path in serial_ports.items():
            if os.path.exists(port_path):
                self.system_address = f"serial://{port_path}:115200"
                self.get_logger().info(f"检测到飞控串口 {port_path}，将使用该设备连接。")
                break
        
        # 如果没有找到可用的串口设备，回退到UDP连接方式
        if self.system_address is None:
            self.system_address = "udpin://0.0.0.0:14540"
            ports_str = ", ".join(serial_ports.values())
            self.get_logger().warning(f"未检测到串口设备 ({ports_str})，将回退到默认的UDP连接方式。")
        # 板外模式状态
        self.offboard_active = False  # 初始化板外模式状态为未激活

        # 初始化号牌识别结果
        # self.license_plate_result = ""  # 初始化号牌识别结果为空字符串
        # self.recognized_timer = 0  # 初始化识别计数器为0

        # 初始化控制器
        self.precision_land_controller = PrecisionLandController(self)  # 创建精准降落控制器
        self.confirm_license_controller = ConfirmLicenseController(self)  # 创建门牌确认控制器

        # 为每个FSM控制器添加状态锁，防止任务堆积
        self.precision_land_update_in_progress = False  # 精准降落更新状态锁
        self.confirm_license_update_in_progress = False  # 门牌确认更新状态锁

        self.offboard_navigation_controller = OffboardNavigationController(self)  # 创建板外导航控制器

        self.hybrid_navigation_controller = HybridNavigationController(self)

        # 创建独立事件循环
        self.loop = asyncio.new_event_loop()  # 创建新的异步事件循环
        
        # 设置ROS接口
        self._setup_services()  # 设置ROS服务
        self._setup_subscriptions()  # 设置ROS订阅和发布
        # 设置状态监控器的信息发布器
        self.status_monitor.set_info_publisher(self.drone_info_publisher)  # 为状态监控器设置信息发布器

        # 启动MAVSDK事件循环线程
        self._start_event_loop()  # 启动事件循环线程

        self.get_logger().info("节点初始化完成")  # 记录初始化完成日志

    def _setup_services(self):
        """设置ROS服务"""
        # 适合于仿真环境，使用该服务进行相对位置的导航（NED坐标系）
        self.navigation_srv = self.create_service(
            Pos, 'drone/navigation', self._sync_navigation_callback)  # 创建相对位置导航服务
        self.hybrid_navigation_srv = self.create_service(
            Nav, 'drone/hybrid_navigation', self._sync_hybrid_navigation_callback)  # 创建混合导航服务
        self.precision_srv = self.create_service(
            Trigger, 'drone/precision_land', self._sync_precision_land_callback)  # 创建精准降落服务
        self.confirm_license_srv = self.create_service(
            Trigger, 'drone/confirm_license', self._sync_confirm_license_callback)  # 创建门牌确认服务

    def _setup_subscriptions(self):
        """设置订阅和发布"""
        qos = rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)  # 设置QoS配置为深度1+尽力而为
        
        self.drone_info_publisher = self.create_publisher(String, 'drone/info', 10)  # 创建无人机状态信息发布器

        self.license_plate_sub = self.create_subscription(
            LicenseInfo, '/license_detection_result', self._license_plate_callback, qos)  # 创建号牌识别结果订阅器
        
        self.target_pose_sub_front = self.create_subscription(
            PoseStamped, '/mono_tracker/target_pose', lambda msg: self._target_pose_callback(msg, 'down'), qos)  # 创建前置ArUco标记位姿订阅器
        
        self.target_pose_sub_down = self.create_subscription(
            PoseStamped, '/infra_tracker/target_pose', lambda msg: self._target_pose_callback(msg, 'front'), qos)  # 创建下方ArUco标记位姿订阅器
    
    def _start_event_loop(self):
        """启动MAVSDK事件循环线程"""
        def run_loop():
            asyncio.set_event_loop(self.loop)  # 设置当前线程的事件循环
            self.loop.run_until_complete(self._run_all())  # 运行异步任务

        self.loop_thread = threading.Thread(target=run_loop, daemon=True)  # 创建守护线程运行事件循环
        self.loop_thread.start()  # 启动事件循环线程

        # 创建状态机主循环定时器
        self.fsm_update_timer = self.create_timer(0.1, self._fsm_update_callback) # 10Hz 状态机更新频率

    async def _run_all(self):
        """先连接无人机，再开始持续监控"""
        try:
            await self.mavsdk_controller._connect_to_drone(self.system_address)  # 连接到无人机
        except Exception as e:
            self.get_logger().error(f"连接到无人机失败: {e}")  # 记录连接失败日志
        await self.status_monitor.start_monitoring()  # 启动状态监控

    def _license_plate_callback(self, msg: LicenseInfo):
        """号牌识别回调函数"""
        # 更新号牌数据库中的识别结果
        self.confirm_license_controller.update_recognized_plate(msg.plate_no, msg.center_x, msg.center_y)  # 更新数据
            
    def _target_pose_callback(self, msg: PoseStamped, source: str):
        """目标位姿回调函数"""
        # 根据来源调用不同的处理函数
        if source == 'down':  # 下方ArUco标记（用于精准降落平台标记）
            # 调用精准降落控制器的方法，传入所有必需的数据
            self.precision_land_controller.update_down_tag(  # 更新下方标记数据
            position_cam=np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float), 
            orientation_cam_quat=np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype=float)
            )       
        else:  # 前置ArUco标记（用于精准降落前置标记和门牌确认）
            self.precision_land_controller.update_front_tag(  # 更新前置标记数据
            position_cam=np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float), 
            orientation_cam_quat=np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype=float)
            )       

    def _fsm_update_callback(self):
        """
        状态机主更新循环。
        使用状态锁确保同一时间只有一个update任务在事件循环中运行。
        """
        # --- 处理精准降落任务 ---
        if self.drone_state.search_started and not self.precision_land_update_in_progress:
            
            # 定义一个专门用于精准降落的回调，在任务完成时释放锁
            def _precision_land_done_callback(task):
                self.precision_land_update_in_progress = False  # 释放锁
                try:
                    task.result()  # 获取任务结果
                except Exception as e:
                    self.get_logger().error(f"Precision Land FSM update exception: {e}")

            # 标记任务正在进行
            self.precision_land_update_in_progress = True
            
            # 提交任务到事件循环
            future = asyncio.run_coroutine_threadsafe(
                self.precision_land_controller.update(), self.loop
            )
            future.add_done_callback(_precision_land_done_callback)

        # --- 处理确认号牌任务 ---
        if self.drone_state.confirm_started and not self.confirm_license_update_in_progress:

            # 定义一个专门用于确认号牌的回调，在任务完成时释放锁
            def _confirm_license_done_callback(task):
                self.confirm_license_update_in_progress = False # 释放锁
                try:
                    task.result()
                except Exception as e:
                    self.get_logger().error(f"Confirm License FSM update exception: {e}")

            # 标记任务正在进行
            self.confirm_license_update_in_progress = True

            # 提交任务到事件循环
            future = asyncio.run_coroutine_threadsafe(
                self.confirm_license_controller.update(), self.loop
            )
            future.add_done_callback(_confirm_license_done_callback)

    async def _precision_land_callback(self):
        """精准降落的同步回调函数"""
        if not await self.status_monitor.check_preflight():
            return False, "无人机飞行前检查未通过，无法精准降落。"
        # 启动精准降落
        await self.precision_land_controller.start()
        return True, "精准降落已成功启动。"
    
    async def _confirm_license_callback(self):
        """确认门牌的同步回调函数"""
        if not await self.status_monitor.check_preflight():
            return False, "无人机飞行前检查未通过，无法确认门牌。"
        # 启动确认门牌
        await self.confirm_license_controller.start()
        return True, "确认门牌已成功启动。"

    async def _hybrid_navigation_callback(self, request):
        """处理混合导航请求"""
        response = Pos.Response()
        if not await self.status_monitor.check_preflight():
            response.success = False
            response.message = "无人机起飞状态不满足"
            return response

        if not validate_gps_coordinates(request.latitude_deg, request.longitude_deg, request.absolute_altitude_m):
            response.success = False
            response.message = "计算后的坐标参数无效"
            return response
        self.drone_state.update_target_position(request.latitude_deg,request.longitude_deg,request.absolute_altitude_m,request.yaw_deg)
        await self.hybrid_navigation_controller.navigate_to_position()
        response.success = True
        response.message = f"混合导航完成"
        self.get_logger().info(response.message)
        return response

    async def _navigation_callback(self, request,response):
        """处理相对位置导航请求 - 使用NED坐标系"""
        if self.offboard_active:
            response.success = False
            response.message = "已有板外导航任务在执行中。"
            return response

        # 检查解锁状态
        if not await self.status_monitor.check_preflight():
            response.success = False
            response.message = "无人机起飞状态不满足"
            return response
        
        # 获取当前位置
        current_position_ned = self.drone_state.current_position_ned
        if current_position_ned is None:
            response.success = False
            response.message = "无法获取当前位置，请等待GPS信号稳定"
            self.get_logger().error("当前位置为空，请检查GPS连接")
            return response
        self.drone_state.update_target_position(request.x+current_position_ned.north_m,
        request.y+current_position_ned.east_m,
        request.z+current_position_ned.down_m,request.w)           
        # 使用板外导航控制器进行相对位置导航
        await self.offboard_navigation_controller.navigate_to_position()
        response.success = True
        response.message = f"导航完成"
        self.get_logger().info(response.message)
            
        return response

    # ========= 同步桥接方法 =========
    def _sync_precision_land_callback(self, request, response):
        """精准降落的同步回调函数"""
        self.get_logger().info("收到精准降落请求...")
        try:
            future = asyncio.run_coroutine_threadsafe(
                self._precision_land_callback(), self.loop)
            success, message = future.result() 
            
            response.success = success
            response.message = message
            return response
        except Exception as e:
            response.success = False
            response.message = f"精准降落过程中发生未知错误: {e}"
            self.get_logger().error(response.message)
            asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land(), self.loop)
            return response
        
    def _sync_confirm_license_callback(self, request, response):
        """确认门牌的同步回调函数"""
        self.get_logger().info("收到确认门牌请求...")
        try:
            future = asyncio.run_coroutine_threadsafe(
                self._confirm_license_callback(), self.loop)
            success, message = future.result() 
            
            response.success = success
            response.message = message
            return response
        except Exception as e:
            response.success = False
            response.message = f"确认门牌过程中发生未知错误: {e}"
            self.get_logger().error(response.message)
            asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land(), self.loop)
            return response

    def _sync_navigation_callback(self, request, response):
        """导航的同步回调函数"""
        try:
            future = asyncio.run_coroutine_threadsafe(
                self._navigation_callback(request,response), self.loop)
            result = future.result(timeout=60)  # 相对导航需要更长时间
            
            # 确保响应对象被正确设置
            if result is not None:
                response.success = result.success
                response.message = result.message
            else:
                response.success = False
                response.message = "导航回调返回空结果"
            return response
        except asyncio.TimeoutError as e:               # 超时
            response.success = False
            response.message = f"板外导航超时: {e}，已启动应急降落"
            self.get_logger().error(response.message)
            asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land_autodisarm(), self.loop)
        except Exception as e:                          # 业务异常
            response.success = False
            response.message = f"板外导航失败: {e}"
            self.get_logger().error(response.message)
            asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land_autodisarm(), self.loop)   

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
        except asyncio.TimeoutError as e:               # 超时
            response.success = False
            response.message = f"混合导航超时: {e}，已启动应急降落"
            self.get_logger().error(response.message)
            asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land_autodisarm(), self.loop)
        except Exception as e:                          # 业务异常
            response.success = False
            response.message = f"混合导航失败: {e}"
            self.get_logger().error(response.message)
            asyncio.run_coroutine_threadsafe(self.mavsdk_controller.land_autodisarm(), self.loop)

    def destroy_node(self):
        """关闭节点"""
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
