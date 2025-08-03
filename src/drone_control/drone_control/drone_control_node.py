#!/usr/bin/env python3
"""
无人机控制节点主文件 - 支持任务模式
使用模块化结构和自定义Nav.srv服务
"""

import math
import threading
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from mavsdk import System, action
from std_srvs.srv import Trigger
from std_msgs.msg import String

# 导入自定义服务类型
from drone_control.srv import Nav, Pos

# 导入核心模块
from drone_control.core import DroneState, HybridNavigationController, LicensePlateProcessor, DroneStatusMonitor
from drone_control.utils.utils import calculate_relative_position_target, validate_gps_coordinates



class DroneControlNode(Node):
    """无人机控制节点主类 - 支持任务模式"""
    def __init__(self):
        super().__init__('drone_control_node')
        self.get_logger().info("无人机控制节点启动中...")
        
        # 初始化组件
        self.drone = System()
        self.drone_state = DroneState()
        self.system_address = "udp://:14540"
        
        # 任务模式状态
        self.hybrid_active = False

        # 初始化控制器
        self.hybrid_navigation_controller = HybridNavigationController(self.drone, self.get_logger(), self.drone_state)
        self.license_plate_processor = LicensePlateProcessor(self.get_logger())
        
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
        # self.connect_srv = self.create_service(
        #     Trigger, 'drone/connect', self._sync_connect_callback)
        # self.arm_srv = self.create_service(
        #     Trigger, 'drone/arm', self._sync_arm_callback)
        # self.takeoff_srv = self.create_service(
        #     Empty, 'drone/takeoff', self._sync_takeoff_callback)
        # self.land_srv = self.create_service(
        #     Empty, 'drone/land', self._sync_land_callback)
        # 使用自定义Nav.srv 和Pos.srv 服务类型
        self.relative_navigation_srv = self.create_service(
            Pos, 'drone/relative_navigation', self._sync_relative_navigation_callback)
        self.hybrid_navigation_srv = self.create_service(
            Nav, 'drone/hybrid_navigation', self._sync_hybrid_navigation_callback)

    def _setup_subscriptions(self):
        """设置订阅和发布"""
        # 订阅
        self.license_plate_sub = self.create_subscription(
            String, 'ocr/license_plate_result', self._license_plate_callback, 10)
        
        # 发布
        self.position_pub = self.create_publisher(String, 'drone/position', 10)

    def _start_event_loop(self):
        """启动MAVSDK事件循环线程"""
        def run_loop():
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self.status_monitor.start_monitoring())
        
        self.loop_thread = threading.Thread(target=run_loop, daemon=True)
        self.loop_thread.start()

    async def _connect_to_drone(self):
        """连接到无人机"""       
        self.get_logger().info(f"尝试连接到无人机: {self.system_address}")
        await self.drone.connect(system_address=self.system_address)
        # 获取初始位置
        await self._get_home_position()
        self.get_logger().info("等待无人机连接...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("成功连接到无人机!")
                self.drone_state.update_connection(True)
                break

    async def _get_home_position(self):
        """获取起始位置 - 增强错误处理"""
        try:
            # 等待telemetry初始化
            await asyncio.sleep(1)
            
            async for position in self.drone.telemetry.position():
                self.drone_state.update_home_position(position)
                self.get_logger().info(f"起始位置: 纬度={position.latitude_deg:.6f}, 经度={position.longitude_deg:.6f}, 高度={position.absolute_altitude_m:.2f}m")
                break
        except Exception as e:
            self.get_logger().error(f"获取起始位置失败: {e}")
            # 设置默认起始位置
            self.drone_state.update_home_position(type('Position', (), {
                'latitude_deg': 0.0,
                'longitude_deg': 0.0,
                'absolute_altitude_m': 0.0
            })())

    async def _relative_navigation_callback(self, request):
        """处理相对位置导航请求 - 使用NED坐标系"""
        response = Pos.Response()
        # 检查解锁状态
        if not await self.status_monitor._check_preflight():
            response.success = False
            response.message = "无人机起飞状态不满足"
            return response
        
        # 检查任务模式是否完成
        if self.hybrid_active:
            response.success = False
            response.message = "混合导航模式未完成，请先等待混合导航模式完成"
            return response
        
        try:
            # 获取当前位置
            current_position = self.drone_state.current_position
            if current_position is None:
                response.success = False
                response.message = "无法获取当前位置"
                return response
            
            # 从Pos.srv消息中获取NED相对偏移
            relative_n = request.north  # 北向偏移（米）
            relative_e = request.east  # 东向偏移（米）
            relative_d = request.down  # 下向偏移（米）
            
            self.get_logger().info(f"[相对导航] 当前位置: 纬度={current_position.latitude_deg:.6f}, 经度={current_position.longitude_deg:.6f}, 高度={current_position.absolute_altitude_m:.2f}m")
            self.get_logger().info(f"[相对导航] NED相对偏移: 北={relative_n:.2f}m, 东={relative_e:.2f}m, 下={relative_d:.2f}m")
            
            # 使用工具函数计算目标位置
            target_lat, target_lon, target_alt = calculate_relative_position_target(
                current_position.latitude_deg,
                current_position.longitude_deg,
                current_position.absolute_altitude_m,
                relative_n, relative_e, relative_d
            )
            
            # 验证计算后的坐标
            if not validate_gps_coordinates(target_lat, target_lon, target_alt):
                response.success = False
                response.message = "计算后的坐标参数无效"
                return response

            self.drone_state.target_position = type('Position', (), {
                'latitude_deg': target_lat,
                'longitude_deg': target_lon,
                'absolute_altitude_m': target_alt
            })()

            # 使用混合导航控制器进行相对位置导航
            self.hybrid_active = True
            await self.hybrid_navigation_controller.navigate_to_position()
            self.hybrid_active = False
            
            response.success = True
            response.message = f"相对导航完成: NED偏移({relative_n:.2f}, {relative_e:.2f}, {relative_d:.2f})"
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"相对导航错误: {str(e)}"
            self.get_logger().error(response.message)
        return response

    async def _hybrid_navigation_callback(self, request):
        """处理混合导航请求 - 先任务模式，后板外模式"""
        response = Nav.Response()
        
        # 检查解锁状态
        if not await self.status_monitor._check_preflight():
            response.success = False
            response.message = "无人机起飞状态不满足"
            return response
        
        # 检查任务模式是否完成
        if self.hybrid_active:
            response.success = False
            response.message = "混合导航模式未完成，请先等待混合导航模式完成"
            return response
        
        try:
            # 从Nav.srv消息中获取坐标
            lat = request.latitude_deg
            lon = request.longitude_deg
            alt = request.absolute_altitude_m
            
            # 验证坐标参数
            if not (-90 <= lat <= 90) or not (-180 <= lon <= 180) or alt < 0:
                response.success = False
                response.message = "坐标参数无效：纬度应在-90到90之间，经度应在-180到180之间，高度应大于0"
                return response
            
            self.drone_state.target_position = type('Position', (), {
                'latitude_deg': lat,
                'longitude_deg': lon,
                'absolute_altitude_m': alt
            })()

            self.get_logger().info(f"[混合导航] 开始混合导航到: 纬度={lat:.6f}, 经度={lon:.6f}, 高度={alt:.2f}m")
            
            self.hybrid_active = True
            # 使用混合导航控制器
            await self.hybrid_navigation_controller.navigate_to_position()
            self.hybrid_active = False
            response.success = True
            response.message = f"混合导航完成: 纬度={lat:.6f}, 经度={lon:.6f}, 高度={alt:.2f}m"
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"混合导航错误: {str(e)}"
            self.get_logger().error(response.message)
        return response

    def _license_plate_callback(self, msg):
        """处理号牌识别结果"""
        self.license_plate_processor.process_license_plate(msg.data)

    # ========= 同步桥接方法 =========

    def _sync_relative_navigation_callback(self, request, response):
        """相对位置导航的同步回调函数"""
        try:
            future = asyncio.run_coroutine_threadsafe(
                self._relative_navigation_callback(request), self.loop)
            result = future.result(timeout=60)  # 相对导航需要更长时间
            
            # 确保响应对象被正确设置
            if result is not None:
                response.success = result.success
                response.message = result.message
            else:
                response.success = False
                response.message = "相对导航回调返回空结果"
                
            return response
        except Exception as e:
            response.success = False
            response.message = f"相对导航超时: {str(e)}"
            return response

    def _sync_hybrid_navigation_callback(self, request, response):
        try:
            future = asyncio.run_coroutine_threadsafe(
                self._hybrid_navigation_callback(request), self.loop)
            result = future.result(timeout=60)  # 混合导航需要更长时间
            
            # 确保响应对象被正确设置
            if result is not None:
                response.success = result.success
                response.message = result.message
            else:
                response.success = False
                response.message = "混合导航回调返回空结果"
                
            return response
        except Exception as e:
            response.success = False
            response.message = f"混合导航超时: {str(e)}"
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

    # 在事件循环中执行自动连接
    asyncio.run_coroutine_threadsafe(node._connect_to_drone(), node.loop)

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 