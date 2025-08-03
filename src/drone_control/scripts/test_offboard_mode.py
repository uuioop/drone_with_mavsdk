#!/usr/bin/env python3
"""
测试OFFBOARD模式的脚本
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
import time

class OffboardTestNode(Node):
    def __init__(self):
        super().__init__('offboard_test_node')
        self.get_logger().info("OFFBOARD模式测试节点启动")
        
        # 创建服务客户端
        self.connect_client = self.create_client(Trigger, 'drone/connect')
        self.arm_client = self.create_client(Trigger, 'drone/arm')
        self.takeoff_client = self.create_client(Trigger, 'drone/takeoff')
        self.land_client = self.create_client(Trigger, 'drone/land')
        self.offboard_start_client = self.create_client(Trigger, 'drone/offboard_start')
        self.offboard_stop_client = self.create_client(Trigger, 'drone/offboard_stop')
        
        # 创建订阅者
        self.offboard_status_sub = self.create_subscription(
            String, 'drone/offboard_status', self.offboard_status_callback, 10)
        
        # 等待服务可用
        self.wait_for_services()
        
    def wait_for_services(self):
        """等待所有服务可用"""
        services = [
            self.connect_client,
            self.arm_client,
            self.takeoff_client,
            self.land_client,
            self.offboard_start_client,
            self.offboard_stop_client
        ]
        
        for service in services:
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"等待服务 {service.srv_name} 可用...")
    
    def offboard_status_callback(self, msg):
        """处理OFFBOARD状态消息"""
        self.get_logger().info(f"OFFBOARD状态: {msg.data}")
    
    async def test_offboard_mode(self):
        """测试OFFBOARD模式"""
        self.get_logger().info("开始OFFBOARD模式测试")
        
        # 1. 连接无人机
        self.get_logger().info("步骤1: 连接无人机")
        connect_request = Trigger.Request()
        connect_future = self.connect_client.call_async(connect_request)
        connect_response = await connect_future
        self.get_logger().info(f"连接结果: {connect_response.message}")
        
        if not connect_response.success:
            self.get_logger().error("连接失败，退出测试")
            return
        
        # 等待连接稳定
        time.sleep(2)
        
        # 2. 启动OFFBOARD模式
        self.get_logger().info("步骤2: 启动OFFBOARD模式")
        offboard_start_request = Trigger.Request()
        offboard_start_future = self.offboard_start_client.call_async(offboard_start_request)
        offboard_start_response = await offboard_start_future
        self.get_logger().info(f"OFFBOARD启动结果: {offboard_start_response.message}")
        
        if not offboard_start_response.success:
            self.get_logger().error("OFFBOARD模式启动失败")
            return
        
        # 等待OFFBOARD模式稳定
        time.sleep(3)
        
        # 3. 解锁无人机
        self.get_logger().info("步骤3: 解锁无人机")
        arm_request = Trigger.Request()
        arm_future = self.arm_client.call_async(arm_request)
        arm_response = await arm_future
        self.get_logger().info(f"解锁结果: {arm_response.message}")
        
        if not arm_response.success:
            self.get_logger().error("解锁失败")
            return
        
        # 等待解锁完成
        time.sleep(2)
        
        # 4. 起飞
        self.get_logger().info("步骤4: 起飞")
        takeoff_request = Trigger.Request()
        takeoff_future = self.takeoff_client.call_async(takeoff_request)
        takeoff_response = await takeoff_future
        self.get_logger().info("起飞指令已发送")
        
        # 等待起飞完成
        time.sleep(10)
        
        # 5. 悬停一段时间
        self.get_logger().info("步骤5: 悬停测试")
        time.sleep(5)
        
        # 6. 降落
        self.get_logger().info("步骤6: 降落")
        land_request = Trigger.Request()
        land_future = self.land_client.call_async(land_request)
        land_response = await land_future
        self.get_logger().info("降落指令已发送")
        
        # 等待降落完成
        time.sleep(10)
        
        # 7. 停止OFFBOARD模式
        self.get_logger().info("步骤7: 停止OFFBOARD模式")
        offboard_stop_request = Trigger.Request()
        offboard_stop_future = self.offboard_stop_client.call_async(offboard_stop_request)
        offboard_stop_response = await offboard_stop_future
        self.get_logger().info(f"OFFBOARD停止结果: {offboard_stop_response.message}")
        
        self.get_logger().info("OFFBOARD模式测试完成")

def main(args=None):
    rclpy.init(args=args)
    node = OffboardTestNode()
    
    try:
        # 运行测试
        rclpy.spin_once(node, timeout_sec=1.0)
        node.get_logger().info("开始OFFBOARD模式测试")
        
        # 这里可以添加具体的测试逻辑
        # 由于这是一个同步脚本，我们只是启动节点并等待
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("测试被用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 