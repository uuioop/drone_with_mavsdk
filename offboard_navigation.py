#!/usr/bin/env python3
"""
板外模式导航控制模块 - 只使用板外模式进行导航
"""

import asyncio
import time
import threading
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityBodyYawspeed, VelocityNedYaw
from drone_control.utils.utils import observe_is_in_air
from drone_control.SQLite.license_plate_database import LicensePlateDatabase

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image


class OffboardNavigationController:
    """板外模式导航控制类 - 只使用板外模式进行导航"""

    def __init__(self, drone, logger, drone_state, license_plate_result, ros_node=None):
        self.drone = drone
        self.logger = logger
        self.drone_state = drone_state
        self.license_plate_result = license_plate_result
        self.ros_node = ros_node

        # 固定速度参数
        self.fixed_velocity = 1.0  # 固定飞行速度 (m/s)
        self.velocity_yaw_rate = 0.5  # 偏航角速度 (rad/s)
        
        # 号牌识别相关
        self.license_plate_db = LicensePlateDatabase("test_license_plates.db")
        self.recognition_result = None
        self.recognition_timeout = 30.0  # 号牌识别超时时间(秒)
        
        # ROS2订阅器
        if self.ros_node:
            self.license_result_subscriber = self.ros_node.create_subscription(
                String,
                '/license_detection_result',
                self.license_recognition_callback,
                10
            )

        
    async def navigate_to_position(self,target_north=None,target_east=None,target_down=None):
        """板外模式导航到目标位置 - 使用位置控制"""
        try:
            # 解锁无人机
            await self.drone.action.arm()
            self.logger.info("解锁成功，准备使用板外模式")

            # 启动板外模式
            await self.start_offboard_mode()

            # 分阶段导航
            if target_north is None or target_east is None or target_down is None:
                target_north, target_east, target_down = (
                    self.drone_state.calculate_target_ned_from_origin()
                )
            self.logger.info(
                f"target_north: {target_north}, target_east: {target_east}, target_down: {target_down}"
            )

            await self.fly_to_target_altitude(target_down)
            await self.fly_to_target_position(target_north, target_east)

            # 观察四周环境
            self.observe_environment()
            
            # 号牌识别检查
            if await self.check_license_plate_recognition():
                self.logger.info("[号牌识别] 识别到匹配的号牌，准备着陆")
            else:
                self.logger.warning("[号牌识别] 未识别到匹配的号牌，取消着陆任务")
                await self.stop_offboard_mode()
                return False

            await self.stop_offboard_mode()

            # 着陆
            await self.drone.action.land()
            await observe_is_in_air(self.drone, self.logger)

        except Exception as e:
            self.logger.error(f"[板外导航] 导航失败: {e}")
            # 尝试降落
            try:
                await self.drone.action.land()
                await observe_is_in_air(self.drone, self.logger)
            except:
                pass
            raise

    async def start_offboard_mode(self):
        """启动板外模式"""
        try:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, 0.0, 0.0)
            )
            await self.drone.offboard.start()
            self.logger.info("[板外导航] 板外模式已启动")
        except Exception as e:
            self.logger.error(f"[板外导航] 启动板外模式失败: {e}")
            raise

    async def observe_environment(self):
        """观察四周环境"""
        self.logger.info("[板外导航] 开始观察四周环境")

        # 旋转180度观察
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 30.0))
        await asyncio.sleep(10)


        # 停止所有移动，确保完全静止
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(1)  # 等待1秒确保停止

        self.logger.info("[板外导航] 观察四周环境完成")

    async def fly_to_target_altitude(self, target_down):
        """飞到目标高度 - 使用位置控制"""
        self.logger.info(f"[板外导航] 阶段1：飞到目标高度 {target_down:.2f}m")
        
        # 获取当前水平位置
        current_north, current_east, _ = (
            self.drone_state.calculate_ned_from_origin()
        )

        # 使用位置控制飞到目标高度
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(current_north, current_east, target_down, 0.0)
        )
        # 等待到达目标高度
        while True:
            altitude_diff = target_down - self.get_current_down()
            if abs(altitude_diff) <= 0.2:
                break
            await asyncio.sleep(0.1)  # 每0.5秒检查一次

        self.logger.info(f"[板外导航] 已到达目标高度 {target_down:.2f}m")

               
    async def fly_to_target_position(self, target_north, target_east):
        """飞到目标水平位置"""
        self.logger.info(
            f"[板外导航] 阶段2：飞到目标水平位置 北={target_north:.2f}m, 东={target_east:.2f}m"
        )

        # 获取当前高度
        _, _, current_down = (
            self.drone_state.calculate_ned_from_origin()
        )

        # 使用位置控制飞到目标位置
    
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(target_north, target_east, current_down, 0.0)
        )
        
        # 等待到达目标位置
        while True:
            # 获取当前水平位置
            current_north, current_east, _ = (
                self.drone_state.calculate_ned_from_origin()
            )

            # 计算水平距离
            distance_n = target_north - current_north
            distance_e = target_east - current_east
            horizontal_distance = (distance_n**2 + distance_e**2) ** 0.5

            if horizontal_distance <= 0.4:
                break
            await asyncio.sleep(0.5)  # 每0.5秒检查一次
        self.logger.info(f"[板外导航] 已到达目标位置 北={target_north:.2f}m, 东={target_east:.2f}m")

    async def get_current_down(self):
        """获取当前下向位置"""
        _, _, current_down = self.drone_state.calculate_ned_from_origin()
        return current_down

    async def stop_offboard_mode(self):
        """停止板外模式"""
        try:
            await self.drone.offboard.stop()
            self.logger.info("[板外导航] 板外模式已停止")
        except OffboardError as error:
            self.logger.error(f"[板外导航] 停止板外模式失败: {error}")

    def license_recognition_callback(self, msg):
        """号牌识别结果回调函数"""
        try:
            # 解析识别结果
            recognition_data = msg.data
            self.logger.info(f"[号牌识别] 收到识别结果: {recognition_data}")
            
            # 简单解析格式，假设格式为 "plate_number:confidence"
            if ":" in recognition_data:
                parts = recognition_data.split(":")
                if len(parts) >= 2:
                    plate_number = parts[0].strip()
                    confidence = float(parts[1].strip()) if parts[1].strip().replace('.', '').isdigit() else 0.0
                    
                    self.recognition_result = {
                        'plate_number': plate_number,
                        'confidence': confidence,
                        'timestamp': time.time()
                    }
                    self.logger.info(f"[号牌识别] 解析结果: 号牌={plate_number}, 置信度={confidence}")
            else:
                # 如果没有置信度信息，直接使用识别结果
                self.recognition_result = {
                    'plate_number': recognition_data.strip(),
                    'confidence': 1.0,
                    'timestamp': time.time()
                }
                
        except Exception as e:
            self.logger.error(f"[号牌识别] 解析识别结果失败: {e}")

    async def check_license_plate_recognition(self):
        """检查号牌识别结果并与数据库比对"""
        try:
            self.logger.info("[号牌识别] 开始号牌识别检查...")
            
            # 重置识别结果
            self.recognition_result = None
            
            # 等待识别结果，设置超时时间
            start_time = time.time()
            while time.time() - start_time < self.recognition_timeout:
                if self.recognition_result is not None:
                    break
                await asyncio.sleep(0.5)  # 每0.5秒检查一次
            
            # 检查是否超时
            if self.recognition_result is None:
                self.logger.warning(f"[号牌识别] 识别超时({self.recognition_timeout}秒)，未收到识别结果")
                return False
            
            # 获取识别到的号牌
            recognized_plate = self.recognition_result['plate_number']
            confidence = self.recognition_result['confidence']
            
            self.logger.info(f"[号牌识别] 识别到号牌: {recognized_plate}, 置信度: {confidence}")
            
            # 检查置信度是否足够高
            if confidence < 0.8:  # 置信度阈值
                self.logger.warning(f"[号牌识别] 置信度过低({confidence}), 不进行匹配")
                return False
            
            # 查询数据库中的所有号牌
            all_plates = self.license_plate_db.get_all_license_plates()
            if not all_plates:
                self.logger.warning("[号牌识别] 数据库中没有号牌数据")
                return False
            
            # 检查识别到的号牌是否在数据库中
            for plate_info in all_plates:
                db_plate_number = plate_info[1]  # 假设第二列是号牌号码
                if recognized_plate == db_plate_number:
                    self.logger.info(f"[号牌识别] 匹配成功！识别号牌 '{recognized_plate}' 在数据库中找到")
                    
                    # 添加识别记录到数据库
                    self.license_plate_db.add_recognition_record(
                        recognized_plate, 
                        confidence, 
                        "无人机识别", 
                        "匹配成功，准备着陆"
                    )
                    return True
            
            self.logger.info(f"[号牌识别] 识别号牌 '{recognized_plate}' 不在数据库中")
            
            # 添加未匹配的识别记录
            self.license_plate_db.add_recognition_record(
                recognized_plate, 
                confidence, 
                "无人机识别", 
                "未在数据库中找到匹配"
            )
            
            return False
            
        except Exception as e:
            self.logger.error(f"[号牌识别] 检查过程中发生错误: {e}")
            return False
