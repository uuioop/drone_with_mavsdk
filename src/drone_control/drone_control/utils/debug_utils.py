#!/usr/bin/env python3
"""
调试工具模块 - 集成 aruco_tracker 和 precision_land 的调试功能
"""

import cv2
import numpy as np
import time
from typing import Optional, Tuple, Dict, Any
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json


class DebugVisualizer:
    """调试可视化工具类，提供图像标注和状态显示功能"""
    
    def __init__(self, logger):
        self.logger = logger
        self.font_face = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.7
        self.thickness = 2
        self.colors = {
            'green': (0, 255, 0),
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'yellow': (0, 255, 255),
            'white': (255, 255, 255),
            'cyan': (255, 255, 0),
            'magenta': (255, 0, 255)
        }
        
    def annotate_aruco_detection(self, image: np.ndarray, tvec: np.ndarray, 
                                quat: Optional[np.ndarray] = None, 
                                marker_id: Optional[int] = None) -> np.ndarray:
        """
        在图像上标注 ArUco 检测信息（类似原始 aruco_tracker 的 annotate_image 功能）
        
        Args:
            image: 输入图像
            tvec: 平移向量 (X, Y, Z)
            quat: 四元数 (可选)
            marker_id: 标记ID (可选)
        
        Returns:
            标注后的图像
        """
        annotated_image = image.copy()
        
        # 计算距离
        distance = np.linalg.norm(tvec)
        
        # 准备文本信息
        texts = []
        texts.append(f"X: {tvec[0]:.2f} Y: {tvec[1]:.2f} Z: {tvec[2]:.2f}")
        texts.append(f"Distance: {distance:.2f}m")
        
        if marker_id is not None:
            texts.append(f"ID: {marker_id}")
            
        if quat is not None:
            # 将四元数转换为欧拉角显示
            from scipy.spatial.transform import Rotation
            r = Rotation.from_quat([quat[0], quat[1], quat[2], quat[3]])
            euler = r.as_euler('xyz', degrees=True)
            texts.append(f"Roll: {euler[0]:.1f}° Pitch: {euler[1]:.1f}° Yaw: {euler[2]:.1f}°")
        
        # 在图像右下角显示信息
        y_offset = 30
        for i, text in enumerate(texts):
            text_size = cv2.getTextSize(text, self.font_face, self.font_scale, self.thickness)[0]
            x = annotated_image.shape[1] - text_size[0] - 10
            y = annotated_image.shape[0] - (len(texts) - i) * y_offset + 10
            
            # 绘制背景矩形
            cv2.rectangle(annotated_image, 
                         (x - 5, y - 20), 
                         (x + text_size[0] + 5, y + 5), 
                         (0, 0, 0), -1)
            
            # 绘制文本
            cv2.putText(annotated_image, text, (x, y), 
                       self.font_face, self.font_scale, 
                       self.colors['yellow'], self.thickness, cv2.LINE_AA)
        
        return annotated_image
    
    def annotate_precision_land_state(self, image: np.ndarray, 
                                    state: str, 
                                    target_info: Optional[Dict] = None,
                                    control_info: Optional[Dict] = None) -> np.ndarray:
        """
        在图像上标注精准降落状态信息（类似原始 precision_land 的调试功能）
        
        Args:
            image: 输入图像
            state: 当前状态
            target_info: 目标信息字典
            control_info: 控制信息字典
        
        Returns:
            标注后的图像
        """
        annotated_image = image.copy()
        
        # 状态信息显示在左上角
        texts = [f"State: {state}"]
        
        if target_info:
            if 'visible' in target_info:
                status = "VISIBLE" if target_info['visible'] else "LOST"
                color = self.colors['green'] if target_info['visible'] else self.colors['red']
                texts.append(f"Target: {status}")
            
            if 'distance' in target_info:
                texts.append(f"Target Dist: {target_info['distance']:.2f}m")
                
            if 'last_seen' in target_info:
                texts.append(f"Last Seen: {target_info['last_seen']:.1f}s ago")
        
        if control_info:
            if 'velocity_cmd' in control_info:
                vel = control_info['velocity_cmd']
                texts.append(f"Vel Cmd: X:{vel[0]:.2f} Y:{vel[1]:.2f} Z:{vel[2]:.2f}")
            
            if 'altitude' in control_info:
                texts.append(f"Altitude: {control_info['altitude']:.2f}m")
        
        # 绘制状态信息
        for i, text in enumerate(texts):
            y = 30 + i * 25
            
            # 绘制背景
            text_size = cv2.getTextSize(text, self.font_face, self.font_scale, self.thickness)[0]
            cv2.rectangle(annotated_image, 
                         (5, y - 20), 
                         (15 + text_size[0], y + 5), 
                         (0, 0, 0), -1)
            
            # 选择颜色
            color = self.colors['green'] if i == 0 else self.colors['white']
            if i == 1 and target_info and 'visible' in target_info:
                color = self.colors['green'] if target_info['visible'] else self.colors['red']
            
            cv2.putText(annotated_image, text, (10, y), 
                       self.font_face, self.font_scale, 
                       color, self.thickness, cv2.LINE_AA)
        
        return annotated_image
    
    def draw_search_pattern(self, image: np.ndarray, waypoints: list, 
                           current_index: int = -1) -> np.ndarray:
        """
        在图像上绘制搜索路径（类似原始 precision_land 的搜索模式可视化）
        
        Args:
            image: 输入图像
            waypoints: 搜索航点列表
            current_index: 当前航点索引
        
        Returns:
            标注后的图像
        """
        if not waypoints:
            return image
            
        annotated_image = image.copy()
        
        # 在图像右上角绘制小地图
        map_size = 150
        map_x = annotated_image.shape[1] - map_size - 10
        map_y = 10
        
        # 绘制地图背景
        cv2.rectangle(annotated_image, 
                     (map_x, map_y), 
                     (map_x + map_size, map_y + map_size), 
                     (50, 50, 50), -1)
        
        # 计算缩放比例
        if len(waypoints) > 1:
            x_coords = [wp[0] for wp in waypoints]
            y_coords = [wp[1] for wp in waypoints]
            x_range = max(x_coords) - min(x_coords)
            y_range = max(y_coords) - min(y_coords)
            max_range = max(x_range, y_range) if max(x_range, y_range) > 0 else 1
            scale = (map_size - 20) / max_range
            
            center_x = (max(x_coords) + min(x_coords)) / 2
            center_y = (max(y_coords) + min(y_coords)) / 2
            
            # 绘制航点和路径
            for i, wp in enumerate(waypoints):
                # 转换到地图坐标
                px = int(map_x + map_size/2 + (wp[0] - center_x) * scale)
                py = int(map_y + map_size/2 + (wp[1] - center_y) * scale)
                
                # 绘制航点
                color = self.colors['red'] if i == current_index else self.colors['blue']
                cv2.circle(annotated_image, (px, py), 3, color, -1)
                
                # 绘制连线
                if i > 0:
                    prev_wp = waypoints[i-1]
                    prev_px = int(map_x + map_size/2 + (prev_wp[0] - center_x) * scale)
                    prev_py = int(map_y + map_size/2 + (prev_wp[1] - center_y) * scale)
                    cv2.line(annotated_image, (prev_px, prev_py), (px, py), 
                            self.colors['cyan'], 1)
        
        # 添加标题
        cv2.putText(annotated_image, "Search Pattern", 
                   (map_x, map_y - 5), 
                   self.font_face, 0.5, self.colors['white'], 1)
        
        return annotated_image


class DebugDataCollector:
    """调试数据收集器，用于收集和发布调试信息"""
    
    def __init__(self, logger, node):
        self.logger = logger
        self.node = node
        self.start_time = time.time()
        
        # 创建调试信息发布者
        self.debug_info_pub = node.create_publisher(String, '~/debug_info', 10)
        self.target_pose_debug_pub = node.create_publisher(PoseStamped, '~/target_pose_debug', 10)
        
    def publish_aruco_debug_info(self, detection_result: Dict[str, Any]):
        """发布 ArUco 检测调试信息"""
        debug_msg = String()
        debug_info = {
            'timestamp': time.time() - self.start_time,
            'type': 'aruco_detection',
            'data': detection_result
        }
        debug_msg.data = json.dumps(debug_info, indent=2)
        self.debug_info_pub.publish(debug_msg)
        
    def publish_precision_land_debug_info(self, state: str, control_data: Dict[str, Any]):
        """发布精准降落调试信息"""
        debug_msg = String()
        debug_info = {
            'timestamp': time.time() - self.start_time,
            'type': 'precision_land',
            'state': state,
            'data': control_data
        }
        debug_msg.data = json.dumps(debug_info, indent=2)
        self.debug_info_pub.publish(debug_msg)
        
    def publish_target_pose_debug(self, tvec: np.ndarray, quat: np.ndarray, frame_id: str = "camera_frame"):
        """发布目标位姿调试信息（类似原始 aruco_tracker 的 target_pose 发布）"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame_id
        
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])
        
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        
        self.target_pose_debug_pub.publish(pose_msg)


class PerformanceMonitor:
    """性能监控器，用于监控处理性能"""
    
    def __init__(self, logger):
        self.logger = logger
        self.timing_data = {}
        
    def start_timing(self, operation_name: str):
        """开始计时"""
        self.timing_data[operation_name] = time.time()
        
    def end_timing(self, operation_name: str) -> float:
        """结束计时并返回耗时"""
        if operation_name in self.timing_data:
            duration = time.time() - self.timing_data[operation_name]
            del self.timing_data[operation_name]
            return duration
        return 0.0
        
    def log_performance(self, operation_name: str, duration: float, threshold: float = 0.1):
        """记录性能信息"""
        if duration > threshold:
            self.logger.warning(f"[PERF] {operation_name} 耗时过长: {duration:.3f}s")
        else:
            self.logger.debug(f"[PERF] {operation_name} 耗时: {duration:.3f}s")
