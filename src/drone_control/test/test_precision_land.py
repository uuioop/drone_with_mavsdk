#!/usr/bin/env python3
"""
精准降落功能测试脚本
"""

import pytest
import asyncio
import numpy as np
from unittest.mock import Mock, AsyncMock, patch

# 导入待测试的模块
from drone_control.core.precision_land import PrecisionLand, PrecisionLandState, ArucoTag
from drone_control.core.aruco_tracker import ArucoTracker


class TestArucoTag:
    """测试ArUco标记数据结构"""
    
    def test_aruco_tag_initialization(self):
        """测试ArUco标记初始化"""
        tag = ArucoTag()
        assert not tag.is_valid()
        assert tag.timestamp == 0.0
        assert np.all(np.isnan(tag.position))
    
    def test_aruco_tag_validity(self):
        """测试ArUco标记有效性检查"""
        tag = ArucoTag()
        tag.position = np.array([1.0, 2.0, 3.0])
        tag.timestamp = 12345.0
        assert tag.is_valid()


class TestPrecisionLand:
    """测试精准降落控制器"""
    
    def setup_method(self):
        """测试前的设置"""
        self.mock_drone = Mock()
        self.mock_logger = Mock()
        self.mock_drone_state = Mock()
        self.mock_aruco_tracker = Mock()
        
        # 模拟drone_state返回值
        self.mock_drone_state.calculate_ned_from_origin.return_value = (0.0, 0.0, -5.0)
        
        self.precision_land = PrecisionLand(
            self.mock_drone,
            self.mock_logger,
            self.mock_drone_state,
            self.mock_aruco_tracker
        )
    
    def test_initialization(self):
        """测试精准降落控制器初始化"""
        assert self.precision_land.state == PrecisionLandState.IDLE
        assert not self.precision_land.search_started
        assert self.precision_land.param_descent_vel == 1.0
        assert self.precision_land.param_vel_p_gain == 1.5
    
    def test_set_parameters(self):
        """测试参数设置"""
        self.precision_land.set_parameters(
            descent_vel=0.8,
            vel_p_gain=2.0,
            max_velocity=2.5
        )
        assert self.precision_land.param_descent_vel == 0.8
        assert self.precision_land.param_vel_p_gain == 2.0
        assert self.precision_land.param_max_velocity == 2.5
    
    def test_update_target_pose(self):
        """测试目标位姿更新"""
        self.precision_land.search_started = True
        position = np.array([1.0, 2.0, 3.0])
        orientation = np.array([0.0, 0.0, 0.0, 1.0])
        
        self.precision_land.update_target_pose(position, orientation)
        
        # 验证目标是否被更新
        assert self.precision_land.tag.is_valid()
    
    def test_check_target_timeout(self):
        """测试目标超时检查"""
        # 测试无效目标
        assert self.precision_land._check_target_timeout() == True
        
        # 测试有效但超时的目标
        self.precision_land.tag.position = np.array([1.0, 2.0, 3.0])
        self.precision_land.tag.timestamp = 0.0  # 很久以前的时间戳
        assert self.precision_land._check_target_timeout() == True
    
    def test_calculate_velocity_setpoint_xy(self):
        """测试XY速度设定值计算"""
        # 设置有效目标
        self.precision_land.tag.position = np.array([1.0, 1.0, -5.0])
        self.precision_land.tag.timestamp = 12345.0
        
        vx, vy = self.precision_land._calculate_velocity_setpoint_xy()
        
        # 验证速度计算结果
        assert isinstance(vx, float)
        assert isinstance(vy, float)
        assert abs(vx) <= self.precision_land.param_max_velocity
        assert abs(vy) <= self.precision_land.param_max_velocity
    
    def test_generate_search_waypoints(self):
        """测试搜索航点生成"""
        self.precision_land._generate_search_waypoints()
        
        assert len(self.precision_land.search_waypoints) > 0
        assert self.precision_land.search_waypoint_index == 0
    
    def test_position_reached(self):
        """测试位置到达检查"""
        target = np.array([0.1, 0.1, -5.0])  # 很接近当前位置
        assert self.precision_land._position_reached(target) == True
        
        target = np.array([10.0, 10.0, -5.0])  # 很远的位置
        assert self.precision_land._position_reached(target) == False
    
    def test_state_switching(self):
        """测试状态切换"""
        initial_state = self.precision_land.state
        self.precision_land._switch_to_state(PrecisionLandState.SEARCH)
        assert self.precision_land.state == PrecisionLandState.SEARCH
        assert self.precision_land.state != initial_state
    
    @pytest.mark.asyncio
    async def test_activate(self):
        """测试激活精准降落模式"""
        await self.precision_land.activate()
        
        assert self.precision_land.search_started == True
        assert self.precision_land.state == PrecisionLandState.SEARCH
        assert len(self.precision_land.search_waypoints) > 0
    
    @pytest.mark.asyncio
    async def test_deactivate(self):
        """测试停用精准降落模式"""
        await self.precision_land.deactivate()
        
        assert self.precision_land.search_started == False
        assert self.precision_land.state == PrecisionLandState.IDLE


class TestArucoTracker:
    """测试ArUco跟踪器"""
    
    def setup_method(self):
        """测试前的设置"""
        self.mock_drone = Mock()
        self.mock_logger = Mock()
        self.mock_drone_state = Mock()
        
        self.aruco_tracker = ArucoTracker(
            self.mock_drone,
            self.mock_logger,
            self.mock_drone_state
        )
    
    def test_initialization(self):
        """测试ArUco跟踪器初始化"""
        assert self.aruco_tracker.aruco_id == 0
        assert self.aruco_tracker.marker_size == 0.5
        assert not self.aruco_tracker.target_found
        assert self.aruco_tracker.precision_land_callback is None
    
    def test_set_aruco_parameters(self):
        """测试ArUco参数设置"""
        self.aruco_tracker.set_aruco_parameters(
            aruco_id=1,
            dictionary=0,
            marker_size=0.3
        )
        assert self.aruco_tracker.aruco_id == 1
        assert self.aruco_tracker.marker_size == 0.3
    
    def test_set_control_parameters(self):
        """测试控制参数设置"""
        self.aruco_tracker.set_control_parameters(
            descend_velocity=-0.8,
            xy_velocity_max=2.0,
            kp_xy=0.8
        )
        assert self.aruco_tracker.descend_velocity == -0.8
        assert self.aruco_tracker.xy_velocity_max == 2.0
        assert self.aruco_tracker.kp_xy == 0.8
    
    def test_set_precision_land_callback(self):
        """测试精准降落回调设置"""
        callback = Mock()
        self.aruco_tracker.set_precision_land_callback(callback)
        assert self.aruco_tracker.precision_land_callback == callback
    
    def test_rot_mat_to_quat(self):
        """测试旋转矩阵到四元数转换"""
        # 单位矩阵应该转换为单位四元数
        identity_matrix = np.eye(3)
        quat = self.aruco_tracker.rot_mat_to_quat(identity_matrix)
        
        assert len(quat) == 4
        assert abs(quat[3] - 1.0) < 1e-6  # w分量应该接近1
        assert abs(quat[0]) < 1e-6  # x分量应该接近0
        assert abs(quat[1]) < 1e-6  # y分量应该接近0
        assert abs(quat[2]) < 1e-6  # z分量应该接近0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
