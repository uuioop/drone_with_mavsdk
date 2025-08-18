#!/usr/bin/env python3
"""
精准降落集成测试脚本
"""

import pytest
import asyncio
import numpy as np
from unittest.mock import Mock, AsyncMock, patch

# 导入待测试的模块
from drone_control.core.offboard_navigation import OffboardNavigationController
from drone_control.core.precision_land import PrecisionLand
from drone_control.core.aruco_tracker import ArucoTracker


class TestPrecisionLandingIntegration:
    """测试精准降落集成功能"""
    
    def setup_method(self):
        """测试前的设置"""
        self.mock_drone = Mock()
        self.mock_logger = Mock()
        self.mock_drone_state = Mock()
        self.mock_license_plate_result = Mock()
        
        # 设置异步mock方法
        self.mock_drone.action.arm = AsyncMock()
        self.mock_drone.action.land = AsyncMock()
        self.mock_drone.offboard.set_position_ned = AsyncMock()
        self.mock_drone.offboard.set_velocity_ned = AsyncMock()
        self.mock_drone.offboard.start = AsyncMock()
        self.mock_drone.offboard.stop = AsyncMock()
        
        # 模拟drone_state返回值
        self.mock_drone_state.calculate_ned_from_origin.return_value = (0.0, 0.0, -5.0)
        self.mock_drone_state.calculate_target_ned_from_origin.return_value = (10.0, 10.0, -10.0)
        
        # 创建ArUco跟踪器
        self.aruco_tracker = ArucoTracker(
            self.mock_drone,
            self.mock_logger,
            self.mock_drone_state
        )
        
        # 创建导航控制器
        self.nav_controller = OffboardNavigationController(
            self.mock_drone,
            self.mock_logger,
            self.mock_drone_state,
            self.mock_license_plate_result,
            self.aruco_tracker
        )
    
    def test_initialization_with_aruco_tracker(self):
        """测试带ArUco跟踪器的初始化"""
        assert self.nav_controller.aruco_tracker is not None
        assert self.nav_controller.precision_land is not None
        assert self.aruco_tracker.precision_land_callback is not None
    
    def test_initialization_without_aruco_tracker(self):
        """测试不带ArUco跟踪器的初始化"""
        nav_controller = OffboardNavigationController(
            self.mock_drone,
            self.mock_logger,
            self.mock_drone_state,
            self.mock_license_plate_result
        )
        assert nav_controller.aruco_tracker is None
        assert nav_controller.precision_land is None
    
    @pytest.mark.asyncio
    async def test_execute_landing_with_precision_land(self):
        """测试带精准降落的降落执行"""
        # 模拟精准降落成功
        with patch.object(self.nav_controller.precision_land, 'execute_precision_landing', 
                         return_value=True) as mock_precision_land:
            await self.nav_controller.execute_landing()
            
            # 验证精准降落被调用
            mock_precision_land.assert_called_once()
            # 验证板外模式被启动
            self.mock_drone.offboard.start.assert_called()
    
    @pytest.mark.asyncio
    async def test_execute_landing_precision_land_failure(self):
        """测试精准降落失败时的回退"""
        # 模拟精准降落失败
        with patch.object(self.nav_controller.precision_land, 'execute_precision_landing', 
                         return_value=False) as mock_precision_land:
            await self.nav_controller.execute_landing()
            
            # 验证精准降落被调用
            mock_precision_land.assert_called_once()
            # 验证回退到普通降落
            self.mock_drone.action.land.assert_called()
    
    @pytest.mark.asyncio
    async def test_execute_landing_without_precision_land(self):
        """测试不带精准降落的降落执行"""
        nav_controller = OffboardNavigationController(
            self.mock_drone,
            self.mock_logger,
            self.mock_drone_state,
            self.mock_license_plate_result
        )
        
        await nav_controller.execute_landing()
        
        # 验证直接使用普通降落
        self.mock_drone.action.land.assert_called()
    
    def test_aruco_callback_integration(self):
        """测试ArUco回调集成"""
        # 模拟ArUco检测到标记
        position = np.array([1.0, 2.0, 3.0])
        orientation = np.array([0.0, 0.0, 0.0, 1.0])
        
        # 激活搜索模式
        self.nav_controller.precision_land.search_started = True
        
        # 调用回调函数
        callback = self.aruco_tracker.precision_land_callback
        assert callback is not None
        
        callback(position, orientation)
        
        # 验证目标被更新
        assert self.nav_controller.precision_land.tag.is_valid()
    
    @pytest.mark.asyncio
    async def test_full_navigation_with_precision_landing(self):
        """测试完整的导航和精准降落流程"""
        # 模拟observe_is_in_air函数
        with patch('drone_control.utils.utils.observe_is_in_air', new_callable=AsyncMock) as mock_observe:
            with patch.object(self.nav_controller.precision_land, 'execute_precision_landing', 
                             return_value=True) as mock_precision_land:
                
                # 执行导航
                await self.nav_controller.navigate_to_position(10.0, 10.0, -10.0)
                
                # 验证关键步骤被调用
                self.mock_drone.action.arm.assert_called_once()
                self.mock_drone.offboard.start.assert_called()
                mock_precision_land.assert_called_once()


class TestParameterConfiguration:
    """测试参数配置"""
    
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
        
        self.precision_land = PrecisionLand(
            self.mock_drone,
            self.mock_logger,
            self.mock_drone_state,
            self.aruco_tracker
        )
    
    def test_aruco_parameter_configuration(self):
        """测试ArUco参数配置"""
        # 测试默认参数
        assert self.aruco_tracker.aruco_id == 0
        assert self.aruco_tracker.marker_size == 0.5
        
        # 测试参数设置
        self.aruco_tracker.set_aruco_parameters(
            aruco_id=1,
            dictionary=0,
            marker_size=0.3
        )
        assert self.aruco_tracker.aruco_id == 1
        assert self.aruco_tracker.marker_size == 0.3
    
    def test_precision_land_parameter_configuration(self):
        """测试精准降落参数配置"""
        # 测试默认参数
        assert self.precision_land.param_descent_vel == 1.0
        assert self.precision_land.param_vel_p_gain == 1.5
        assert self.precision_land.param_max_velocity == 3.0
        
        # 测试参数设置
        self.precision_land.set_parameters(
            descent_vel=0.8,
            vel_p_gain=2.0,
            max_velocity=2.5,
            target_timeout=5.0
        )
        assert self.precision_land.param_descent_vel == 0.8
        assert self.precision_land.param_vel_p_gain == 2.0
        assert self.precision_land.param_max_velocity == 2.5
        assert self.precision_land.param_target_timeout == 5.0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
