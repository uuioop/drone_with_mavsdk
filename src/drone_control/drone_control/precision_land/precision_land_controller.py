# drone_control/flight_controller/precision_land_controller.py
from drone_control.fsm import ControllerBase
from .precision_land_nodes import SearchState, AlignOnPlatformTagState,\
    AlignOnGuideTagState, ApproachGuideTagState, DescendState, FinishedState,AutoTuneState,IdleState
from drone_control.utils import ArucoTagProcessor
import numpy as np
from drone_control.utils import PIDAutoTuner

class PrecisionLandController(ControllerBase):
    """
    精準降落控制器 (對應 C++ 的 Player)。
    """
    def __init__(self, node,pid_config):
        # 呼叫父類別的 __init__
        super().__init__(node)
        self._tag_front_config ={
            'tag_name': 'land_front',
            'rotation_matrix': np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]),
            'alignment_axes': ['y','z'],
            'aligned_tolerance': 0.2,
            'yaw_tolerance_deg': 3.0,
            'max_yaw_rate_deg_s': 15.0,
            'min_yaw_rate_deg_s': 2.0,
            'target_timeout': 5.0,
            'target_offset': np.array([1.8, 0, 0]),
            'kp': 0.5,
            'ki': 0.0,
            'kd': 0.0,
            'max_speed': 0.6
        }
        self._tag_down_config ={
            'tag_name': 'land_down',
            'rotation_matrix': np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]),
            'alignment_axes': ['x','y'],
            'aligned_tolerance': 0.2,
            'yaw_tolerance_deg': 3.0,
            'max_yaw_rate_deg_s': 15.0,
            'min_yaw_rate_deg_s': 2.0,
            'target_timeout': 5.0,
            'target_offset': np.array([0, 0, 0]),
            'kp': 0.5,
            'ki': 0.0,
            'kd': 0.0,
            'max_speed': 0.6
        }
        self._tag_front_processor = ArucoTagProcessor(self._tag_front_config,self.logger)
        self._tag_down_processor = ArucoTagProcessor(self._tag_down_config,self.logger)
        self.pid_config=pid_config
        self.autotuner = PIDAutoTuner(self.logger,self.pid_config.get('output_amplitude', 0.5),self.pid_config.get('input_tolerance',0.02))
        self.autotune_axis = 'x'  # 'x' 或 'y'，可以通过外部服务设置
        self.autotune_results = None
        

    def _register_states(self):
        """實現父類別的抽象方法，註冊所有精準降落的狀態。"""
        self.state_machine.register_state("search", SearchState(self))
        self.state_machine.register_state("align_on_guide_tag", AlignOnGuideTagState(self))
        self.state_machine.register_state("approach_guide_tag", ApproachGuideTagState(self))
        self.state_machine.register_state("align_on_platform_tag", AlignOnPlatformTagState(self))
        self.state_machine.register_state("descend", DescendState(self))
        self.state_machine.register_state("finished", FinishedState(self))
        self.state_machine.register_state("autotune", AutoTuneState(self))
        self.state_machine.register_state("idle", IdleState(self))

    async def start(self):
        """啟動精準降落任務。"""
        self.logger.info("啟動精準降落狀態機...")
        await self.state_machine.switch_state("search")
    
    async def start_autotune(self, axis: str):
        """外部调用此方法以启动自动调参。"""
        if axis not in ['x', 'y']:
            self.logger.error(f"无效的调参轴: {axis}。必须是 'x' 或 'y'。")
            return
        self.autotune_axis = axis
        self.logger.info(f"准备启动 {axis} 轴的自动调参...")
        # 启动自动调参需要无人机先解锁并进入offboard模式
        await self.mavsdk_controller.start_offboard()
        self.drone_state.search_started = True # 确保主循环会调用update
        await self.state_machine.switch_state("autotune")
        
    async def update(self):
        """精準降落控制器的主更新迴圈。"""
        self._tag_front_processor.update_and_log_validity()
        self._tag_down_processor.update_and_log_validity()
        await super().update()
    
    def update_front_tag(self,position_cam,orientation_cam_quat):
        self._tag_front_processor.process_tag_detection(position_cam,orientation_cam_quat)
    
    def update_down_tag(self,position_cam,orientation_cam_quat):
        self._tag_down_processor.process_tag_detection(position_cam,orientation_cam_quat)
