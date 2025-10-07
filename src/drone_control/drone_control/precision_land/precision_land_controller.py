# drone_control/flight_controller/precision_land_controller.py
"""
精准降落控制器 - 基于ArUco标签的视觉引导降落系统

功能概述：
1. 使用前置引导标记引导无人机接近降落平台
2. 使用下方平台标记进行最终精准对准
3. 状态机控制整个降落流程（搜索→对准→接近→最终对准→下降→完成）
4. 提供双标签配置支持不同阶段的引导需求

状态流转：search → align_on_guide_tag → approach_guide_tag → align_on_platform_tag → descend → finished
"""
from drone_control.fsm import ControllerBase
from .precision_land_nodes import SearchState, AlignOnPlatformTagState,\
    AlignOnGuideTagState, ApproachGuideTagState, DescendState, FinishedState,LandState
from drone_control.utils import ArucoTagProcessor
import numpy as np

class PrecisionLandController(ControllerBase):
    """
    精准降落控制器 - 管理双ArUco标签引导的降落流程
    
    主要功能：
    - 配置和管理前置引导标记与下方平台标记
    - 协调状态机执行搜索、对准、接近、最终对准、下降等状态
    - 提供标签检测数据更新接口
    - 控制无人机起飞和离板模式启动
    
    属性：
        _tag_front_config: 前置引导标记配置（引导接近阶段）
        _tag_down_config: 下方平台标记配置（最终对准阶段）  
        _tag_front_processor: 前置标记ArUco处理器
        _tag_down_processor: 下方标记ArUco处理器
    """
    def __init__(self, node):
        """
        初始化精准降落控制器
        
        参数：
            node: ROS2节点实例，提供状态监控和MAVSDK控制器访问
            
        初始化内容：
        - 配置前置引导标记参数（引导无人机接近平台）
        - 配置下方平台标记参数（最终精准对准）
        - 创建双ArUco标签处理器实例
        """
        # 调用父类构造函数
        super().__init__(node)
        # 前置引导标记配置（引导无人机接近平台）
        self._tag_front_config ={
            'tag_name': 'land_front',                    # 标记名称标识
            'rotation_matrix': np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]),  # 相机到机体的旋转矩阵
            'alignment_axes': ['y','z'],                # 对准轴：Y轴（左右）和Z轴（高度）
            'aligned_tolerance': 0.2,                 # 对准容差：0.2米
            'yaw_tolerance_deg': 15.0,                  # 偏航角容差：15度
            'max_yaw_rate_deg_s': 10.0,                 # 最大偏航角速度：10度/秒
            'min_yaw_rate_deg_s': 2.0,                  # 最小偏航角速度：2度/秒
            'target_timeout': 0.5,                      # 目标超时时间：0.5秒
            'target_offset': np.array([1.4, 0.0, 0.0]), # 目标偏移：前方1.4米，左右0.0米，上下0.0米
            'kp': 0.4,                                  # PID比例系数
            'ki': 0.0,                                  # PID积分系数
            'kd': 0.0,                                  # PID微分系数
            'max_speed': 0.4                            # 最大移动速度：0.4米/秒
        }
        # 下方平台标记配置（最终精准对准）
        self._tag_down_config ={
            'tag_name': 'land_down',                    # 标记名称标识
            'max_detection_distance': np.array([np.inf, np.inf, 2.5]),  # 最大检测距离(XYZ轴)
            'rotation_matrix': np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]),  # 相机到机体的旋转矩阵
            'alignment_axes': ['x','y'],                # 对准轴：X轴（前后）和Y轴（左右）
            'aligned_tolerance': 0.12,                # 对准容差：0.12米（更严格）
            'target_timeout': 3.0,                      # 目标超时时间：3秒
            'target_offset': np.array([0, 0, 0]),       # 目标偏移：无偏移，精准对准
            'kp': 0.4,                                  # PID比例系数
            'ki': 0.0,                                  # PID积分系数
            'kd': 0.0,                                  # PID微分系数
            'max_speed': 0.4                            # 最大移动速度：0.4米/秒
        }

        # 创建ArUco标记处理器实例
        # 前置处理器：处理引导接近阶段的标记检测和对准
        self._tag_front_processor = ArucoTagProcessor(self._tag_front_config,self.logger)
        # 下方处理器：处理最终精准对准阶段的标记检测和对准
        self._tag_down_processor = ArucoTagProcessor(self._tag_down_config,self.logger)
        

    def _register_states(self):
        """
        注册状态机状态 - 定义精准降落的完整流程
        
        状态说明：
        - search: 搜索ArUco引导标记
        - align_on_guide_tag: 对准前置引导标记（调整YZ轴和偏航角）
        - approach_guide_tag: 接近引导标记（向前移动到平台上方）
        - align_on_platform_tag: 对准平台标记（最终XY平面精对准）
        - descend: 下降状态（垂直降落并保持XY对准）
        - finished: 完成状态（降落成功，清理资源）
        - land: 降落状态（无标记时的安全降落）
        """
        self.state_machine.register_state("search", SearchState(self))
        self.state_machine.register_state("align_on_guide_tag", AlignOnGuideTagState(self))
        self.state_machine.register_state("approach_guide_tag", ApproachGuideTagState(self))
        self.state_machine.register_state("align_on_platform_tag", AlignOnPlatformTagState(self))
        self.state_machine.register_state("descend", DescendState(self))
        self.state_machine.register_state("finished", FinishedState(self))
        self.state_machine.register_state("land", LandState(self))

    async def start(self):
        """
        启动精准降落任务
        """
        self.logger.info("启动精准降落状态机...")
        # 如果无人机未解锁，执行解锁和起飞到1米高度
        if not await self.status_monitor.check_armed():
            await self.mavsdk_controller.arm()
            await self.mavsdk_controller.takeoff(1.0 ,10.0)
        # 启动离板控制模式，允许外部控制无人机运动
        await self.mavsdk_controller.start_offboard()
        # 设置搜索开始标志，确保主循环会调用update方法
        self.drone_state.search_started = True
        # 切换到搜索状态，开始ArUco标记搜索
        await self.state_machine.switch_state("search")
    
    async def update(self):
        """
        精准降落控制器的主更新循环
        """
        # 更新前置标记检测有效性，用于状态机判断
        self._tag_front_processor.update_and_log_validity()
        # 更新下方标记检测有效性，用于状态机判断
        self._tag_down_processor.update_and_log_validity()
        # 调用父类更新，执行当前状态机的on_update方法
        await super().update()
    
    def update_front_tag(self,position_cam,orientation_cam_quat):
        """
        更新前置引导标记检测结果
        """
        self._tag_front_processor.process_tag_detection(position_cam,orientation_cam_quat)
    
    def update_down_tag(self,position_cam,orientation_cam_quat):
        """
        更新下方平台标记检测结果
        """
        self._tag_down_processor.process_tag_detection(position_cam,orientation_cam_quat)
