"""
确认号牌控制器 - 无人机精准降落系统

功能：通过识别特定号牌来确认降落目标，实现精准降落
- 使用ArUco标签进行初步对齐
- 通过号牌识别进行最终确认
- 状态机控制整个确认流程
"""

from drone_control.fsm import ControllerBase
from drone_control.utils import ArucoTagProcessor
from .confirm_license_nodes import ConfirmState,SearchState,RepositionState
from drone_control.SQLite import LicensePlateProcessor
import numpy as np

class ConfirmLicenseController(ControllerBase):
    """确认号牌任务控制器 - 管理ArUco对齐和号牌识别流程"""
    def __init__(self, node):
        """初始化确认号牌控制器
        参数：
            node: ROS2节点实例，用于访问日志和其他处理器
        """
        super().__init__(node)
        self.license_config ={
            'target_timeout': 1.0,
            'image_frame': np.array([1920,1080]),       # RGB图像尺寸
            'detection_frame': np.array([640,360])      # 检测框尺寸（三分之一）
        }

        # 获取号牌识别处理器 - 用于最终确认
        self.license_plate_processor = LicensePlateProcessor(self.node.get_logger(),self.license_config)
        self.license_plate_processor.add_license_plate_to_database("闽DA01010330",90,90,12)  # 添加测试门牌数据
        self.license_plate_processor.add_license_plate_to_database("闽DA01010230",90,90,12)  # 添加测试门牌数据
        
        self.license_plate_processor.set_target_license_plate("闽DA01010330")  # 设置目标门牌
        
        # 初始化移动方向变量，默认向上飞行
        self.move_axis, self.move_direction = ('z', -1.0)

    def _register_states(self):
        """注册状态机状态 - 定义确认流程的三个主要状态"""
        # 搜索状态：寻找稳定门牌
        self.state_machine.register_state("search", SearchState(self))
        # 确认状态：识别门牌并确认匹配
        self.state_machine.register_state("confirm", ConfirmState(self))
        # 重新定位状态：确认不匹配时重新定位
        self.state_machine.register_state("reposition", RepositionState(self))

    async def start(self):
        """启动确认号牌任务 - 进入确认流程
        
        流程：
        1. 解锁无人机
        2. 启动板外控制模式
        3. 设置确认开始标志
        4. 切换到确认状态（跳过搜索和对齐）
        """
        if not await self.status_monitor.check_armed():
            await self.mavsdk_controller.arm()
            await self.mavsdk_controller.takeoff(1.0 ,10.0)
        await self.mavsdk_controller.start_offboard()         # 启动板外控制模式
        self.drone_state.confirm_started = True               # 设置确认开始标志
        await self.switch_state("search")                   # 进入搜索状态
    
    async def update(self):
        """确认号牌控制器的主更新循环 - 每帧调用"""
        # 调用父类更新逻辑（状态机更新）
        await super().update()
    
    def update_recognized_plate(self,plate_no,center_x,center_y):
        """更新当前识别到的号牌状态"""
        self.license_plate_processor.update_recognized_plate(plate_no,center_x,center_y)

    async def switch_to_land(self):
        """切换到精准降落状态 - 确认完成后的下一步"""
        # 启动精准降落控制器
        await self.node.precision_land_controller.start()
        # 重置状态标志
        self.drone_state.confirm_started = False
        self.drone_state.search_started = True

    