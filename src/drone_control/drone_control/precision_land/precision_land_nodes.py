# drone_control/flight_controller/precision_land_states.py
import time
from drone_control.fsm import ControllerBaseState
from typing import TYPE_CHECKING
import numpy as np

# 类型检查导入 - 为IDE提供精确的类型提示
if TYPE_CHECKING:
    from .precision_land_controller import PrecisionLandController

class SearchState(ControllerBaseState['PrecisionLandController']):
    """
    搜索状态 - 寻找ArUco引导标记并缓慢下降
    
    功能：
    - 控制无人机缓慢下降并搜索前置ArUco引导标记
    - 设置搜索超时机制，避免无限期搜索
    - 发现标记后立即切换到对准状态
    
    状态流转：
    - 发现前置引导标记 → AlignOnGuideTagState
    - 搜索超时 → LandState（应急降落）
    """
    async def on_enter(self):
        """
        进入搜索状态 - 初始化搜索参数
        """
        self.owner.logger.info("[PL] 进入：搜索状态")
        self.total_search_time = 15.0  # 总搜索时间：15秒
        self._last_search_time = time.time()  # 记录搜索开始时间

    async def on_update(self):
        """搜索状态主循环：旋转搜索引导标记"""
        current_search_time = time.time()
        # 检查搜索是否超时（15秒），超时则切换到应急降落状态
        if current_search_time - self._last_search_time > self.total_search_time:
            self.owner.logger.info("[PL] 搜索不到引导标记，等待后续命令")
            await self.owner.switch_state('land')
        else:
            # 默认控制指令：缓慢下降，其他轴保持静止
            vx,vy,vz,yaw_rate = 0.0,0.0,-0.5,0.0
        
        # 优先检测前置引导标记是否有效
        if self.owner._tag_front_processor.is_valid():
                self.owner.logger.info("[PL] 发现引导标记，开始对准...")
                await self.owner.switch_state('align_on_guide_tag')

        await self.owner.mavsdk_controller.set_velocity_body(vx,vy,vz,yaw_rate)
        # 更新时间戳，用于下次循环的超时判断
        self._last_search_time = current_search_time


    async def on_exit(self):
        pass
class AlignOnGuideTagState(ControllerBaseState['PrecisionLandController']):
    """
    对准引导标记状态 - 根据ArUco标签位置调整无人机姿态
    
    功能：
    - 使用前置ArUco引导标记进行位置和姿态调整
    - 主要调整Y轴位置（左右）Z轴（上下）和偏航角
    - 当偏航角误差较大时，优先调整角度而非位置
    
    状态流转：
    - 对准完成 → ApproachGuideTagState
    - 发现下方平台标记 → AlignOnPlatformTagState
    - 标记丢失 → 悬停等待（保持在当前状态）
    """
    async def on_enter(self): 
        self.owner.logger.info("[PL] 進入：对准前置引导标记状态")
        self.tolerances = {
            'x': 6,     # X轴容差：6米
            'z': 0.2,     # Z轴容差：0.2米
            'yaw': 20.0   # 偏航角容差：20度
        }
        self.last_pos_x = None

    async def on_update(self): 
        """对准引导标记状态主循环：调整Y轴和偏航角"""
        # 检查前置引导标记是否有效，无效则悬停等待
        if not self.owner._tag_front_processor.is_valid():
            self.owner.logger.debug("[PL] 对准前置引导标记状态下，搜索不到引导标记，等待后续命令")
            await self.owner.mavsdk_controller.set_velocity_body(0.0,0.0,0.0,0.0)
            return
        
        # 检测下方平台标记，发现则直接切换到最终对准状态
        if self.owner._tag_down_processor.is_valid():
            await self.owner.switch_state('align_on_platform_tag')
            return

        # 检查是否已对准引导标记，对准完成则进入接近状态
        if self.owner._tag_front_processor.is_aligned():
            await self.owner.switch_state('approach_guide_tag')
            return
        
        try:
            # 获取位置误差：Z轴、偏航角误差
            x_err, _, z_err, yaw_err = self.owner._tag_front_processor.get_alignment_errors()
        except ValueError as e:
            self.owner.logger.error(f"[PL] 对准前置引导标记状态, 获取误差向量失败: {e}")
            return
        
        # 计算速度指令，使用引导标记配置
        vx,vy,vz,yaw_rate = self.owner._tag_front_processor.calculate_velocity_command(True)
        
        # 如果偏航角误差较大，则优先调整角度，限制平移速度
        if abs(yaw_err) > self.tolerances['yaw'] :
            vy = 0  # 暂停Y轴移动，专注调整偏航角
            vx = 0  # 限制X轴速度
            vz = np.clip(vz,-0.2,0.2)   # 限制Z轴速度
        
        # 如果Z轴误差较大，优先调整高度，限制其他轴速度
        if abs(z_err) > self.tolerances['z'] :
            vy = np.clip(vy,-0.2,0.2)   # 限制Y轴速度
            vx = 0  # 限制X轴速度

        # 如果X轴误差较大，允许小范围移动
        if abs(x_err) > self.tolerances['x'] :
            vx = np.clip(vx,-0.15,0.15)   # 允许X轴小范围移动
        
        # 发送速度控制指令，实现精确对准
        await self.owner.mavsdk_controller.set_velocity_body(vx,vy,vz,yaw_rate)
        

    async def on_exit(self): 
        pass
class ApproachGuideTagState(ControllerBaseState['PrecisionLandController']):
    """
    接近引导标记状态 - 向前移动到平台上方
    
    功能：
    - 使用前置ArUco引导标记计算前进指令
    - 控制无人机向前移动到平台上方位置
    - 保持对引导标记的对准状态
    - 检测到下方平台标记时可直接切换到最终对准
    
    状态流转：
    - 发现下方平台标记 → AlignOnPlatformTagState
    - 标记丢失 → 悬停等待（保持在当前状态）
    """
    async def on_enter(self): 
        self.owner.logger.info("[PL] 進入：接近前置引导标记状态")
    async def on_update(self): 
        if not self.owner._tag_front_processor.is_valid():
            self.owner.logger.debug("[PL] 接近前置引导标记状态下，搜索不到引导标记，等待后续命令")
            await self.owner.mavsdk_controller.set_velocity_body(0.0,0.0,0.0,0.0)
            return
        
        # 检测下方平台标记，发现则直接切换到最终对准状态
        if self.owner._tag_down_processor.is_valid():
            await self.owner.switch_state('align_on_platform_tag')
            return
        
        # 使用引导标记处理器计算速度指令（包含前进分量）
        vx,vy,vz,yaw_rate = self.owner._tag_front_processor.calculate_velocity_command(True)
        # 发送速度指令，实现向前移动到平台上方
        await self.owner.mavsdk_controller.set_velocity_body(vx,vy,vz,yaw_rate)
    
    async def on_exit(self): 
        pass
class AlignOnPlatformTagState(ControllerBaseState['PrecisionLandController']):
    """
    对准平台标记状态 - 最终XY平面精对准
    
    功能：
    - 使用下方ArUco平台标记进行最终精准对准
    - 主要调整X轴（前后）和Y轴（左右）位置
    - 实现厘米级精度的XY平面对准
    - 为垂直降落做准备，确保无人机在平台正上方
    
    状态流转：
    - 对准完成 → DescendState（开始垂直降落）
    - 标记丢失 → 悬停等待（保持在当前状态）
    """
    async def on_enter(self): 
        self.owner.logger.info("[PL] 進入：对准平台标记状态")
        self.last_pos_z =None
    async def on_update(self): 
        # 检查下方平台标记是否有效，无效则切换到应急降落
        if not self.owner._tag_down_processor.is_valid():
            self.owner.logger.debug("[PL] 对准平台标记状态下，搜索不到平台标记，等待后续命令")
            await self.owner.mavsdk_controller.set_velocity_body(0.0,0.0,0.0,0.0)
            return

        # 检查是否已对准平台标记，对准完成则开始垂直降落
        if self.owner._tag_down_processor.is_aligned():
            await self.owner.switch_state('descend')
            return
        
        # 使用平台标记处理器计算XY平面速度指令（不包含Z轴和偏航）
        vx,vy,_,yaw_rate = self.owner._tag_down_processor.calculate_velocity_command(False)
        # 发送速度指令，专注XY平面对准，Z轴速度设为0
        await self.owner.mavsdk_controller.set_velocity_body(vx,vy,0,yaw_rate)
    async def on_exit(self): 
        pass
class DescendState(ControllerBaseState['PrecisionLandController']):
    """
    下降状态 - 垂直降落并保持XY对准
    
    功能：
    - 控制无人机垂直下降（Z轴速度0.5米/秒）
    - 保持XY平面对准，使用下方平台标记进行微调
    - 检测着陆状态，着陆完成后切换到完成状态
    - 标记丢失时切换到应急降落状态
    
    状态流转：
    - 检测到着陆 → FinishedState（降落成功）
    - 标记丢失 → LandState（应急降落）
    """
    async def on_enter(self): 
        self.owner.logger.info("[PL] 進入：下降状态")
    async def on_update(self): 
        # 检查是否已着陆，着陆完成则切换到完成状态
        if self.owner.drone_state.landed:
            await self.owner.switch_state('finished')
            return
        
        # 检查下方平台标记是否有效，无效则切换到应急降落
        if not self.owner._tag_down_processor.is_valid():
            self.owner.logger.info("[PL] 下降状态下，搜索不到平台标记，切换到应急降落")
            await self.owner.switch_state('land')
            return
        
        # 使用平台标记处理器计算XY平面速度指令（不包含Z轴和偏航）
        vx,vy,_,yaw_rate = self.owner._tag_down_processor.calculate_velocity_command(False)
        vz=0.5
        await self.owner.mavsdk_controller.set_velocity_body(vx,vy,vz,yaw_rate)
    async def on_exit(self): 
        pass
class FinishedState(ControllerBaseState['PrecisionLandController']):
    """
    完成状态 - 精准降落成功完成
    
    功能：
    - 表示精准降落流程已成功完成
    - 停止所有运动，设置零速度指令
    - 作为最终状态，保持无人机在当前位置
    
    状态流转：
    - 无转出状态（最终状态）
    """
    async def on_enter(self): 
        self.owner.drone_state.search_started = False
        await self.owner.mavsdk_controller.stop_offboard()
        await self.owner.mavsdk_controller.disarm()
        self.owner.logger.info("[PL] 進入：完成状态")
    async def on_update(self): pass
    async def on_exit(self): pass

class LandState(ControllerBaseState['PrecisionLandController']):
    """
    降落状态 - 应急降落状态
    """
    async def on_enter(self): 
        self.owner.logger.info("[PL] 進入：降落状态")
        await self.owner.mavsdk_controller.set_velocity_body(0.0, 0.0, 0.5, 0.0)
    async def on_update(self): 
        await self.owner.mavsdk_controller.set_velocity_body(0.0, 0.0, 0.5, 0.0)
    async def on_exit(self): 
        pass
