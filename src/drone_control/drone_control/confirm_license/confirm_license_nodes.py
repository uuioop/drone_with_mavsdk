"""
确认号牌状态节点 - 无人机精准降落系统的状态机实现

包含一个主要状态：

- ConfirmState: 识别并确认号牌匹配

状态流转：Confirm -> PrecisionLand状态机
"""

from drone_control.fsm import ControllerBaseState
import time
from typing import TYPE_CHECKING
from drone_control.SQLite.license_plate_process import PlateMatchResult

# 类型检查导入 - 为IDE提供精确的类型提示
if TYPE_CHECKING:
    from .confirm_license_controller import ConfirmLicenseController

class SearchState(ControllerBaseState['ConfirmLicenseController']):
    """搜索状态 - 无人机搜索目标号牌"""
    
    async def on_enter(self):
        """进入搜索状态 - 初始化搜索参数"""
        self.owner.logger.info('[CL] SearchState on_enter')

    async def on_update(self):
        """搜索状态更新 - 检查是否找到目标号牌"""
        vx,vy,vz,yaw_rate = 0.0, 0.0, -0.5, 0.0
        if self.owner.license_plate_processor.is_valid():
            # 获取当前稳定识别到的号牌
            plate_to_check = self.owner.license_plate_processor.get_stable_license_plate()
            # 检查号牌是否已被拒绝
            if plate_to_check in self.owner.license_plate_processor.rejected_plates:
                self.owner.logger.debug(f"[CL] 忽略已在黑名单中的号牌: {plate_to_check}")
            elif self.owner.license_plate_processor.is_aligned():
                self.owner.logger.info(f"[CL] 发现新的有效号牌 {plate_to_check}，准备确认。")
                await self.owner.switch_state('confirm')
                return
            else:
                # 左右未对齐时，根据Y轴速度指令调整位置
                vy = self.owner.license_plate_processor.calculate_velocity(['y'])[0]
        
        await self.owner.mavsdk_controller.set_velocity_body(vx, vy, vz, yaw_rate)

    async def on_exit(self):
        """离开搜索状态 - 停止搜索"""
        await self.owner.mavsdk_controller.set_velocity_body(0.0, 0.0, 0.0, 0.0)

class ConfirmState(ControllerBaseState['ConfirmLicenseController']):
    """确认状态 - 识别号牌并确认是否匹配目标"""
    async def on_enter(self):
        """进入确认状态 - 初始化时间戳并开始等待识别结果"""
        self.owner.logger.info('[CL] ConfirmState on_enter')
        # 清空之前的拒绝记录
        self.owner.license_plate_processor.clear_rejected_plate()
        self.entry_time = time.time()  # 记录进入时间，用于延时等待识别结果
        self.timeout = 2.5

    async def on_update(self):
        """确认状态更新 - 处理号牌识别结果并决定下一步动作"""
        if not self.owner.license_plate_processor.is_valid():
            await self.owner.switch_state('search')
            return
        
        # 延时等待，给无人机一个平缓状态去识别门牌
        if time.time() - self.entry_time <= self.timeout:
            await self.owner.mavsdk_controller.set_velocity_body(0.0, 0.0, 0.0, 0.0)
            return
        
        # 检查是否完全匹配目标号牌
        if self.owner.license_plate_processor.compare_plate():
            self.owner.logger.info(f'[CL] 识别到目标门牌: {self.owner.license_plate_processor.get_stable_license_plate()}，确认匹配。')
            await self.owner.switch_to_land()
            return
        else:
            rejected_plate = self.owner.license_plate_processor.get_stable_license_plate()
            self.owner.logger.info(f'[CL] 识别到非目标门牌: {rejected_plate}，将其加入临时黑名单。')
            # 将被拒绝的号牌添加到黑名单中
            self.owner.license_plate_processor.rejected_plates.add(rejected_plate)
            await self.owner.switch_state('reposition')
            return

    async def on_exit(self):
        """离开确认状态"""
        pass
class RepositionState(ControllerBaseState['ConfirmLicenseController']):
    """重新定位状态 - 无人机重新定位到目标位置"""
    async def on_enter(self):
        """进入重新定位状态 - 初始化重新定位参数"""
        self.owner.logger.info('[CL] RepositionState on_enter: 开始重新定位')
        self.entry_time = time.time()  # 记录进入时间，用于延时等待重新定位完成
        self.timeout = 1.4
        # 获取移动方向指令
        self.move_axis, self.move_direction = self.owner.license_plate_processor.calculate_movement_from_plate_diff()
        
        # 如果没有有效的方向指令，使用默认的向上飞行
        if self.move_axis is None:
            self.move_axis, self.move_direction = ('z', -1.0)  # 默认向上飞
            self.owner.logger.info(f'[CL] 未获取到有效方向指令，使用默认向上飞行')
        else:
            self.owner.logger.info(f'[CL] 确定移动方向: {self.move_axis}轴，方向: {self.move_direction}')
        
        # 设置移动速度
        self.move_speed = 0.5  # 设置一个固定的移动速度
    
    async def on_update(self):
        """重新定位状态更新 - 检查是否重新定位完成"""
        # 延时等待，给无人机一个飞行状态去重新定位
        if time.time() - self.entry_time > self.timeout:
            await self.owner.switch_state('search')
            return
        # 初始化速度向量
        vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0
        
        # 根据确定的轴和方向设置相应的速度分量
        if self.move_axis == 'y':
            vy = self.move_direction * self.move_speed
        elif self.move_axis == 'z':
            vz = self.move_direction * self.move_speed
        
        # 执行移动
        await self.owner.mavsdk_controller.set_velocity_body(vx, vy, vz, yaw_rate)

    async def on_exit(self):
        """离开重新定位状态"""
        pass