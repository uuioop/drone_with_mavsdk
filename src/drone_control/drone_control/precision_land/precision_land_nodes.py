# drone_control/flight_controller/precision_land_states.py
import time
from drone_control.fsm import ControllerBaseState
from typing import TYPE_CHECKING
import numpy as np

# 為了讓 VSCode 等 IDE 提供精確的型別提示
if TYPE_CHECKING:
    from .precision_land_controller import PrecisionLandController

class SearchState(ControllerBaseState['PrecisionLandController']):
    async def on_enter(self):
        self.owner.logger.info("[PL 狀態] 進入：搜索模式")
        await self.owner.mavsdk_controller.arm()
        await self.owner.mavsdk_controller.start_offboard()
        self.owner.drone_state.search_started = True

    async def on_update(self):
        if self.owner._tag_front_processor.is_valid():
            vx,vy,vz,yaw_rate = self.owner._tag_front_processor.calculate_velocity_command(True)
        else:
            vx,vy,vz,yaw_rate = 0.0,0.0,-1.0,0.0
        # 优先寻找前置引导标记
        if self.owner._tag_front_processor.is_valid():
                self.owner.logger.info("[PL] 发现引导标记，开始对准...")
                await self.owner.switch_state('align_on_guide_tag')
        # 后面写判断是否角度误差大，才能直接进入平台对准
        # elif is_down_valid:
        #     # 如果直接看到了平台标记，也可以直接开始对准
        #     self.logger.info("[PL] 直接发现平台标记，开始对准...")
        #     self._switch_state('align_on_platform_tag')
        await self.owner.mavsdk_controller.set_velocity_body(vx,vy,vz,yaw_rate)

    async def on_exit(self):
        self.owner.logger.info("[PL 狀態] 離開：搜索模式")

class AlignOnGuideTagState(ControllerBaseState['PrecisionLandController']):
    async def on_enter(self): 
        self.owner.logger.info("[PL 狀態] 進入：对准前置引导标记模式")
    async def on_update(self): 
        if not self.owner._tag_front_processor.is_valid():
            await self.owner.switch_state('search')
            return
        _,vy,vz,yaw_rate = self.owner._tag_front_processor.calculate_velocity_command(True)
        if self.owner._tag_front_processor.is_aligned():
            await self.owner.switch_state('approach_guide_tag')
            return
        await self.owner.mavsdk_controller.set_velocity_body(0.0,vy,vz,yaw_rate)
        # print(f"[PL 狀態] 对准前置引导标记模式, vy: {vy:.2f}, vz: {vz:.2f}, yaw_rate: {yaw_rate:.2f}")
    async def on_exit(self): 
        self.owner.logger.info("[PL 狀態] 離開：对准前置引导标记模式")

class ApproachGuideTagState(ControllerBaseState['PrecisionLandController']):
    async def on_enter(self): 
        self.owner.logger.info("[PL 狀態] 進入：接近前置引导标记模式")
    async def on_update(self): 
        if not self.owner._tag_front_processor.is_valid():
            await self.owner.switch_state('search')
            return
        if self.owner._tag_down_processor.is_valid():
            await self.owner.switch_state('align_on_platform_tag')
            return
        vx,vy,vz,yaw_rate = self.owner._tag_front_processor.calculate_velocity_command(True)
        # print(f"[PL 狀態] 接近前置引导标记模式, vx: {vx:.2f}, vy: {vy:.2f}, vz: {vz:.2f}, yaw_rate: {yaw_rate:.2f}")

        await self.owner.mavsdk_controller.set_velocity_body(vx,vy,vz,yaw_rate)
    async def on_exit(self): 
        self.owner.logger.info("[PL 狀態] 離開：接近前置引导标记模式")

class AlignOnPlatformTagState(ControllerBaseState['PrecisionLandController']):
    async def on_enter(self): 
        self.owner.logger.info("[PL 狀態] 進入：对准平台标记模式")
    async def on_update(self): 
        if not self.owner._tag_down_processor.is_valid():
            await self.owner.switch_state('search')
            return

        if self.owner._tag_down_processor.is_aligned():
            await self.owner.switch_state('descend')
            return
        vx,vy,_,yaw_rate = self.owner._tag_down_processor.calculate_velocity_command(False)
        print(f"[PL 狀態] 对准平台标记模式, vx: {vx:.2f}, vy: {vy:.2f}, yaw_rate: {yaw_rate:.2f}")
        await self.owner.mavsdk_controller.set_velocity_body(vx,vy,0.0,yaw_rate)
    async def on_exit(self): 
        self.owner.logger.info("[PL 狀態] 離開：对准平台标记模式")

class DescendState(ControllerBaseState['PrecisionLandController']):
    async def on_enter(self): 
        self.owner.logger.info("[PL 狀態] 進入：下降模式")
    async def on_update(self): 
        if self.owner.drone_state.landed:
            await self.owner.switch_state('finished')
            return
        if not self.owner._tag_down_processor.is_valid():
            await self.owner.switch_state('search')
            return
        
        vx,vy,_,yaw_rate = self.owner._tag_down_processor.calculate_velocity_command(False)
        vz=0.5
        await self.owner.mavsdk_controller.set_velocity_body(vx,vy,vz,yaw_rate)
    async def on_exit(self): 
        self.owner.logger.info("[PL 狀態] 離開：下降模式")
    
class FinishedState(ControllerBaseState['PrecisionLandController']):
    async def on_enter(self): 
        self.owner.drone_state.search_started = False
        await self.owner.mavsdk_controller.stop_offboard()
        await self.owner.mavsdk_controller.disarm()
        self.owner.logger.info("[PL 狀態] 進入：完成模式")
    async def on_update(self): pass
    async def on_exit(self): pass

class AutoTuneState(ControllerBaseState['PrecisionLandController']):
    """
    执行高层位置控制器PID自动调参的状态。
    """
    async def on_enter(self):
        self.owner.logger.info("[PL 狀態] 進入：自动调参模式")
        self.axis_to_tune = self.owner.autotune_axis  # 'x' or 'y'
        config = self.owner._tag_down_config
        config['target_timeout']=1.5
        self.owner._tag_down_processor.update_config(config)
        self.owner.logger.info(f"====== 开始调参: {self.axis_to_tune.upper()} 轴 ======")
        self.owner.autotuner.start()
        self.start_time = time.time()
        self.tune_duration = 25  # 调参执行时间（秒）

    async def on_update(self):
        try:
            # 调参时必须能稳定看到下视Tag
            tag_processor = self.owner._tag_down_processor
            is_tag_valid = tag_processor.is_valid()
            # self.owner.logger.info(f"[AutoTuner Update] Tag valid: {is_tag_valid}", throttle_duration_sec=1.0)
            
            if not is_tag_valid:
                self.owner.logger.warning("[AutoTuner] 平台标记丢失，中断调参！")
                await self.owner.mavsdk_controller.set_velocity_body(0.0, 0.0, 0.0, 0.0)
                await self.owner.switch_state('idle')  # 修正：添加 await
                return

            # 获取当前误差
            position_error_body = tag_processor.tag.position

            if self.axis_to_tune == 'x':
                error = position_error_body[0]
                vx = self.owner.autotuner.step(error)
                vy, vz, yaw_rate = 0.0, 0.0, 0.0
            elif self.axis_to_tune == 'y':
                error = position_error_body[1]
                vy = self.owner.autotuner.step(error)
                vx, vz, yaw_rate = 0.0, 0.0, 0.0
            else:
                self.owner.logger.error(f"无效的调参轴: {self.axis_to_tune}")
                await self.owner.switch_state('idle') 
                return

            # 发送控制指令
            await self.owner.mavsdk_controller.set_velocity_body(vx, vy, vz, yaw_rate)

            # 检查是否达到调参时间
            if time.time() - self.start_time > self.tune_duration:
                self.owner.logger.info(f"调参时间已到 ({self.tune_duration}s)，开始分析数据...")
                self.owner.autotune_results = self.owner.autotuner.analyze()
                if self.owner.autotune_results:
                    self.owner.logger.info(f"[AutoTuner] 自动调参完成: {self.axis_to_tune.upper()} 轴")
                    self.owner.logger.info(f"[AutoTuner] 计算得到的PID参数: {self.owner.autotune_results}")
                else:
                    self.owner.logger.warning(f"[AutoTuner] 自动调参失败: {self.axis_to_tune.upper()} 轴")
                await self.owner.switch_state('idle') # 修正：添加 await

        except Exception as e:
            self.owner.logger.error(f"[AutoTuner Update ERROR] 在 on_update 中发生致命错误: {e}", exc_info=True)
            await self.owner.switch_state('idle')

    async def on_exit(self):
        self.owner.logger.info("[PL 狀態] 離開：自动调参模式")
        # 确保无人机悬停
        await self.owner.mavsdk_controller.set_velocity_body(0.0, 0.0, 0.0, 0.0)
        if self.owner.autotuner.is_tuning:
            self.owner.logger.info("[AutoTuner] 自动调参意外中止")

class IdleState(ControllerBaseState['PrecisionLandController']):
    async def on_enter(self): 
        self.owner.logger.info("[PL 狀態] 進入：待机模式")
        await self.owner.mavsdk_controller.set_velocity_body(0.0, 0.0, 0.0, 0.0)
    async def on_update(self): 
        await self.owner.mavsdk_controller.set_velocity_body(0.0, 0.0, 0.0, 0.0)
    async def on_exit(self): 
        self.owner.logger.info("[PL 狀態] 離開：待机模式")

