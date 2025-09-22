import time
import asyncio

class ConfirmPositionController:
    def __init__(self, mavsdk_controller, logger, drone_state,license_plate_processor):
        self.mavsdk_controller = mavsdk_controller
        self.logger = logger
        self.drone_state = drone_state
        self.license_plate_processor = license_plate_processor

    async def update(self):
        # process_result=self.license_plate_processor.process_license_plate(license_detection_result)
        if True:
            while not self.drone_state.tag_detected:
                await self.mavsdk_controller.set_velocity_body(0.0, 0.0, -0.4, 0.0)
                await asyncio.sleep(0.2)
            # 初始上升已完成，后续进入此状态时直接悬停
            await self.mavsdk_controller.set_velocity_body(0.0, 0.0, 0.0, 0.0)
            self._switch_to_precision_land()
            # self.logger.info(f"[号牌处理] 匹配成功！找到目标号牌: {license_detection_result}")
        else:
            await self.mavsdk_controller.set_velocity_body(0.0, 0.0, 0.0, 0.0)
            self.logger.info(f"[号牌处理] 匹配失败，继续搜索...")
    
    def _switch_to_precision_land(self):
        # 开始精准降落流程，进入搜索模式
        self.drone_state.search_started = True
        # 重置
        self.drone_state.update_confirm_start(False)
    