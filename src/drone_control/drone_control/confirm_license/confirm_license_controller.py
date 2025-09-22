from drone_control.fsm import ControllerBase
# from drone_control.utils import ArucoTag

class ConfirmLicenseController(ControllerBase):
    def __init__(self, node):
        super().__init__(node)
    def _register_states(self):
        return super()._register_states()
    async def start(self):
        """啟動確認号牌任務。"""
        await super().start()
    async def update(self):
        """確認号牌控制器的主更新迴圈。"""
        await super().update()