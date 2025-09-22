# drone_control/flight_controller/controller_base.py
import abc
from drone_control.fsm.state_machine import StateMachine

class ControllerBase(abc.ABC):
    """
    任務控制器的抽象基底類別 (對應 C++ 的 Character)。
    定義了一個擁有狀態機的獨立行為單元。
    """
    def __init__(self, node):
        self.node=node
        self.state_machine = StateMachine()
        self.drone_state = node.drone_state
        self.mavsdk_controller = node.mavsdk_controller
        self.logger = node.get_logger()

        # 衍生類別應該呼叫這個方法來註冊自己的狀態
        self._register_states()

    @abc.abstractmethod
    def _register_states(self):
        """
        衍生類別必須實現此方法，來建立並註冊所有專屬的狀態。
        """
        pass

    @abc.abstractmethod
    async def start(self):
        """
        啟動此控制器任務的進入點。
        """
        pass
    
    @abc.abstractmethod
    async def update(self):
        """
        主更新迴圈，將邏輯委託給狀態機。
        """
        await self.state_machine.update()

    async def switch_state(self, state_id: str):
        """
        切換到指定的狀態。
        """
        await self.state_machine.switch_state(state_id)
