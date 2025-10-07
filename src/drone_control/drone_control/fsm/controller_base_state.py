# drone_control/flight_controller/controller_state.py
from drone_control.fsm import StateNode
from typing import Generic, TypeVar

# 建立一個泛型型別變數，它可以代表任何一種控制器
TController = TypeVar('TController')
class ControllerBaseState(StateNode, Generic[TController]):
    """
    一個專為控制器設計的狀態泛型基底類別。
    它自動處理 owner 的接收和儲存，並提供精確的型別提示。
    """
    def __init__(self, owner: TController):
        """
        這個 __init__ 處理了所有重複的程式碼。
        """
        self.owner = owner
    
