# drone_control/fsm/state_machine.py
from .state_node import StateNode
from typing import Dict

class StateMachine:
    """
    這個版本完全模仿您的 C++ 實現。它本身不直接持有 context，
    只負責管理和驅動註冊的 State 物件。
    """
    def __init__(self):
        self._state_pool: Dict[str, StateNode] = {}
        self._current_state: StateNode = None

    async def switch_state(self, state_id: str):
        """根據 ID 安全地切換狀態。"""
        if self._current_state:
            await self._current_state.on_exit()
        
        self._current_state = self._state_pool.get(state_id)
        
        if self._current_state:
            await self._current_state.on_enter()

    def register_state(self, state_id: str, state_instance: StateNode):
        """註冊一個狀態實例到狀態池中。"""
        self._state_pool[state_id] = state_instance

    async def update(self):
        """驅動當前狀態執行其 update 邏輯。"""
        if not self._current_state:
            return
        await self._current_state.on_update()