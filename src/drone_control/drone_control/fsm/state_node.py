# drone_control/fsm/state_node.py
import abc

class StateNode(abc.ABC):
    """
    通用的狀態抽象基類 (對應 C++ 的 StateNode)。
    
    所有具體的狀態都應該繼承自這個類別。這個基類本身不持有
    任何 context (擁有者)，context 由衍生類別自己在建構時接收並儲存。
    """
    @abc.abstractmethod
    async def on_enter(self):
        """進入此狀態時執行的非同步動作。"""
        pass

    @abc.abstractmethod
    async def on_update(self):
        """在此狀態下，每一幀或每一次更新時執行的非同步動作。"""
        pass

    @abc.abstractmethod
    async def on_exit(self):
        """離開此狀態時執行的非同步動作。"""
        pass