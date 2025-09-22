# drone_control/managers/node_manager.py

class LoggerManager:
    """
    一個單例，用於持有對主 ROS 節點的全局引用，並提供對其資源的存取。
    """
    def __init__(self):
        self._logger = None
        print("--- Logger Manager Initialized ---")

    def register_node(self, node):
        """
        由主節點在初始化時呼叫，進行註冊。
        """
        if not self._node:
            self._node = node
            self._logger = node.get_logger()
            self._logger.info("主節點已成功註冊到 NodeManager。")

    def get_logger(self):
        """
        獲取與主節點關聯的 logger。
        這是我們需要的核心便利函式！
        """
        if not self._logger:
            raise RuntimeError("LoggerManager: 主節點尚未註冊或已銷毀！")
        return self._logger

# 在模組層級建立唯一的全域實例
# 任何地方只要 `import node_manager`，得到的都是同一個 `manager` 物件
logger_manager = LoggerManager()
