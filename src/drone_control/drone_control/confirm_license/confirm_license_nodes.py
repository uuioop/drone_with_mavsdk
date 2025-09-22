from drone_control.fsm import ControllerBaseState
from typing import TYPE_CHECKING

# 為了讓 VSCode 等 IDE 提供精確的型別提示
if TYPE_CHECKING:
    from .confirm_license_controller import ConfirmLicenseController

# class SearchState(ControllerBaseState['ConfirmLicenseController']):
#     def __init__(self):
#         pass