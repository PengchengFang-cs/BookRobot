from typing import List

from .base_controller import BaseController
from .arm_controller import ArmController
from .ee_controller import EEController
from .head_controller import HeadController
from .torso_controller import TorsoController
from .mobile_controller import MobileController

__all__ = [
    "BaseController",
    "ArmController",
    "EEController",
    "HeadController",
    "TorsoController",
    "MobileController",
    "ControllerManager",
]

class ControllerManager:
    def __init__(self, controllers: List[BaseController]):
        self.controllers = controllers

    def init_solver(self, arm_selection):
        for controller in self.controllers:
            controller.init_solver(arm_selection)

    def update(self):
        for controller in self.controllers:
            controller.update()
