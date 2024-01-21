from real_time_adaptation.service import PickAndPlaceRTA_Service

from planner.robot import Robot
from ur3e_hande.ur3e_hande_controller.scripts.planner.planner import Planner
from ur3e_hande.ur3e_hande_controller.scripts.planner.workstation import Workstation

class UnityPickAndPlaceRTA(Planner):
    def __init__(self, robot: Robot, sequence: dict):
        super().__init__(robot)

        self.sequence = sequence
        self.workstation = Workstation()

    def start_server(self):
        pass