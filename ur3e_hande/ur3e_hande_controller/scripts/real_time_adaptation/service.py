#!/usr/bin/env python

import rospy

import sys
import copy
import moveit_commander

from ur3e_hande_controller.srv import UR3eMoveService, UR3eMoveServiceRequest, UR3eMoveServiceResponse

from ur3e_hande.ur3e_hande_controller.scripts.pick_and_place.settings import *
from ..planner.robot import Robot
from ..planner.planner import Planner
from ..planner.workstation import Workstation

class PickAndPlaceRTA_Service(Planner):
    def __init__(self, robot: Robot, sequence: dict):
        super().__init__(robot)

        self.sequence = sequence
        self.workstation = Workstation()

    def start_server(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur3e_moveit_server')

        rospy.Service('ur3e_moveit_rta', UR3eMoveService, self.run)
        rospy.spin()

    def plan_pick_and_place(self, position_type, position):
        offset_z_position = copy.deepcopy(position)
        offset_z_position.position.z += OFFSET_Z[position_type]

        if position_type == 'place':
            offset_xz_position = copy.deepcopy(offset_z_position)
            offset_xz_position.position.x += PLACE_OFFSET_X
            self.add_trajectory(offset_xz_position)
        
        self.add_trajectory(offset_z_position)
        self.add_trajectory(position)
        self.add_trajectory(offset_z_position)

        if position_type == 'place':
            self.add_trajectory(offset_xz_position)

    def get_position_type(self, position_name):
        if 'pick' in position_name.lower():
            return 'pick'
        
        if 'place' in position_name.lower():
            return 'place'
        
        raise Exception('Position names must be prefixed by "pick" or "place"')

    def run(self, request: UR3eMoveServiceRequest) -> UR3eMoveServiceResponse:
        self.reset()
        self.robot.update(request.joints_input.joints)
        
        for _, position_name in sorted(self.sequence.items()):
            position_type = self.get_position_type(position_name)
            self.plan_pick_and_place(position_type, request.__getattribute__(position_name))

        self.move_group.clear_pose_targets()
        
        return self.response