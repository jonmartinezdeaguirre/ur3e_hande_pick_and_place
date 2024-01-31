#!/usr/bin/env python

import rospy

import sys
import copy
import moveit_commander

from ur3e_hande_controller.srv import UR3ePickAndPlace, UR3ePickAndPlaceRequest, UR3ePickAndPlaceResponse

from settings import *
from planner import Planner

class UnityPickAndPlace(Planner):
    def __init__(self, sequence: dict) -> None:
        super().__init__()
        self.sequence = sequence
        self.response = UR3ePickAndPlaceResponse()

    def start_server(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur3e_moveit_server')

        rospy.Service('ur3e_moveit', UR3ePickAndPlace, self.run)
        rospy.spin()

    def add_trajectory(self, pose_type, pose):
        trajectory = self.get_robot_trajectory('ptp', pose_type, pose)
        self.response.trajectories.append(trajectory)

    def plan_pick(self, position) -> None:
        offset_z_position = copy.deepcopy(position)
        offset_z_position.position.z += PICK_OFFSET['z']

        trajectories = [offset_z_position, position, offset_z_position]
        self.add_trajectories(trajectories)

    def plan_place(self, position) -> None:
        offset_z_position = copy.deepcopy(position)
        offset_z_position.position.z += PLACE_OFFSET['z']

        offset_xz_position = copy.deepcopy(offset_z_position)
        offset_xz_position.position.x += PLACE_OFFSET['x']

        trajectories = [offset_xz_position, offset_z_position, position, offset_z_position, offset_xz_position]
        self.add_trajectories(trajectories)

    def add_trajectories(self, trajectories: list):
        for trajectory in trajectories:
            self.add_trajectory('pose', trajectory)

    def get_position_type(self, position_name) -> str:
        if 'pick' in position_name.lower():
            return 'pick'
        
        if 'place' in position_name.lower():
            return 'place'
        
        raise Exception('Position sequence is incorrectly defined')

    def run(self, request: UR3ePickAndPlaceRequest) -> UR3ePickAndPlaceResponse:
        self.reset()
        self.ur3e_hande.update_robot(request.targets_input.targets)
        
        for _, position_name in sorted(self.sequence.items()):
            position_type = self.get_position_type(position_name)

            if position_type == 'pick':
                self.plan_pick(request.__getattribute__(position_name))

            if position_type == 'place':
                self.plan_place(request.__getattribute__(position_name))

        self.move_group_arm.clear_pose_targets()
        self.move_group_gripper.clear_pose_targets()
        
        return self.response

def start():
    pick_and_place = UnityPickAndPlace(TRAJECTORY_SEQUENCE)
    pick_and_place.start_server()

if __name__ == "__main__":
    start()