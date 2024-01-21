#!/usr/bin/env python

import rospy

import sys
import copy
import moveit_commander

from std_msgs.msg import String
from ur3e_hande_controller.msg import UR3eTarget, UR3eTrajectoryRTA
from ur3e_hande_controller.srv import UR3ePickAndPlaceRTA, UR3ePickAndPlaceRTARequest, UR3ePickAndPlaceRTAResponse

from settings_rta import *
from planner import Planner

class UnityPickAndPlaceRTA(Planner):
    def __init__(self) -> None:
        super(UnityPickAndPlaceRTA, self).__init__()

    def start_server(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur3e_moveit_server')

        rospy.Service('ur3e_moveit', UR3ePickAndPlaceRTA, self.run)
        rospy.spin()

    def add_trajectory(self, group, pose_type, pose, move_type="ptp"):
        trajectory_msg = UR3eTrajectoryRTA()

        if group == 'arm':
            trajectory = self.get_robot_trajectory(move_type, pose_type, pose)
            self.ur3e_hande.update_robot(trajectory.joint_trajectory.points[-1].positions, pose)
            trajectory_msg.group = String('arm')

        if group == 'gripper':
            trajectory = self.get_gripper_trajectory(pose_type, pose)
            self.ur3e_hande.update_gripper(trajectory.joint_trajectory.points[-1].positions)
            trajectory_msg.group = String('gripper')

        trajectory_msg.trajectory = trajectory
        self.response.trajectories.append(trajectory_msg)

    def check_limits(self, ball_position, basket_limits):
        x_in_limits = basket_limits['x'][0] < ball_position.x and ball_position.x < basket_limits['x'][1]
        y_in_limits = basket_limits['y'][0] < ball_position.y and ball_position.y < basket_limits['y'][1]
        z_in_limits = basket_limits['z'][0] < ball_position.z and ball_position.z < basket_limits['z'][1]

        return x_in_limits and y_in_limits and z_in_limits

    def check_ball_position(self, ball: UR3eTarget):
        basket_limits = BASKET_INNER_LIMITS[ball.color.data]
        ball_position = copy.deepcopy(ball.pose.position)
        ball_position.z += HANDE_OFFSET

        if ball.color.data == 'red':
            other_basket_limits = BASKET_INNER_LIMITS['blue']

        if ball.color.data == 'blue':
            other_basket_limits = BASKET_INNER_LIMITS['red']
        
        ball_in_correct_basket = self.check_limits(ball_position, basket_limits)
        ball_in_incorrect_basket = self.check_limits(ball_position, other_basket_limits)

        return ball_in_correct_basket, ball_in_incorrect_basket

    def plan_ball_trajectory(self, ball: UR3eTarget, ball_in_incorrect_basket) -> None:
        if ball_in_incorrect_basket and ball.color.data == 'red':
            return self.plan_basket_trajectory(ball.pose, 'blue', 'gripper_close')
        
        if ball_in_incorrect_basket and ball.color.data == 'blue':
            return self.plan_basket_trajectory(ball.pose, 'red', 'gripper_close')
        
        self.plan_trajectory(ball.pose, PICK_OFFSET['z'], 'gripper_close')
        self.add_trajectory('arm', 'name', 'arm_safe')

    def plan_basket_trajectory(self, pose, basket_color, gripper_action) -> None:
        if basket_color == 'red':
            pose.orientation.x = 1
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 0

        if basket_color == 'blue':
            pose.orientation.x = 0
            pose.orientation.y = -1
            pose.orientation.z = 0
            pose.orientation.w = 0

        self.add_trajectory('arm', 'name', 'arm_preplace_' + basket_color)
        self.plan_trajectory(pose, PLACE_OFFSET['z'], gripper_action)
        self.add_trajectory('arm', 'name', 'arm_preplace_' + basket_color)
        self.add_trajectory('arm', 'name', 'arm_safe')

    def plan_trajectory(self, pose, offset_z, gripper_action):
        offset_z_position = copy.deepcopy(pose)
        offset_z_position.position.z += offset_z

        self.add_trajectory('arm', 'pose', offset_z_position)
        self.add_trajectory('arm', 'pose', pose)
        self.add_trajectory('gripper', 'name', gripper_action)
        self.add_trajectory('arm', 'pose', offset_z_position)

    def run(self, req: UR3ePickAndPlaceRTARequest) -> UR3ePickAndPlaceRTAResponse:
        self.reset()
        self.response = UR3ePickAndPlaceRTAResponse()
        self.ur3e_hande.update(req.state.joints, req.state.fingers)
        
        ball_in_correct_basket, ball_in_incorrect_basket = self.check_ball_position(req.ball)
        
        if ball_in_correct_basket:
            self.response.trajectories = [UR3eTrajectoryRTA()]
            self.response.trajectories[0].group = String("none")
            return self.response

        self.plan_ball_trajectory(req.ball, ball_in_incorrect_basket)
        self.plan_basket_trajectory(req.basket.pose, req.basket.color.data, 'gripper_open')

        self.move_group_arm.clear_pose_targets()
        self.move_group_gripper.clear_pose_targets()
        
        return self.response

def start():
    pick_and_place = UnityPickAndPlaceRTA()
    pick_and_place.start_server()

if __name__ == "__main__":
    start()