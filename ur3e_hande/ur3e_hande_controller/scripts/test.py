#!/usr/bin/env python

import rospy

import sys
import time
import copy
import moveit_commander

from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Pose
from moveit_msgs.msg import OrientationConstraint
from ur3e_hande_controller.msg import UR3eTarget, UR3eTrajectoryRTA
from ur3e_hande_controller.srv import UR3ePickAndPlaceRTA, UR3ePickAndPlaceRTARequest, UR3ePickAndPlaceRTAResponse

from settings_rta import *
from planner import Planner

class Test(Planner):
    def __init__(self) -> None:
        super(Test, self).__init__()

    def start(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur3e_moveit_server')
        self.run()
        moveit_commander.roscpp_shutdown()

    def add_trajectory(self, group, pose_type, pose, move_type="ptp"):
        trajectory_msg = UR3eTrajectoryRTA()

        if group == 'arm':
            trajectory = self.get_robot_trajectory(move_type, pose_type, pose)
            # self.ur3e_hande.update_robot(trajectory.joint_trajectory.points[-1].positions, pose)
            trajectory_msg.group = String('arm')

        if group == 'gripper':
            trajectory = self.get_gripper_trajectory(pose_type, pose)
            self.ur3e_hande.update_gripper(trajectory.joint_trajectory.points[-1].positions)
            trajectory_msg.group = String('gripper')

        trajectory_msg.trajectory = trajectory

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
            orientation = Quaternion(1, 0, 0, 0)
            pose.orientation = orientation

        if basket_color == 'blue':
            orientation = Quaternion(0, -1, 0, 0)
            pose.orientation = orientation

        self.add_trajectory('arm', 'name', 'arm_preplace_' + basket_color)
        self.plan_trajectory(pose, PLACE_OFFSET['z'], gripper_action)
        self.add_trajectory('arm', 'name', 'arm_preplace_' + basket_color)
        self.add_trajectory('arm', 'name', 'arm_safe')

    def plan_trajectory(self, pose, offset_z, gripper_action):
        offset_z_position = copy.deepcopy(pose)
        offset_z_position.position.z += offset_z

        self.add_trajectory('arm', 'pose', offset_z_position)
        self.add_trajectory('arm', 'pose', pose, 'lin')
        self.add_trajectory('gripper', 'name', gripper_action)
        self.add_trajectory('arm', 'pose', offset_z_position, 'lin')

    def run(self):
        self.reset()
        self.ur3e_hande.robot.state = self.move_group_arm.get_current_state()
        self.ur3e_hande.gripper.state = self.move_group_gripper.get_current_state()

        self.ur3e_hande.robot.pose = self.move_group_arm.get_current_pose("hande_left_finger").pose

        time.sleep(5)

        pose1 = Pose()
        pose1.position.x = 0.3
        pose1.position.y = 0
        pose1.position.z = 0.2
        pose1.orientation = Quaternion(1, 0, 0, 0)

        pose2 = Pose()
        pose2.position.x = 0.5
        pose2.position.y = 0.1
        pose2.position.z = 0.2
        pose2.orientation = Quaternion(1, 0, 0, 0)

        offset_z_position = copy.deepcopy(pose1)
        offset_z_position.position.z += .1

        self.add_trajectory('arm', 'pose', offset_z_position)
        self.add_trajectory('arm', 'pose', pose1, 'lin')
        self.add_trajectory('arm', 'pose', pose2, 'lin')
        self.add_trajectory('arm', 'pose', pose1, 'lin')
        self.add_trajectory('arm', 'pose', offset_z_position)

def start():
    pick_and_place = Test()
    pick_and_place.start()

if __name__ == "__main__":
    start()