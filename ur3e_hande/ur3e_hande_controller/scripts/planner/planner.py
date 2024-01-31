from moveit_commander import MoveGroupCommander

from moveit_msgs.msg import RobotTrajectory
from planner.ur3e_hande import UR3e_Hande
from planner.workstation import Workstation

ARM_GROUP_NAME = "arm"
GRIPPER_GROUP_NAME = "gripper"

class Planner:
    def __init__(self):
        self.ur3e_hande = UR3e_Hande()
        self.workstation = Workstation()
        
        self.move_group_arm = MoveGroupCommander(ARM_GROUP_NAME)
        self.move_group_gripper = MoveGroupCommander(GRIPPER_GROUP_NAME)
    
    def plan_ptp(self, move_group: MoveGroupCommander, start_state, pose_type, pose):
        move_group.set_start_state(start_state)

        if pose_type == 'pose':
            move_group.set_pose_target(pose)
        if pose_type == 'name':
            move_group.set_named_target(pose)

        trajectory = move_group.plan()
        
        if not trajectory:
            return

        return trajectory[1]
    
    def plan_lin(self, move_group: MoveGroupCommander, robot, pose):
        move_group.set_start_state(robot.state)

        waypoints = [robot.pose, pose]
        trajectory = move_group.compute_cartesian_path(waypoints, 0.005, 0.0)
        
        if not trajectory:
            return
        
        return trajectory[0]
    
    def get_robot_trajectory(self, move_type: str, pose_type: str, pose) -> RobotTrajectory:
        if move_type.lower() == 'ptp':
            trajectory = self.plan_ptp(self.move_group_arm, self.ur3e_hande.robot.state, pose_type, pose)

        if move_type.lower() == 'lin':
            trajectory = self.plan_lin(self.move_group_arm, self.ur3e_hande.robot, pose)
        
        self.ur3e_hande.update_robot(trajectory.joint_trajectory.points[-1].positions, pose)

        return trajectory

    def get_gripper_trajectory(self, pose_type, pose) -> RobotTrajectory:
        trajectory = self.plan_ptp(self.move_group_gripper, self.ur3e_hande.gripper.state, pose_type, pose)
        self.ur3e_hande.update_gripper(trajectory.joint_trajectory.points[-1].positions)
        
        return trajectory

    def reset(self):
        self.ur3e_hande.reset()
        self.move_group_arm = MoveGroupCommander(ARM_GROUP_NAME)
        self.move_group_gripper = MoveGroupCommander(GRIPPER_GROUP_NAME)