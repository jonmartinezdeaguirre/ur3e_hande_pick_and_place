from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState

from geometry_msgs.msg import Pose

class Robot:
    def __init__(self, joint_names) -> None:
        self.joint_names = joint_names

        self.joints = JointState()
        self.joints.name = self.joint_names

        self.state = RobotState()

        self.pose = Pose()

    def update(self, joint_positions, pose=None):
        self.joints.position = joint_positions
        self.state.joint_state = self.joints

        if pose:
            self.pose = pose

    def reset(self):
        self.joints = JointState()
        self.joints.name = self.joint_names

        self.state = RobotState()