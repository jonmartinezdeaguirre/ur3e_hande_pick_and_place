from planner.robot import Robot

UR3E_HANDE_JOINTS = {
    'ur3e': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
    'hande': ['hande_left_finger_joint']
}

class UR3e_Hande:
    def __init__(self) -> None:
        self.robot = Robot(UR3E_HANDE_JOINTS['ur3e'])
        self.gripper = Robot(UR3E_HANDE_JOINTS['hande'])

    def update_robot(self, joint_positions, pose=None):
        self.robot.update(joint_positions, pose)
    
    def update_gripper(self, finger_positions):
        self.gripper.update(finger_positions)

    def update(self, joint_positions, finger_positions):
        self.robot.update(joint_positions)
        self.gripper.update(finger_positions)

    def reset(self):
        self.robot.reset()
        self.gripper.reset()