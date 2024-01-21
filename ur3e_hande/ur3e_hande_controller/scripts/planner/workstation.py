from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

class Workstation:
    def __init__(self):
        self.scene = PlanningSceneInterface()

        self.set_workstation()

    def set_workstation(self):
        self.set_table()
        self.set_baskets()

    def set_table(self):
        self.create_box('table_top', (0, 0, -0.025), (1.5, 1.5, 0.05))
        self.create_box('table_leg1', (0.7125, 0.7125, -0.55), (0.075, 0.075, 1))
        self.create_box('table_leg2', (0.7125, -0.7125, -0.55), (0.075, 0.075, 1))
        self.create_box('table_leg3', (-0.7125, 0.7125, -0.55), (0.075, 0.075, 1))
        self.create_box('table_leg4', (-0.7125, -0.7125, -0.55), (0.075, 0.075, 1))

    def set_baskets(self):
        self.set_basket('red', position=(-0.1, 0.35, 0.02))
        self.set_basket('blue', position=(-0.1, -0.35, 0.02))

    def set_basket(self, color, position):
        base_x, base_y, base_z = position

        offset1, offset2 = 0.02, 0.13
        wall_z = base_z + 0.075

        wall_size1 = (0.26, 0.04, 0.11)
        wall_size2 = (0.04, 0.26, 0.11)
        
        self.create_box(color + 'basket_base', (base_x, base_y, base_z), (0.3, 0.3, 0.04))
        self.create_box(color + 'basket_wall1', (base_x - offset1, base_y + offset2, wall_z), wall_size1)
        self.create_box(color + 'basket_wall2', (base_x + offset1, base_y - offset2, wall_z), wall_size1)
        self.create_box(color + 'basket_wall3', (base_x + offset2, base_y + offset1, wall_z), wall_size2)
        self.create_box(color + 'basket_wall4', (base_x - offset2, base_y - offset1, wall_z), wall_size2)
        
    def create_box(self, name, position, size):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.orientation.w = 1.0
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]

        self.scene.add_box(name, pose, size=size)