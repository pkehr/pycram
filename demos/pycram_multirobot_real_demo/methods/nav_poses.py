from enum import Enum, auto

from demos.pycram_multirobot_real_demo.utils import rotated_quaternion
from pycram.datastructures.enums import ROBOTS
from pycram.datastructures.pose import Pose


class NavOptions(Enum):
    STARTING = auto()
    TABLE_ONE = auto()
    TABLE_TWO = auto()
    KITCHEN = auto()

    FROM_ONE_TO_TWO_DIRECT = auto()
    FROM_ONE_TO_TWO_SUBPOINTS = auto()


class NavPoses:
    def __init__(self):
        self.hsrb_poses = {}
        self.turtle_poses = {}

        self.set_poses()

    def set_poses(self):
        self.set_starting_poses()
        self.set_object_poses()
        self.set_nav_poses()

    def set_starting_poses(self):
        starting_position_hsrb = [2.4, 3.0, 0.0]
        starting_orientation_hsrb = rotated_quaternion(angle=90)
        starting_pose_hsrb = Pose(position=starting_position_hsrb, orientation=starting_orientation_hsrb)

        starting_position_turtle = [1.54, 4.27, 0.0]
        starting_orientation_turtle = rotated_quaternion(angle=90)
        starting_pose_turtle = Pose(position=starting_position_turtle, orientation=starting_orientation_turtle)

        self.hsrb_poses[NavOptions.STARTING] = starting_pose_hsrb
        self.turtle_poses[NavOptions.STARTING] = starting_pose_turtle

    def set_object_poses(self):
        table_one_nav_position = [2.4, 4.2, 0.0]
        table_one_nav_orientation = rotated_quaternion(angle=90)
        table_one_nav_pose = Pose(position=table_one_nav_position, orientation=table_one_nav_orientation)

        table_two_nav_position = [4.0, 3.5, 0.0]
        table_two_nav_orientation = rotated_quaternion(angle=90)
        table_two_nav_pose = Pose(position=table_two_nav_position, orientation=table_two_nav_orientation)

        self.hsrb_poses[NavOptions.TABLE_ONE] = table_one_nav_pose
        self.hsrb_poses[NavOptions.TABLE_TWO] = table_two_nav_pose

    def set_nav_poses(self):
        self.hsrb_poses[NavOptions.FROM_ONE_TO_TWO_DIRECT] = [self.hsrb_poses[NavOptions.TABLE_ONE],
                                                              self.hsrb_poses[NavOptions.TABLE_TWO]]

        table_one_nav_position = [2.4, 4.2, 0.0]
        table_one_rotated_orientation = rotated_quaternion(angle=-90)
        table_one_rotated_pose = Pose(position=table_one_nav_position, orientation=table_one_rotated_orientation)

        second_nav_position = [2.4, 2.3, 0.0]
        second_nav_orientation = rotated_quaternion(angle=-90)
        second_nav_pose = Pose(position=second_nav_position, orientation=second_nav_orientation)

        third_nav_position = [3.5, 2.3, 0.0]
        third_nav_orientation = rotated_quaternion(angle=90)
        third_nav_pose = Pose(position=third_nav_position, orientation=third_nav_orientation)

        self.hsrb_poses[NavOptions.FROM_ONE_TO_TWO_SUBPOINTS] = [self.hsrb_poses[NavOptions.TABLE_ONE],
                                                                 table_one_rotated_pose, second_nav_pose,
                                                                 third_nav_pose,
                                                                 self.hsrb_poses[NavOptions.TABLE_TWO]]
