from demos.pycram_multirobot_real_demo.utils import rotated_quaternion
from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import ObjectType, ROBOTS
from pycram.datastructures.pose import Pose
from pycram.description import ObjectDescription
from pycram.designator import ObjectDesignatorDescription
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.world_concepts.world_object import Object

extension = ObjectDescription.get_file_extension()


def spawn_robot(robot: ROBOTS, name: str):
    robot_object = Object(name, ObjectType.ROBOT, f"{name}{extension}")
    robot_desig = ObjectDesignatorDescription(names=[name]).resolve()
    robot_color = Color(R=0.6, G=0.6, B=0.6, A=1)
    robot_object.set_color(robot_color)
    # RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states", multirobot_name='hsrb')
    robot_move = PoseNavigator(robot)

    return robot_object, robot_desig, robot_move


def setup_demo_objects():
    table_one_nav_position = [2.45, 0.969, 0.0]
    table_one_nav_orientation = rotated_quaternion(angle=90)
    table_one_nav_pose = Pose(position=table_one_nav_position, orientation=table_one_nav_orientation)

    table_two_nav_position = [2.7, 2.7, 0.0]
    table_two_nav_orientation = rotated_quaternion(angle=-90)
    table_two_nav_pose = Pose(position=table_two_nav_position, orientation=table_two_nav_orientation)

    milk_position = [2.585, 5.85, 0.8]
    milk_pickup_orientation = rotated_quaternion(angle=90)
    milk_starting_pose = Pose(position=milk_position, orientation=milk_pickup_orientation)

    milk_place_orientation = rotated_quaternion(angle=180)
    milk_placing_pose = Pose(position=[1.87, 5.24, 0.45], orientation=milk_place_orientation)

    milk_object = Object("milk", ObjectType.MILK, "milk.stl", pose=milk_starting_pose)
    milk_desig = ObjectDesignatorDescription.Object(milk_object.name, ObjectType.MILK, milk_object)

    chips_position = [2.885, 5.85, 0.78]
    chips_pickup_orientation = rotated_quaternion(angle=90)
    chips_starting_pose = Pose(position=chips_position, orientation=chips_pickup_orientation)

    chips_place_orientation = rotated_quaternion(angle=-90)
    chips_placing_pose = Pose(position=[2.7, 2.7, 0.7], orientation=chips_place_orientation)

    chips_object = Object("chips", ObjectType.MILK, "milk.stl", pose=chips_starting_pose)
    chips_desig = ObjectDesignatorDescription.Object(milk_object.name, ObjectType.MILK, chips_object)

    return table_one_nav_pose, table_two_nav_pose, milk_desig, milk_placing_pose, chips_desig, chips_placing_pose
