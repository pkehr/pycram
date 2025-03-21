from demos.pycram_multirobot_real_demo.methods.nav_poses import NavPoses
from demos.pycram_multirobot_real_demo.utils import rotated_quaternion
from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import ObjectType, ROBOTS, ExecutionType
from pycram.datastructures.pose import Pose

from pycram.designator import ObjectDesignatorDescription
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object

extension = ObjectDescription.get_file_extension()


def spawn_robot(robot: ROBOTS, name: str, execution_type=ExecutionType.REAL):
    robot_object = Object(name, ObjectType.ROBOT, f"{name}{extension}")
    robot_desig = ObjectDesignatorDescription(names=[name]).resolve()
    robot_color = Color(R=0.6, G=0.6, B=0.6, A=1)
    robot_object.set_color(robot_color)
    # RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states", multirobot_name='hsrb')

    robot_move = None

    if execution_type == ExecutionType.REAL:
        robot_move = PoseNavigator(robot)

    return robot_object, robot_desig, robot_move


def setup_demo_objects():
    # Nav poses
    nav_poses = NavPoses()

    # Objects
    milk_position = [2.585, 5.85, 0.8]
    milk_pickup_orientation = rotated_quaternion(angle=90)
    milk_starting_pose = Pose(position=milk_position, orientation=milk_pickup_orientation)

    milk_place_orientation = rotated_quaternion(angle=180)
    milk_placing_pose = Pose(position=[1.87, 5.24, 0.45], orientation=milk_place_orientation)

    milk_object = Object("milk", ObjectType.MILK, "milk.stl", pose=milk_starting_pose)
    milk_desig = ObjectDesignatorDescription.Object(milk_object.name, ObjectType.MILK, milk_object)

    coffee_position = [2.585, 5.85, 0.8]
    coffee_pickup_orientation = rotated_quaternion(angle=90)
    coffee_starting_pose = Pose(position=coffee_position, orientation=coffee_pickup_orientation)

    coffee_place_orientation = rotated_quaternion(angle=180)
    coffee_placing_pose = Pose(position=[1.87, 5.24, 0.45], orientation=coffee_place_orientation)

    coffee_object = Object("coffee", ObjectType.MILK, "milk.stl", pose=coffee_starting_pose)
    coffee_desig = ObjectDesignatorDescription.Object(milk_object.name, ObjectType.MILK, coffee_object)

    chips_position = [2.885, 5.85, 0.78]
    chips_pickup_orientation = rotated_quaternion(angle=90)
    chips_starting_pose = Pose(position=chips_position, orientation=chips_pickup_orientation)

    chips_place_orientation = rotated_quaternion(angle=-90)
    chips_placing_pose = Pose(position=[2.7, 2.7, 0.7], orientation=chips_place_orientation)

    chips_object = Object("chips", ObjectType.MILK, "milk.stl", pose=chips_starting_pose)
    chips_desig = ObjectDesignatorDescription.Object(milk_object.name, ObjectType.MILK, chips_object)

    return (nav_poses,
            milk_object, milk_desig, milk_placing_pose,
            coffee_object, coffee_desig, coffee_placing_pose,
            chips_object, chips_desig, chips_placing_pose)
