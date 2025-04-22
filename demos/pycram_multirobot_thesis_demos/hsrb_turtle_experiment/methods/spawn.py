from nav_poses import NavPoses
from objects import ObjectManager
from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import ObjectType, ROBOTS, ExecutionType

from pycram.designator import ObjectDesignatorDescription
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object

extension = ObjectDescription.get_file_extension()


def spawn_robot(robot: ROBOTS, name: str, execution_type: ExecutionType = ExecutionType.REAL):
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
    objects = ObjectManager()

    return nav_poses, objects
