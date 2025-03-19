from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import ObjectType, ROBOTS
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