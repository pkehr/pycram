import math
from typing import List

from geometry_msgs.msg import PoseStamped

from pycram.datastructures.enums import ExecutionType
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot, semi_real_robot, real_robot
from pycram.utilities.robocup_utils import HSRBMoveGripperReal, TextToSpeechPublisher, ImageSwitchPublisher
from pycram.world_concepts.world_object import Object


def convert_to_radians(angle: int) -> float:
    radians = (angle * math.pi) / 180
    return radians


def rotated_quaternion(angle: int) -> List[float]:
    angle = convert_to_radians(angle)
    quaternion = [0, 0, math.sin(angle / 2), math.cos(angle / 2)]

    return quaternion


def get_robot_mode(execution_type: ExecutionType):
    if ExecutionType.SIMULATED == execution_type:
        return simulated_robot
    elif ExecutionType.SEMI_REAL == execution_type:
        return semi_real_robot
    elif ExecutionType.REAL == execution_type:
        return real_robot
    else:
        raise ValueError('Invalid execution type')

def set_real_publisher(execution_type: ExecutionType):
    gripper, talk, image_switch_publisher = None, None, None

    if execution_type == ExecutionType.REAL:
        gripper = HSRBMoveGripperReal()
        talk = TextToSpeechPublisher()
        image_switch_publisher = ImageSwitchPublisher()

    return gripper, talk, image_switch_publisher

def sync_object_from_giskard(obj: Object, gk_wrapper):
    obj_pose: PoseStamped = gk_wrapper.giskard_wrapper.world.get_group_info(obj.name).root_link_pose

    obj.set_pose(Pose.from_pose_stamped(obj_pose))

