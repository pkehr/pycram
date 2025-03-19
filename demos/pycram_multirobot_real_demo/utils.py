import math

from pycram.datastructures.enums import ExecutionType
from pycram.process_module import simulated_robot, semi_real_robot, real_robot


def convert_to_radians(angle):
    radians = (angle * math.pi) / 180
    return radians


def rotated_quaternion(angle):
    angle = convert_to_radians(angle)
    quaternion = [0, 0, math.sin(angle / 2), math.cos(angle / 2)]

    return quaternion


def get_robot_mode(execution_type):
    if ExecutionType.SIMULATED == execution_type:
        return simulated_robot
    elif ExecutionType.SEMI_REAL == execution_type:
        return semi_real_robot
    elif ExecutionType.REAL == execution_type:
        return real_robot
