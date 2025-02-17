import math


def convert_to_radians(angle):
    radians = (angle * math.pi) / 180
    return radians


def rotated_quaternion(angle):
    angle = convert_to_radians(angle)
    quaternion = [0, 0, math.sin(angle / 2), math.cos(angle / 2)]

    return quaternion