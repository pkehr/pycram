from typing import List

import rospy

from demos.pycram_multirobot_demo.setup.actions import actions
from demos.utils.enums import ROBOTS
from demos.utils.launcher import launch_all_robots
from demos.utils.object_spawner import create_robot
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot


def create_default_spawn_poses(amount_of_robots: int) -> List[Pose]:
    poses = []
    x_offset = 0

    row_break = 4


    for i in range(amount_of_robots):
        if i == row_break:
            x_offset += 2

        y_position = ((i % row_break) * 2) - 5
        poses.append(Pose([x_offset, y_position, 0]))

    return poses


def n_robots(robots: List[ROBOTS], launch_robots=True):
    if launch_robots:
        launched_robots = launch_all_robots(robots=robots)

    robot_poses = create_default_spawn_poses(amount_of_robots=len(robots))

    robot_objects = []

    for r, p in zip(robots, robot_poses):
        robot = create_robot(r, p)
        robot_objects.append(robot)

    rospy.sleep(3)

    for current_robot in robot_objects:
        print(f"{current_robot.name} actions")

        with simulated_robot(current_robot):
            actions(torso=0.25)
