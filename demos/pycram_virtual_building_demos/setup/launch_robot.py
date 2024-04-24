import time
from enum import Enum, auto
from typing import List, Tuple

import roslaunch
import rospy

from roslaunch.parent import ROSLaunchParent

# For future work / robots
# def launch_hsrb():
#    # name = 'hsrb'
#    # urdf = 'hsrb.urdf'
#    executable = 'hsrb_standalone.launch'
#    launch_robot(executable)


# def launch_armar6():
#    # name = 'armar6'
#    # urdf = 'armar6.urdf'
#    executable = 'armar6_standalone.launch'
#    launch_robot(executable)

class Robots(Enum):
    PR2 = auto()


class RvizConfig(Enum):
    LOCAL = auto()
    WEB = auto()
    WITHOUT = auto()


class LaunchRobot:
    def __init__(self, robot: Robots, rviz: RvizConfig):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        launch_files = self.__get_launch_files(robot, rviz)

        self.parent: ROSLaunchParent = roslaunch.parent.ROSLaunchParent(self.uuid, launch_files)

    def start(self):
        self.parent.start()
        rospy.sleep(5)

    def shutdown(self):
        self.parent.shutdown()

    def __get_launch_files(self, robot, rviz) -> List:
        launch_files = []
        self.pycram_pkg = 'pycram'

        robot_config = self.__get_robot_config(robot)
        rviz_config = self.__get_rviz_config(rviz)

        if robot_config is not None:
            launch_files.append(robot_config)

        if rviz_config is not None:
            launch_files.append(rviz_config)

        return launch_files

    def __get_robot_config(self, robot):
        robot_launch_file = None

        if robot == Robots.PR2:
            cli_robot = [self.pycram_pkg, 'pr2_standalone.launch']
            robot_launch_file = roslaunch.rlutil.resolve_launch_arguments(cli_robot)[0]

        return robot_launch_file

    def __get_rviz_config(self, rviz):
        rviz_launch_file = None

        if rviz == RvizConfig.LOCAL:
            cli_rviz = [self.pycram_pkg, "local_rviz.launch"]
            rviz_launch_file = roslaunch.rlutil.resolve_launch_arguments(cli_rviz)[0]

        elif rviz == RvizConfig.WEB:
            cli_rviz = ['rvizweb', 'rvizweb.launch', 'config_file:=${PYCRAM_WS}/src/pycram/binder/rviz_configs/pr2_config.json']

            rviz_launch_file = (roslaunch.rlutil.resolve_launch_arguments(cli_rviz)[0], cli_rviz[2:])

        elif rviz == RvizConfig.WITHOUT:
            rospy.logdebug('launching without rviz')

        return rviz_launch_file
