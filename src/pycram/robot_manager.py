import logging
import re
from abc import ABC

import rospy
from pycram.robot_descriptions import DonbotDescription, PR2Description, BoxyDescription, \
    HSRDescription, UR5Description, TiagoDescription, StretchDescription

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class RobotManager(ABC):
    """
    Base class for managing multiple robots simultaneously
    """
    active_robot = None
    """
    Whether the robot for which the process module is intended for is real or a simulated one
    """
    available_robots = {}
    """
    List of all available robots
    """
    _instance = None
    """
    Singelton instance of this Robot Manager
    """

    robot_description = None
    """
    Robot description of active robot
    """

    def __new__(cls, *args, **kwargs):
        """
        Creates a new instance if :py:attr:`~RobotManager._instance` is None, otherwise the instance
        in :py:attr:`~RobotManager._instance` is returned.
        :return: Singelton instance of this Robot Manager
        """
        if not cls._instance:
            cls._instance = super(RobotManager, cls).__new__(cls)
            return cls._instance
        else:
            return cls._instance

    def __init__(self):
        """
        Init for RobotManager.
        Currently does nothing
        """
        pass

    @staticmethod
    def add_robot(robot_name, robot):
        """
        Add another robot to the list of available robots
        """
        RobotManager.available_robots[robot_name] = robot

    @staticmethod
    def set_active_robot(robot_name=None):
        """
        Returns the Process Module manager for the currently loaded robot or None if there is no Manager.

        :return: ProcessModuleManager instance of the current robot
        """

        instance = RobotManager()

        if robot_name is None:
            instance.robot_description = RobotManager.update_robot_description(from_ros=True)
            return

        instance.active_robot = RobotManager.available_robots[robot_name]
        instance.robot_description = RobotManager.update_robot_description(robot_name=robot_name)
        # instance.robot_description = RobotManager.update_robot_description(from_ros=True,
        #                                                                   topic=f'/{robot_name}/robot_description')

        logging.info(f'Setting active robot. Is now: {instance.active_robot.name}')

    @staticmethod
    def update_robot_description(robot_name=None, from_ros=None, topic='/robot_description'):
        # Get robot name
        if robot_name:
            robot = robot_name
        elif from_ros:
            try:
                urdf = rospy.get_param(topic)
            except Exception as e:
                logger.error("(robot-description) Could not get robot name from parameter server. Try again.")
                return None
            res = re.findall(r"robot\ *name\ *=\ *\"\ *[a-zA-Z_0-9]*\ *\"", urdf)
            if len(res) == 1:
                begin = res[0].find("\"")
                end = res[0][begin + 1:].find("\"")
                robot = res[0][begin + 1:begin + 1 + end].lower()
        else:
            return None

        # Choose Description based on robot name
        if 'iai_donbot' in robot:
            description = DonbotDescription
        elif 'pr2' in robot:
            description = PR2Description
        elif 'boxy' in robot:
            description = BoxyDescription
        elif 'hsr' in robot:
            description = HSRDescription
        elif "ur5_robotiq" in robot:
            description = UR5Description
        elif "tiago_dual" in robot:
            description = TiagoDescription
        elif "stretch" in robot:
            description = StretchDescription
        else:
            logger.error("(robot-description) The given robot name %s has no description class.", robot_name)
            return None
        return description()


def get_robot_description():
    return RobotManager().robot_description
