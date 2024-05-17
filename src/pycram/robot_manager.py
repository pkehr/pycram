from abc import ABC

from pycram.robot_descriptions import update_robot_description


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
    def set_active_robot(robot_name):
        """
        Returns the Process Module manager for the currently loaded robot or None if there is no Manager.

        :return: ProcessModuleManager instance of the current robot
        """
        RobotManager.active_robot = RobotManager.available_robots[robot_name]
        global active_robot_description
        active_robot_description = RobotManager.active_robot.description
        print(f'Setting active robot. Is now: {RobotManager.active_robot.name}')


active_robot_description = None

