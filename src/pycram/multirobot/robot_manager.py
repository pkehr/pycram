import logging
from abc import ABC

from pycram.multirobot.object_observer import ObjectObserver
from pycram.robot_description import RobotDescriptionManager

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
    object_observer: ObjectObserver = None
    """
    Observer for currently used objects
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

        rdm = RobotDescriptionManager()
        rdm.load_description(name=robot_name)
        logging.info(f'Setting active robot. Is now: {robot_name}')

    @staticmethod
    def multiple_robots_active():
        if len(list(RobotManager.available_robots.keys())) > 1:
            RobotManager.object_observer = ObjectObserver()
            return True

        return False

    @staticmethod
    def block_object(object_desig):
        """
        Block a given object, if is not blocked already
        """
        if RobotManager.object_observer is not None:
            RobotManager.object_observer.block_object(object_desig)

    @staticmethod
    def release_object(object_desig):
        if RobotManager.object_observer is not None:
            RobotManager.object_observer.release_object(object_desig)
