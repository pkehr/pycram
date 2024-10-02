from threading import Lock
from typing_extensions import Any

import actionlib

from .. import world_reasoning as btr
import numpy as np
import rospy

from ..external_interfaces.navigate import PoseNavigator
from ..multirobot import RobotManager
from ..process_module import ProcessModule, ProcessModuleManager
from ..external_interfaces.ik import request_ik
from ..utils import _apply_ik
from ..local_transformer import LocalTransformer
from ..designators.object_designator import ObjectDesignatorDescription
from ..designators.motion_designator import MoveMotion, LookingMotion, \
    DetectingMotion, MoveTCPMotion, MoveArmJointsMotion, WorldStateDetectingMotion, MoveJointsMotion, \
    MoveGripperMotion, OpeningMotion, ClosingMotion
from ..robot_description import RobotDescription
from ..datastructures.world import World
from ..world_concepts.world_object import Object
from ..datastructures.pose import Pose
from ..datastructures.enums import JointType, ObjectType, Arms, ExecutionType
from ..external_interfaces import giskard
from ..external_interfaces.robokudo import *



class TurtlebotNavigationReal(ProcessModule):
    """
    Process module for the real PR2 that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        move = PoseNavigator(namespace="turtle")
        # giskard.achieve_cartesian_goal(designator.target, robot_description.base_link, "map")
        move.pub_now(designator.target)


class TurtlebotNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion):
        robot = RobotManager.active_robot
        robot.set_pose(desig.target)


class TurtlebotManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("turtlebot")
        self._navigate_lock = Lock()
        self._looking_lock = Lock()
        self._detecting_lock = Lock()
        self._move_tcp_lock = Lock()
        self._move_arm_joints_lock = Lock()
        self._world_state_detecting_lock = Lock()
        self._move_joints_lock = Lock()
        self._move_gripper_lock = Lock()
        self._open_lock = Lock()
        self._close_lock = Lock()

    def navigate(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return TurtlebotNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return TurtlebotNavigationReal(self._navigate_lock)


