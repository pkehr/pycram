from threading import Lock
from typing_extensions import Any

import actionlib

from .. import world_reasoning as btr
import numpy as np
import rospy

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
from ..datastructures.enums import JointType, ObjectType, Arms
from ..external_interfaces import giskard
from ..external_interfaces.robokudo import query


def _park_arms(arm):
    """
    Defines the joint poses for the parking positions of the arms of ARMAR6 and applies them to the
    in the BulletWorld defined robot.
    :return: None
    """

    robot = RobotManager.active_robot
    if arm == "right":
        for joint, pose in RobotDescription.current_robot_description.get_static_joint_chain("right", "park").items():
            robot.set_joint_position(joint, pose)
    if arm == "left":
        for joint, pose in RobotDescription.current_robot_description.get_static_joint_chain("left", "park").items():
            robot.set_joint_position(joint, pose)


class ARMAR6Navigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion):
        robot = RobotManager.active_robot
        robot.set_pose(desig.target)


class ARMAR6MoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = RobotManager.active_robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("middle_neck"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("upper_neck"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)

        # For some reason the values for position.y and position.z are swapped, so for now the formula is adjusted accordingly.
        # Not guaranteed to work in all cases, depending on the reason why the values are swapped for this robot (maybe wrong urdf or something?)
        new_tilt = - np.arctan2(pose_in_tilt.position.y,
                                np.sqrt(pose_in_tilt.position.x ** 2 + pose_in_tilt.position.z ** 2))

        current_pan = robot.get_joint_position("neck_1_yaw")
        current_tilt = robot.get_joint_position("neck_2_pitch")

        robot.set_joint_position("neck_1_yaw", new_pan + current_pan)
        robot.set_joint_position("neck_2_pitch", new_tilt + current_tilt)


class ARMAR6MoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """

    def _execute(self, desig: MoveGripperMotion):
        robot = RobotManager.active_robot
        motion = desig.motion
        for joint, state in RobotDescription.current_robot_description.get_arm_chain(
                desig.gripper).get_static_gripper_state(motion).items():
            robot.set_joint_position(joint, state)


class ARMAR6Detecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig: DetectingMotion):
        robot = RobotManager.active_robot
        object_type = desig.object_type
        # Should be "wide_stereo_optical_frame"
        cam_frame_name = RobotDescription.current_robot_description.get_camera_frame()
        # should be [0, 0, 1]
        camera_description = RobotDescription.current_robot_description.cameras[
            list(RobotDescription.current_robot_description.cameras.keys())[0]]
        front_facing_axis = camera_description.front_facing_axis

        objects = World.current_world.get_object_by_type(object_type)
        for obj in objects:
            if btr.visible(obj, robot.get_link_pose(cam_frame_name), front_facing_axis):
                return obj


class ARMAR6MoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig: MoveTCPMotion):
        target = desig.target
        robot = RobotManager.active_robot

        _move_arm_tcp(target, robot, desig.arm)


class ARMAR6MoveArmJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig: MoveArmJointsMotion):

        robot = RobotManager.active_robot
        if desig.right_arm_poses:
            robot.set_joint_positions(desig.right_arm_poses)
        if desig.left_arm_poses:
            robot.set_joint_positions(desig.left_arm_poses)


class ARMAR6MoveJoints(ProcessModule):
    """
    Process Module for generic joint movements, is not confined to the arms but can move any joint of the robot
    """

    def _execute(self, desig: MoveJointsMotion):
        robot = RobotManager.active_robot
        robot.set_joint_positions(dict(zip(desig.names, desig.positions)))


class ARMAR6WorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.type == obj_type, World.current_world.objects))[0]


class ARMAR6Open(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, RobotManager.active_robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                          part_of_object.get_joint_limits(container_joint)[1])


class ARMAR6Close(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """

    def _execute(self, desig: ClosingMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, RobotManager.active_robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                              part_of_object.get_joint_limits(container_joint)[0])


def _move_arm_tcp(target: Pose, robot: Object, arm: Arms) -> None:
    gripper = RobotDescription.current_robot_description.get_arm_chain(arm).get_tool_frame()

    joints = RobotDescription.current_robot_description.get_arm_chain(arm).joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv)


###########################################################
########## Process Modules for the Real ARMAR6 ############
###########################################################


class ARMAR6NavigationReal(ProcessModule):
    """
    Process module for the real ARMAR6 that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        giskard.achieve_cartesian_goal(designator.target, RobotDescription.current_robot_description.base_link, "map")


class ARMAR6NavigationSemiReal(ProcessModule):
    """
    Process module for the real ARMAR6 that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        giskard.achieve_cartesian_goal(designator.target, RobotDescription.current_robot_description.base_link, "map")
        # queryPoseNav(designator.target)


class ARMAR6MoveHeadReal(ProcessModule):
    """
    Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
    as the simulated one
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = RobotManager.active_robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2) * -1

        current_pan = robot.get_joint_position("head_pan_joint")
        current_tilt = robot.get_joint_position("head_tilt_joint")

        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal({"head_pan_joint": new_pan + current_pan,
                                    "head_tilt_joint": new_tilt + current_tilt})


class ARMAR6DetectingReal(ProcessModule):
    """
    Process Module for the real ARMAR6 that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, designator: DetectingMotion) -> Any:
        query_result = query(ObjectDesignatorDescription(types=[designator.object_type]))
        # print(query_result)
        obj_pose = query_result["ClusterPoseBBAnnotator"]

        lt = LocalTransformer()
        obj_pose = lt.transform_pose(obj_pose, RobotManager.active_robot.get_link_tf_frame("torso_lift_link"))
        obj_pose.orientation = [0, 0, 0, 1]
        obj_pose.position.x += 0.05

        world_obj = World.current_world.get_object_by_type(designator.object_type)
        if world_obj:
            world_obj[0].set_pose(obj_pose)
            return world_obj[0]
        elif designator.object_type == ObjectType.JEROEN_CUP:
            cup = Object("cup", ObjectType.JEROEN_CUP, "jeroen_cup.stl", pose=obj_pose)
            return cup
        elif designator.object_type == ObjectType.BOWL:
            bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=obj_pose)
            return bowl

        return world_obj[0]


class ARMAR6MoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real ARMAR6 while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPMotion) -> Any:
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")

        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)
        giskard.achieve_cartesian_goal(pose_in_map, RobotDescription.current_robot_description.get_arm_chain(
            designator.arm).get_tool_frame(),
                                       "torso_lift_link")
        # robot_description.base_link)


class ARMAR6MoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real ARMAR6 to the given configuration while avoiding all collisions
    """

    def _execute(self, designator: MoveArmJointsMotion) -> Any:
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
        if designator.right_arm_poses:
            joint_goals.update(designator.right_arm_poses)
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(joint_goals)


class ARMAR6MoveJointsReal(ProcessModule):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """

    def _execute(self, designator: MoveJointsMotion) -> Any:
        name_to_position = dict(zip(designator.names, designator.positions))
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(name_to_position)


class ARMAR6MoveGripperReal(ProcessModule):
    """
     Opens or closes the gripper of the real ARMAR6 with the help of giskard.
     """

    def _execute(self, designator: MoveGripperMotion) -> Any:
        try:
            from tmc_control_msgs.msg import GripperApplyEffortActionGoal

            if (designator.motion == "open"):
                # TODO topic will need to be adjusted
                pub_gripper = rospy.Publisher('/ARMAR6/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                              queue_size=10)
                rate = rospy.Rate(10)
                rospy.sleep(2)
                msg = GripperApplyEffortActionGoal()  # sprechen joint gripper_controll_manager an, indem wir goal publishen type den giskard fürs greifen erwartet
                msg.goal.effort = 0.8
                pub_gripper.publish(msg)

            elif (designator.motion == "close"):
                # TODO topic will need to be adjusted
                pub_gripper = rospy.Publisher('/ARMAR6/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                              queue_size=10)
                rate = rospy.Rate(10)
                rospy.sleep(2)
                msg = GripperApplyEffortActionGoal()
                msg.goal.effort = -0.8
                pub_gripper.publish(msg)

        except ModuleNotFoundError as e:
            rospy.logwarn("Failed to import TMC messages, ARMAR can not be used")


class ARMAR6OpenReal(ProcessModule):
    """
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion) -> Any:
        giskard.achieve_open_container_goal(RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
                                            designator.object_part.name)


class ARMAR6CloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion) -> Any:
        giskard.achieve_close_container_goal(RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
                                             designator.object_part.name)


class ARMAR6Manager(ProcessModuleManager):

    def __init__(self):
        super().__init__("Armar6")
        self._navigate_lock = Lock()
        self._pick_up_lock = Lock()
        self._place_lock = Lock()
        self._looking_lock = Lock()
        self._detecting_lock = Lock()
        self._move_tcp_lock = Lock()
        self._move_arm_joints_lock = Lock()
        self._world_state_detecting_lock = Lock()
        self._move_joints_lock = Lock()
        self._move_gripper_lock = Lock()
        self._open_lock = Lock()
        self._close_lock = Lock()
        self._talk_lock = Lock()

    def navigate(self):
        if ProcessModuleManager.execution_type == "simulated":
            return ARMAR6Navigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "real":
            return ARMAR6NavigationReal(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return ARMAR6NavigationSemiReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return ARMAR6MoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == "real":
            return ARMAR6MoveHeadReal(self._looking_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return ARMAR6MoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return ARMAR6Detecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "real":
            return ARMAR6DetectingReal(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return ARMAR6Detecting(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return ARMAR6MoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "real":
            return ARMAR6MoveTCPReal(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return ARMAR6MoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return ARMAR6MoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return ARMAR6MoveArmJointsReal(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return ARMAR6MoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated" or ProcessModuleManager.execution_type == "real":
            return ARMAR6WorldStateDetecting(self._world_state_detecting_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return ARMAR6WorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return ARMAR6MoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return ARMAR6MoveJointsReal(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return ARMAR6MoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return ARMAR6MoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "real":
            return ARMAR6MoveGripperReal(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return ARMAR6MoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return ARMAR6Open(self._open_lock)
        elif ProcessModuleManager.execution_type == "real":
            return ARMAR6OpenReal(self._open_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return ARMAR6OpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return ARMAR6Close(self._close_lock)
        elif ProcessModuleManager.execution_type == "real":
            return ARMAR6CloseReal(self._close_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return ARMAR6CloseReal(self._close_lock)
