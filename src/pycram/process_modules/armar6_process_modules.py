from threading import Lock
from threading import Lock
from typing import Any

import numpy as np
import rospy

from .. import world_reasoning as btr
from ..designators.motion_designator import *
from ..datastructures.enums import JointType, ObjectType
from ..external_interfaces import giskard
from ..external_interfaces.ik import request_ik
# from ..external_interfaces.robokudo import queryEmpty, queryHuman, stop_queryHuman
from ..helper import _apply_ik
from ..local_transformer import LocalTransformer
# from ..external_interfaces.navigate import queryPoseNav
from ..process_module import ProcessModule
from ..robot_manager import get_robot_description
from ..worlds.bullet_world import BulletWorld


def _park_arms(arm):
    """
    Defines the joint poses for the parking positions of the arms of ARMAR6 and applies them to the
    in the BulletWorld defined robot.
    :return: None
    """

    robot = BulletWorld.robot
    if arm == "right":
        for joint, pose in get_robot_description()().get_static_joint_chain("right", "park").items():
            robot.set_joint_position(joint, pose)
    if arm == "left":
        for joint, pose in get_robot_description()().get_static_joint_chain("left", "park").items():
            robot.set_joint_position(joint, pose)


class ARMAR6Navigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion):
        robot = BulletWorld.robot
        robot.set_pose(desig.target)


class ARMAR6MoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = BulletWorld.robot

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
        robot = BulletWorld.robot
        gripper = desig.gripper
        motion = desig.motion
        for joint, state in get_robot_description().get_static_gripper_chain(gripper, motion).items():
            robot.set_joint_position(joint, state)


class ARMAR6Detecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig: DetectingMotion):
        robot = BulletWorld.robot
        object_type = desig.object_type
        # Should be "wide_stereo_optical_frame"
        cam_frame_name = get_robot_description().get_camera_frame()
        # should be [0, 0, 1]
        front_facing_axis = get_robot_description().front_facing_axis
        if desig.technique == 'all':
            rospy.loginfo("Fake detecting all generic objects")
            objects = BulletWorld.current_bullet_world.get_all_objets_not_robot()
        elif desig.technique == 'human':
            rospy.loginfo("Fake detecting human -> spawn 0,0,0")
            human = []
            human.append(Object("human", ObjectType.HUMAN, "human_male.stl", pose=Pose([0, 0, 0])))
            object_dict = {}

            # Iterate over the list of objects and store each one in the dictionary
            for i, obj in enumerate(human):
                object_dict[obj.name] = obj
            return object_dict

        else:
            rospy.loginfo("Fake -> Detecting specific object type")
            objects = BulletWorld.current_bullet_world.get_objects_by_type(object_type)

        object_dict = {}

        perceived_objects = []
        for obj in objects:
            if btr.visible(obj, robot.get_link_pose(cam_frame_name), front_facing_axis):
                perceived_objects.append(ObjectDesignatorDescription.Object(obj.name, obj.type, obj))
        # Iterate over the list of objects and store each one in the dictionary
        for i, obj in enumerate(perceived_objects):
            object_dict[obj.name] = obj

        rospy.loginfo("returning dict objects")
        return object_dict


class ARMAR6MoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig: MoveTCPMotion):
        target = desig.target
        robot = BulletWorld.robot

        _move_arm_tcp(target, robot, desig.arm)


class ARMAR6MoveArmJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig: MoveArmJointsMotion):

        robot = BulletWorld.robot
        if desig.right_arm_poses:
            robot.set_joint_positions(desig.right_arm_poses)
        if desig.left_arm_poses:
            robot.set_joint_positions(desig.left_arm_poses)


class ARMAR6MoveJoints(ProcessModule):
    """
    Process Module for generic joint movements, is not confined to the arms but can move any joint of the robot
    """

    def _execute(self, desig: MoveJointsMotion):
        robot = BulletWorld.robot
        robot.set_joint_positions(dict(zip(desig.names, desig.positions)))


class ARMAR6WorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.type == obj_type, BulletWorld.current_bullet_world.objects))[0]


class ARMAR6Open(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(container_joint)[1])


class ARMAR6Close(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """

    def _execute(self, desig: ClosingMotion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(container_joint)[0])


def _move_arm_tcp(target: Pose, robot: Object, arm: str) -> None:
    gripper = get_robot_description().get_tool_frame(arm)

    joints = get_robot_description().chains[arm].joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv, joints)


###########################################################
########## Process Modules for the Real ARMAR6 ############
###########################################################



class ARMAR6NavigationSemiReal(ProcessModule):
    """
    Process module for the real ARMAR6 that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        giskard.achieve_cartesian_goal(designator.target, get_robot_description().base_link, "map")
        # queryPoseNav(designator.target)

class ARMAR6MoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real ARMAR6 while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPMotion) -> Any:
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")
        giskard.avoid_all_collisions()
        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)
        giskard.achieve_cartesian_goal(pose_in_map, get_robot_description().get_tool_frame(designator.arm),
                                       "map")


class ARMAR6MoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real ARMAR6 to the given configuration while avoiding all collisions
    """

    def _execute(self, designator: MoveArmJointsMotion) -> Any:
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
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
        giskard.achieve_open_container_goal(get_robot_description().get_tool_frame(designator.arm),
                                            designator.object_part.name)


class ARMAR6CloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion) -> Any:
        giskard.achieve_close_container_goal(get_robot_description().get_tool_frame(designator.arm),
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
        elif ProcessModuleManager.execution_type == "semi_real":
            return ARMAR6NavigationSemiReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return ARMAR6MoveHead(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return ARMAR6Detecting(self._detecting_lock)
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
        
