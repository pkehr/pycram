from typing import List, Optional

import rospy

from .objects import ObjectOptions
from .utils import rotated_quaternion
from pycram.datastructures.enums import Grasp, Arms
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import *


def navigate_to_many_points(nav_poses: List[Pose]):
    for pose in nav_poses:
        NavigateAction(target_locations=[pose]).resolve().perform()
        #rospy.sleep(8)


def transport_object(object_option: ObjectOptions, object_dicts, execution_mode, robot, pickup_from_turtle=False, nav_poses=None,
                     place_on_turtle=False, grasp_type: Grasp = Grasp.FRONT):
    object_desig = object_dicts.desigs[object_option]
    object_placing_pose = object_dicts.placing_pose_on_turtle[object_option] if place_on_turtle else \
        object_dicts.placing_pose_on_table[object_option]

    with execution_mode(robot):
        hsrb_transport_object(object_desig=object_desig, nav_poses=nav_poses,
                              placing_pose=object_placing_pose, grasp_type=grasp_type, pickup_from_turtle=pickup_from_turtle)
    rospy.loginfo(f"{str(object_option)} transported to Table 2")


def hsrb_transport_object(object_desig,
                          nav_poses: Optional[List] = None,
                          placing_pose: Optional[Pose] = None,
                          grasp_type: Grasp = Grasp.FRONT,
                          pickup_from_turtle=False):
    ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()

    PickUpAction(object_designator_description=object_desig, arms=[Arms.LEFT],
                 grasps=[grasp_type], pickup_from_turtle=pickup_from_turtle).resolve().perform()

    ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()

    MoveTorsoAction(positions=[0.0]).resolve().perform()

    if nav_poses is not None:
        navigate_to_many_points(nav_poses)

    PlaceAction(object_desig, [placing_pose], [grasp_type], [Arms.LEFT], [False]).resolve().perform()

    ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()

    MoveTorsoAction(positions=[0.0]).resolve().perform()


def turtle_turn(angle: int = 0):
    # Create pose with rotaded Quaternion in turtle frame
    new_pose = Pose(position=[0, 0, 0], orientation=rotated_quaternion(angle))

    # TODO: Transform Pose into map frame

    NavigateAction(target_locations=[new_pose]).resolve().perform()


def turtle_turn_right():
    turtle_turn(angle=90)


def turtle_turn_left():
    turtle_turn(angle=-90)


def drive_with_multiple_points(poses: List[Pose]):
    for pose in poses:
        NavigateAction(target_locations=[pose]).resolve().perform()
