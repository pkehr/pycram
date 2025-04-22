from typing import List, Optional

from utils import rotated_quaternion
from pycram.datastructures.enums import Grasp, Arms
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import ParkArmsAction, PickUpAction, NavigateAction, PlaceAction


def hsrb_transport_object(object_desig,
                          nav_poses: Optional[List] = None,
                          placing_pose: Optional[Pose] = None,
                          grasp_type=Grasp.RIGHT):
    # table_obj = DetectAction(technique='all').resolve().perform()

    ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()

    PickUpAction(object_designator_description=object_desig, arms=[Arms.LEFT],
                 grasps=[grasp_type]).resolve().perform()

    ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()

    if nav_poses is not None:
        for pose in nav_poses:
            NavigateAction(target_locations=[pose]).resolve().perform()

    PlaceAction(object_desig, [placing_pose], [grasp_type], [Arms.LEFT], [False]).resolve().perform()

    ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()


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
