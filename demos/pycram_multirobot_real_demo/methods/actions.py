from demos.pycram_multirobot_real_demo.utils import rotated_quaternion
from pycram.datastructures.enums import Grasp, Arms
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import ParkArmsAction, PickUpAction, NavigateAction, PlaceAction


def hsrb_transport_object(object_desig, placing_pose, placing_nav_pose=None):
    # table_obj = DetectAction(technique='all').resolve().perform()

    ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()

    PickUpAction(object_designator_description=object_desig, arms=[Arms.LEFT],
                 grasps=[Grasp.FRONT]).resolve().perform()

    ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()

    if placing_nav_pose is not None:
        NavigateAction(target_locations=[placing_nav_pose]).resolve().perform()

    PlaceAction(object_desig, [placing_pose], [Grasp.FRONT], [Arms.LEFT], [False]).resolve().perform()

    ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()


def turtle_turn(angle=0):
    # Create pose with rotaded Quaternion in turtle frame
    new_pose = Pose(position=[0, 0, 0], orientation=rotated_quaternion(angle))

    # TODO: Transform Pose into map frame

    NavigateAction(target_locations=[new_pose]).resolve().perform()


def turtle_turn_right():
    turtle_turn(angle=90)


def turtle_turn_left():
    turtle_turn(angle=-90)


def turtle_drive_to_table():
    goal_position = [1.7, 3.7, 0.0]
    goal_orientation = rotated_quaternion(angle=-90)

    goal_pose = Pose(position=goal_position, orientation=goal_orientation)
    NavigateAction(target_locations=[goal_pose]).resolve().perform()