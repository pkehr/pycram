from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import Arms
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
from pycram.external_interfaces.navigate import PoseNavigator
# new imports
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.process_module import real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, \
    HSRBMoveGripperReal, rotated_quaternion
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

extension = ObjectDescription.get_file_extension()
world = BulletWorld(WorldMode.DIRECT)
gripper = HSRBMoveGripperReal()

# Spawn HSRB
robot_hsrb = Object("hsrb", ObjectType.ROBOT, f"hsrb{extension}")
robot_desig_hsrb = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot_color = Color(R=0.6, G=0.6, B=0.6, A=1)
robot_hsrb.set_color(robot_color)
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states", multirobot_name='hsrb')
hsrb_move = PoseNavigator()

# Spawn Turtle
#robot_turtle = Object("turtlebot", ObjectType.ROBOT, f"turtlebot{extension}")
#robot_desig_turtle = ObjectDesignatorDescription(names=["turtlebot"]).resolve()
#robot_color = Color(R=0.6, G=0.6, B=0.6, A=1)
#robot_turtle.set_color(robot_color)
#turtle_move = PoseNavigator(namespace='turtle')

kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_5.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

# important Publishers
talk = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()


def hsrb_transport_object(object_desig, placing_pose, placing_nav_pose=None):
    # table_obj = DetectAction(technique='all').resolve().perform()

    ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()

    PickUpAction(object_designator_description=[object_desig], arms=[Arms.LEFT],
                 grasps=[Grasp.FRONT]).resolve().perform()

    ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()

    if placing_nav_pose is not None:
        NavigateAction(target_locations=[placing_nav_pose]).resolve().perform()

    PlaceAction(object_desig, [placing_pose], Grasp.FRONT, [Arms.LEFT]).resolve().perform()

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


def demo():
    table_one_nav_position = [2.59, 5.2, 0.0]
    table_one_nav_orientation = rotated_quaternion(angle=90)
    table_one_nav_pose = Pose(position=table_one_nav_position, orientation=table_one_nav_orientation)

    table_two_nav_position = [2.7, 2.7, 0.0]
    table_two_nav_orientation = rotated_quaternion(angle=-90)
    table_two_nav_pose = Pose(position=table_two_nav_position, orientation=table_two_nav_orientation)

    milk_position = [2.585, 5.85, 0.8]
    milk_pickup_orientation = rotated_quaternion(angle=90)
    milk_starting_pose = Pose(position=milk_position, orientation=milk_pickup_orientation)

    milk_place_orientation = rotated_quaternion(angle=180)
    milk_placing_pose = Pose(position=[1.87, 5.24, 0.45], orientation=milk_place_orientation)

    milk_object = Object("milk", ObjectType.MILK, "milk.stl", pose=milk_starting_pose)
    milk_desig = ObjectDesignatorDescription.Object(milk_object.name, ObjectType.MILK, milk_object)

    chips_position = [2.885, 5.85, 0.78]
    chips_pickup_orientation = rotated_quaternion(angle=90)
    chips_starting_pose = Pose(position=chips_position, orientation=chips_pickup_orientation)

    chips_place_orientation = rotated_quaternion(angle=-90)
    chips_placing_pose = Pose(position=[2.7, 2.7, 0.7], orientation=chips_place_orientation)

    chips_object = Object("chips", ObjectType.MILK, "milk.stl", pose=chips_starting_pose)
    chips_desig = ObjectDesignatorDescription.Object(milk_object.name, ObjectType.MILK, chips_object)
    print("starting_demo")

    '''
    Navigate
    Robot:      HSRB
    From:       starting_pose
    To:         Table#1 
    '''
    with real_robot(robot_hsrb):
        NavigateAction(target_locations=[table_one_nav_pose]).resolve().perform()

    '''
    Transport
    Object:     Milk
    From:       Table#1
    To:         Turtlebot 
    '''
    with real_robot(robot_hsrb):
        hsrb_transport_object(object_desig=milk_desig, placing_pose=milk_placing_pose)
        NavigateAction(target_locations=[table_one_nav_pose]).resolve().perform()

    '''
    Navigate
    Robot:      Turtlebot
    From:       Table#1
    To:         Table#2 
    '''
    # with real_robot(robot_turtle):
    #    turtle_drive_to_table()
    #    return

    '''
    Transport
    Object:     Chips
    From:       Table#1
    To:         Table#2 
    '''
    with real_robot(robot_hsrb):
        hsrb_transport_object(object_desig=chips_desig, placing_nav_pose=table_two_nav_pose, placing_pose=chips_placing_pose)
    return

    '''
    Transport
    Object:     Milk
    From:       Turtlebot
    To:         Table#2 
    '''
    with real_robot(robot_hsrb):
        hsrb_transport_object(object_desig=milk_desig, placing_pose=milk_placing_pose)
        ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()

    print("end")


if __name__ == "__main__":
    demo()
