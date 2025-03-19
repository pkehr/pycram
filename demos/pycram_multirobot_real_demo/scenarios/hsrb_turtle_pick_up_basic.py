from demos.pycram_multirobot_real_demo.methods.actions import turtle_drive_to_table, hsrb_transport_object
from demos.pycram_multirobot_real_demo.methods.spawn import spawn_robot, setup_demo_objects
from pycram.datastructures.enums import ROBOTS
from pycram.datastructures.enums import WorldMode
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *

from pycram.object_descriptors.urdf import ObjectDescription
from pycram.process_module import real_robot
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, \
    HSRBMoveGripperReal
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

extension = ObjectDescription.get_file_extension()
world = BulletWorld(WorldMode.DIRECT)
gripper = HSRBMoveGripperReal()

# Spawn HSRB
robot_hsrb, hsrb_desig, hsrb_move = spawn_robot(ROBOTS.HSRB, name='hsrb')

# Spawn Turtle
robot_turtle, robot_desig_turtle, turtle_move = spawn_robot(ROBOTS.TURTLE, name='turtlebot')

kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2024_1.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

# important Publishers
talk = TextToSpeechPublisher()
image_switch_publisher = ImageSwitchPublisher()


def demo():
    # Setup demo objects
    table_one_nav_pose, table_two_nav_pose, milk_desig, milk_placing_pose, chips_desig, chips_placing_pose = setup_demo_objects()
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
    with real_robot(robot_turtle):
        turtle_drive_to_table()

    '''
    Transport
    Object:     Chips
    From:       Table#1
    To:         Table#2 
    '''
    with real_robot(robot_hsrb):
        hsrb_transport_object(object_desig=chips_desig, placing_nav_pose=table_two_nav_pose,
                              placing_pose=chips_placing_pose)

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
