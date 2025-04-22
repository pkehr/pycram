from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.actions import hsrb_transport_object, \
    drive_with_multiple_points
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.spawn import spawn_robot, setup_demo_objects
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.utils import get_robot_mode
from pycram.datastructures.enums import ROBOTS, ExecutionType
from pycram.datastructures.enums import WorldMode
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *

from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, \
    HSRBMoveGripperReal
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld


# TODO: Inspect real robot and simulation demo

def single_robot_demo(execution_type: ExecutionType, world_mode: WorldMode = WorldMode.DIRECT):
    world = BulletWorld(world_mode)
    robot_hsrb, hsrb_desig, hsrb_move = spawn_robot(ROBOTS.HSRB, name='hsrb', execution_type=execution_type)

    # Environment
    kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2024_1.urdf")
    kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

    gripper, talk, image_switch_publisher = None, None, None

    robot_mode = get_robot_mode(execution_type)

    if execution_type == ExecutionType.REAL:
        gripper = HSRBMoveGripperReal()
        talk = TextToSpeechPublisher()
        image_switch_publisher = ImageSwitchPublisher()

    # Setup demo objects
    (starting_pose_hsrb, _,
     table_one_nav_pose, table_two_nav_pose,
     hsrb_table_one_to_table_two, _,
     milk_object, milk_desig, milk_placing_pose,
     coffee_object, coffee_desig, coffee_placing_pose,
     chips_object, chips_desig, chips_placing_pose) = setup_demo_objects()

    navigate_start_hsrb = True
    navigate_table_one_hsrb = True

    transport_milk = False
    transport_coffee = False
    transport_chips = False

    print("starting_demo")

    '''
    Navigate
    Robot:      HSRB
    From:       Anywhere
    To:         starting_pose 
    '''
    if navigate_start_hsrb:
        with robot_mode(robot_hsrb):
            NavigateAction(target_locations=[starting_pose_hsrb]).resolve().perform()
        rospy.loginfo("HSRB is at starting position")

    '''
    Navigate
    Robot:      HSRB
    From:       starting_pose
    To:         Table#1 
    '''
    if navigate_table_one_hsrb:
        with robot_mode(robot_hsrb):
            NavigateAction(target_locations=[table_one_nav_pose]).resolve().perform()
        rospy.loginfo("HSRB is at the first table")

    '''
    Transport
    Object:     Object 1 (Milk)
    From:       Table#1
    To:         Table#2
    '''
    if transport_milk:
        with robot_mode(robot_hsrb):
            hsrb_transport_object(object_desig=milk_desig, nav_poses=hsrb_table_one_to_table_two,
                                  placing_pose=milk_placing_pose)
        rospy.loginfo("Object 1 transported on turtlebot")

    '''
    Navigate
    From:       Table#2
    From:       Table#1
    '''
    if transport_milk:
        with robot_mode(robot_hsrb):
            drive_with_multiple_points(reversed(hsrb_table_one_to_table_two))

    '''
    Transport
    Object:     Object 2 (Coffee)
    From:       Table#1
    To:         Table#2
    '''
    if transport_coffee:
        with robot_mode(robot_hsrb):
            hsrb_transport_object(object_desig=coffee_desig, nav_poses=hsrb_table_one_to_table_two,
                                  placing_pose=coffee_placing_pose)
        rospy.loginfo("Object 2 transported on turtlebot")

    '''
    Navigate
    From:       Table#2
    From:       Table#1
    '''
    if transport_coffee:
        with robot_mode(robot_hsrb):
            drive_with_multiple_points(reversed(hsrb_table_one_to_table_two))

    '''
    Pickup
    Object:     Object 3 (Chips)
    From:       Table#1
    Robot:      HSRB 
    '''
    if transport_chips:
        with robot_mode(robot_hsrb):
            hsrb_transport_object(object_desig=chips_desig, nav_poses=hsrb_table_one_to_table_two,
                                  placing_pose=chips_placing_pose)
            rospy.loginfo("Pickup Object 3")

    rospy.loginfo("Multi robot execution: Done")


if __name__ == "__main__":
    execution_type = ExecutionType.SIMULATED
    world_mode = WorldMode.GUI

    single_robot_demo(execution_type=execution_type, world_mode=world_mode)
