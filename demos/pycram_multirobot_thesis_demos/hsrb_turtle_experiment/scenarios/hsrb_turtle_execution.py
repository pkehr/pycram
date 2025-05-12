from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.actions import hsrb_transport_object, \
    navigate_to_many_points
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.nav_poses import NavOptions
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.objects import ObjectOptions
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.spawn import spawn_robot, setup_demo_objects
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.utils import get_robot_mode, set_real_publisher
from pycram.datastructures.enums import ROBOTS
from pycram.datastructures.enums import WorldMode
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *

from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld


# TODO: Inspect real robot and simulation demo

def hsrb_turtle_demo(execution_type: ExecutionType, world_mode: WorldMode = WorldMode.DIRECT):
    world = BulletWorld(world_mode)

    # Spawn HSRB
    robot_hsrb, hsrb_desig, hsrb_move = spawn_robot(ROBOTS.HSRB, name='hsrb', execution_type=execution_type)

    # Spawn Turtle
    robot_turtle, robot_desig_turtle, turtle_move = spawn_robot(ROBOTS.TURTLE, name='turtlebot')

    # Environment
    kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2.urdf")
    kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

    robot_mode = get_robot_mode(execution_type)

    RobotManager.set_giskard_robot(robot_hsrb.name)

    gripper, talk, image_switch_publisher = set_real_publisher(execution_type)

    # Setup demo objects
    nav_poses, objects = setup_demo_objects()

    navigate_start_turtle = False
    navigate_start_hsrb = True
    navigate_table_one_hsrb = True

    transport_milk = 0
    transport_coffee = 1
    transport_chips = 0

    navigate_table_two_turtle = False
    navigate_table_two_hsrb = 0

    # giskard.clear()
    giskard.sync_worlds()

    print("starting_demo")

    '''
    Navigate
    Robot:      HSRB
    From:       Anywhere
    To:         starting_pose 
    '''
    if navigate_start_hsrb:
        starting_pose_hsrb = nav_poses.hsrb_poses[NavOptions.STARTING]

        with robot_mode(robot_hsrb):
            NavigateAction(target_locations=[starting_pose_hsrb]).resolve().perform()
        rospy.loginfo("HSRB is at starting position")

    '''
    Navigate
    Robot:      Turtlebot
    From:       Anywhere
    To:         starting_pose (Table 1)
    '''
    if navigate_start_turtle:
        starting_pose_turtle = nav_poses.turtle_poses[NavOptions.STARTING]

        with robot_mode(robot_turtle):
            NavigateAction(target_locations=[starting_pose_turtle]).resolve().perform()
        rospy.loginfo("Turtle is at starting position")

    '''
    Navigate
    Robot:      HSRB
    From:       starting_pose
    To:         Table#1 
    '''
    if navigate_table_one_hsrb:
        with robot_mode(robot_hsrb):
            NavigateAction(target_locations=[nav_poses.hsrb_poses[NavOptions.TABLE_ONE]]).resolve().perform()
        rospy.loginfo("HSRB is at the first table")

    '''
    Transport
    Object:     Object 1 (Milk)
    From:       Table#1
    To:         Turtlebot 
    '''
    if transport_milk:
        table_one_nav_pose = nav_poses.hsrb_poses[NavOptions.TABLE_ONE]
        milk_desig = objects.desigs[ObjectOptions.MILK]
        milk_placing_pose = objects.placing_poses[ObjectOptions.MILK]

        with robot_mode(robot_hsrb):
            hsrb_transport_object(object_desig=milk_desig, placing_pose=milk_placing_pose, grasp_type=Grasp.RIGHT)
            NavigateAction(target_locations=[table_one_nav_pose]).resolve().perform()
        rospy.loginfo("Object 1 transported on turtlebot")

    '''
    Transport
    Object:     Object 2 (Coffee)
    From:       Table#1
    To:         Turtlebot 
    '''
    if transport_coffee:
        table_one_nav_pose = nav_poses.hsrb_poses[NavOptions.TABLE_ONE]
        coffee_desig = objects.desigs[ObjectOptions.COFFEE]
        coffee_placing_pose = objects.placing_poses[ObjectOptions.COFFEE]

        with robot_mode(robot_hsrb):
            hsrb_transport_object(object_desig=coffee_desig, placing_pose=coffee_placing_pose)
            NavigateAction(target_locations=[table_one_nav_pose]).resolve().perform()
        rospy.loginfo("Object 2 transported on turtlebot")

    '''
    Navigate
    Robot:      Turtlebot
    From:       Table#1
    To:         Table#2 
    '''
    if navigate_table_two_turtle:
        turtle_table_one_to_table_two = nav_poses.turtle_poses[NavOptions.FROM_ONE_TO_TWO_SUBPOINTS]

        with robot_mode(robot_turtle):
            # TODO: This has to be more points, as navigation tries to run against the wall otherwise
            navigate_to_many_points(nav_poses=turtle_table_one_to_table_two)
            rospy.loginfo("Turtlebot at Table 2")

    '''
    Pickup
    Object:     Object 3 (Chips)
    From:       Table#1
    Robot:      HSRB 
    '''
    if transport_chips:
        chips_desig = objects.desigs[ObjectOptions.CHIPS]

        with robot_mode(robot_hsrb):
            PickUpAction(object_designator_description=chips_desig, arms=[Arms.LEFT],
                         grasps=[Grasp.FRONT]).resolve().perform()
            ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()
            rospy.loginfo("Pickup Object 3")

    '''
    Navigate
    Robot:      HSRB
    From:       Table#1
    To:         Table#2 
    '''
    if navigate_table_two_hsrb:
        hsrb_table_one_to_table_two = nav_poses.hsrb_poses[NavOptions.FROM_ONE_TO_TWO_DIRECT]

        with robot_mode(robot_hsrb):
            navigate_to_many_points(nav_poses=hsrb_table_one_to_table_two)
            rospy.loginfo("Moved to Table 2")

    '''
    Place
    Object:     Object 3 (Chips)
    To:         Table#2
    Robot:      HSRB 
    '''
    if transport_chips:
        chips_desig = objects.desigs[ObjectOptions.CHIPS]
        chips_placing_pose = objects.placing_poses[ObjectOptions.CHIPS]

        with robot_mode(robot_hsrb):
            PlaceAction(chips_desig, target_locations=[chips_placing_pose], arms=[Arms.LEFT], grasps=[Grasp.FRONT],
                        with_force_torque=[False]).resolve().perform()
            ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()
            rospy.loginfo("Object 3 placed on Table 2")

    '''
    Transport
    Object:     Object 1 (Milk)
    From:       Turtlebot
    To:         Table#2 
    '''
    if transport_milk:
        milk_desig = objects.desigs[ObjectOptions.MILK]
        milk_placing_pose = objects.placing_poses[ObjectOptions.MILK]

        with robot_mode(robot_hsrb):
            hsrb_transport_object(object_desig=milk_desig, placing_pose=milk_placing_pose)
            ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()

        rospy.loginfo("Object 1 transported to Table 2")

    '''
    Transport
    Object:     Object 2 (Coffee)
    From:       Turtlebot
    To:         Table#2 
    '''
    if transport_coffee:
        coffee_desig = objects.desigs[ObjectOptions.COFFEE]
        coffee_placing_pose = objects.placing_poses[ObjectOptions.COFFEE]

        with robot_mode(robot_hsrb):
            hsrb_transport_object(object_desig=coffee_desig, placing_pose=coffee_placing_pose)
            ParkArmsAction(arms=[Arms.LEFT]).resolve().perform()
        rospy.loginfo("Object 2 transported on turtlebot")

    rospy.loginfo("Multi robot execution: Done")


if __name__ == "__main__":
    execution_type = ExecutionType.SEMI_REAL
    world_mode = WorldMode.DIRECT

    hsrb_turtle_demo(execution_type=execution_type, world_mode=world_mode)
