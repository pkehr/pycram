import rospy
from giskard_msgs.msg import LinkName

from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.actions import navigate_to_many_points, \
    transport_object
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.nav_poses import NavOptions
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.objects import ObjectOptions
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.scenario_selection import ScenarioSelection
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.spawn import spawn_robot, setup_demo_objects
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.utils import get_robot_mode, \
    set_real_publisher, rotated_quaternion, sync_object_from_giskard
from pycram.datastructures.enums import ROBOTS
from pycram.datastructures.enums import WorldMode
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *

from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld
import pycram.external_interfaces.giskard as gk


def hsrb_turtle_demo(execution_type: ExecutionType, world_mode: WorldMode = WorldMode.DIRECT):
    world = BulletWorld(world_mode)

    # Spawn HSRB
    robot_hsrb, hsrb_desig, hsrb_move = spawn_robot(ROBOTS.HSRB, name='hsrb', execution_type=execution_type)

    # Spawn Turtle
    robot_turtle, robot_desig_turtle, turtle_move = spawn_robot(ROBOTS.TURTLE, name='turtlebot')

    # Environment
    # kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2.urdf")
    # kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

    robot_mode = get_robot_mode(execution_type)

    RobotManager.set_giskard_robot(robot_hsrb.name)

    gripper, talk, image_switch_publisher = set_real_publisher(execution_type)

    # Setup demo objects
    nav_poses, objects = setup_demo_objects()

    table_one_nav_pose_hsrb, table_one_nav_pose_hsrb_rotated, table_two_nav_pose_hsrb, table_two_to_one_nav_pose = nav_poses.get_table_nav_poses(
        execution_type)
    table_two_hsrb_to_turtle = Pose(position=[4.55, 3.74, 0.0], orientation=rotated_quaternion(angle=180))
    table_two_hsrb_pose = [nav_poses.hsrb_poses[NavOptions.TABLE_TWO]]

    demo_scenario: ScenarioSelection = ScenarioSelection()
    demo_scenario.set_demo_scenario(use_turtle=True)

    turtle_tip_link = LinkName('base_footprint_turtle', 'turtle')
    gk.clear()
    gk.sync_worlds()

    print("starting_demo")
    with robot_mode(robot_hsrb):
        TalkingMotion("Starting multi-robot demo").perform()

    '''
    Navigate
    Robot:      HSRB
    From:       Anywhere
    To:         starting_pose 
    '''
    if demo_scenario.navigate_start_hsrb:
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
    if demo_scenario.navigate_start_turtle:
        starting_pose_turtle = nav_poses.turtle_poses[NavOptions.STARTING]
        starting_pose_turtle_rotated = Pose(position=starting_pose_turtle.position,
                                            orientation=rotated_quaternion((-90)))

        with robot_mode(robot_turtle):
            NavigateAction(target_locations=[starting_pose_turtle]).resolve().perform()
            NavigateAction(target_locations=[starting_pose_turtle_rotated]).resolve().perform()
        rospy.loginfo("Turtle is at starting position")

    '''
    Navigate
    Robot:      HSRB
    From:       starting_pose
    To:         Table#1 
    '''
    if demo_scenario.navigate_table_one_hsrb:
        with robot_mode(robot_hsrb):
            NavigateAction(target_locations=table_one_nav_pose_hsrb).resolve().perform()
        rospy.loginfo("HSRB is at the first table")

    '''
    Transport
    Object:     Object 1 (Milk)
    From:       Table#1
    To:         Turtlebot 
    '''
    if demo_scenario.transport_milk:
        transport_object(ObjectOptions.MILK,
                         object_dicts=objects,
                         robot=robot_hsrb,
                         nav_poses=table_one_nav_pose_hsrb_rotated,
                         execution_mode=robot_mode,
                         place_on_turtle=True)
        gk.achieve_attached(objects.desigs[ObjectOptions.MILK], tip_link=turtle_tip_link)

        with robot_mode(robot_hsrb):
            NavigateAction(target_locations=table_one_nav_pose_hsrb).resolve().perform()
        rospy.loginfo("Object 1 transported on turtlebot")

    '''
    Transport
    Object:     Object 2 (Coffee)
    From:       Table#1
    To:         Turtlebot 
    '''
    if demo_scenario.transport_coffee:
        transport_object(ObjectOptions.COFFEE,
                         object_dicts=objects,
                         robot=robot_hsrb,
                         nav_poses=table_one_nav_pose_hsrb_rotated,
                         execution_mode=robot_mode,
                         place_on_turtle=True)
        gk.achieve_attached(objects.desigs[ObjectOptions.COFFEE], tip_link=turtle_tip_link)

        with robot_mode(robot_hsrb):
            NavigateAction(target_locations=table_one_nav_pose_hsrb).resolve().perform()
        rospy.loginfo("Object 2 transported on turtlebot")

    '''
    Navigate
    Robot:      Turtlebot
    From:       Table#1
    To:         Table#2 
    '''
    if demo_scenario.navigate_table_two_turtle:
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
    if demo_scenario.transport_chips:
        transport_object(ObjectOptions.CHIPS,
                         object_dicts=objects,
                         nav_poses=table_two_nav_pose_hsrb,
                         robot=robot_hsrb,
                         execution_mode=robot_mode)
        rospy.loginfo("Object 3 placed on Table 2")

    '''
    Transport
    Object:     Object 1 (Milk)
    From:       Turtlebot
    To:         Table#2 
    '''
    if demo_scenario.transport_milk:
        milk_obj = objects.objects[ObjectOptions.MILK]

        sync_object_from_giskard(milk_obj, gk, adjusted_rotation=rotated_quaternion(180))

        with robot_mode(robot_hsrb):
            NavigateAction(target_locations=[table_two_hsrb_to_turtle]).resolve().perform()

            transport_object(ObjectOptions.MILK,
                             object_dicts=objects,
                             pickup_from_turtle=True,
                             nav_poses=table_two_hsrb_pose,
                             robot=robot_hsrb,
                             execution_mode=robot_mode)

        rospy.loginfo("Object 1 transported to Table 2")

    '''
    Transport
    Object:     Object 2 (Coffee)
    From:       Turtlebot
    To:         Table#2 
    '''
    if demo_scenario.transport_coffee:
        coffee_obj = objects.objects[ObjectOptions.COFFEE]

        sync_object_from_giskard(coffee_obj, gk, adjusted_rotation=rotated_quaternion(180))

        with robot_mode(robot_hsrb):
            NavigateAction(target_locations=[table_two_hsrb_to_turtle]).resolve().perform()

            transport_object(ObjectOptions.COFFEE,
                             object_dicts=objects,
                             pickup_from_turtle=True,
                             nav_poses=table_two_hsrb_pose,
                             robot=robot_hsrb,
                             execution_mode=robot_mode)
        rospy.loginfo("Object 2 transported on turtlebot")

    #with robot_mode(robot_hsrb):
    #    TalkingMotion("Done!").perform()

    rospy.loginfo("Multi robot execution: Done")


if __name__ == "__main__":
    execution_type = ExecutionType.REAL
    world_mode = WorldMode.DIRECT

    hsrb_turtle_demo(execution_type=execution_type, world_mode=world_mode)
