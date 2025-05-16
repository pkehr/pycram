from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.actions import navigate_to_many_points, \
    transport_object
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.nav_poses import NavOptions
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.objects import ObjectOptions
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.scenario_selection import ScenarioSelection
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.spawn import spawn_robot, setup_demo_objects
from demos.pycram_multirobot_thesis_demos.hsrb_turtle_experiment.methods.utils import get_robot_mode, set_real_publisher
from pycram.datastructures.enums import ROBOTS
from pycram.datastructures.enums import WorldMode
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *

from pycram.worlds.bullet_world import BulletWorld
import pycram.external_interfaces.giskard as gk


# TODO: Inspect real robot and simulation demo

def single_robot_demo(execution_type: ExecutionType, world_mode: WorldMode = WorldMode.DIRECT):
    world = BulletWorld(world_mode)
    robot_hsrb, hsrb_desig, hsrb_move = spawn_robot(ROBOTS.HSRB, name='hsrb', execution_type=execution_type)

    # Environment
    # kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2.urdf")
    # kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

    robot_mode = get_robot_mode(execution_type)

    gripper, talk, image_switch_publisher = set_real_publisher(execution_type)

    # tfb = TFBroadcaster()

    # Setup demo objects
    nav_poses, objects = setup_demo_objects()

    table_one_nav_pose, table_two_nav_pose, table_two_to_one_nav_pose = nav_poses.get_table_nav_poses(
        execution_type)

    demo_scenario: ScenarioSelection = ScenarioSelection()
    demo_scenario.set_demo_scenario(use_turtle=True)

    # TODO: Check if this breaks real world execution
    gk.clear()
    gk.sync_worlds()

    # tfb.update()

    print("starting_demo")
    #with robot_mode(robot_hsrb):
    #    TalkingMotion("Starting demo").perform()

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
    Robot:      HSRB
    From:       starting_pose
    To:         Table#1 
    '''
    if demo_scenario.navigate_table_one_hsrb:
        with robot_mode(robot_hsrb):
            NavigateAction(target_locations=table_one_nav_pose).resolve().perform()
        rospy.loginfo("HSRB is at the first table")

    '''
    Transport
    Object:     Object 1 (Milk)
    From:       Table#1
    To:         Table#2
    '''
    if demo_scenario.transport_milk:
        transport_object(ObjectOptions.MILK,
                         object_dicts=objects,
                         nav_poses=table_two_nav_pose,
                         robot=robot_hsrb,
                         execution_mode=robot_mode)

    '''
    Navigate
    From:       Table#2
    From:       Table#1
    '''
    if demo_scenario.transport_milk:
        with robot_mode(robot_hsrb):
            navigate_to_many_points(nav_poses=table_two_to_one_nav_pose)

    '''
    Transport
    Object:     Object 2 (Coffee)
    From:       Table#1
    To:         Table#2
    '''
    if demo_scenario.transport_coffee:
        transport_object(ObjectOptions.COFFEE,
                         object_dicts=objects,
                         nav_poses=table_two_nav_pose,
                         robot=robot_hsrb,
                         execution_mode=robot_mode)

    '''
    Navigate
    From:       Table#2
    From:       Table#1
    '''
    if demo_scenario.transport_coffee:
        with robot_mode(robot_hsrb):
            navigate_to_many_points(nav_poses=table_two_to_one_nav_pose)

    '''
    Pickup
    Object:     Object 3 (Chips)
    From:       Table#1
    Robot:      HSRB 
    '''
    if demo_scenario.transport_chips:
        transport_object(ObjectOptions.CHIPS,
                         object_dicts=objects,
                         nav_poses=table_two_nav_pose,
                         robot=robot_hsrb,
                         execution_mode=robot_mode)

    with robot_mode(robot_hsrb):
        TalkingMotion("Done!").perform()

    rospy.loginfo("Single robot execution: Done")


if __name__ == "__main__":
    execution_type = ExecutionType.SEMI_REAL
    world_mode = WorldMode.DIRECT

    single_robot_demo(execution_type=execution_type, world_mode=world_mode)
