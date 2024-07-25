import rospy
from IPython.core.display_functions import display
from ipywidgets import HTML

from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld
from setup import ROBOTS, actions, create_robot, set_environment, ENVIRONMENTS, DEMOS


def multirobot_demo_simple(robot_one: ROBOTS, robot_two: ROBOTS):
    pose_pr2 = Pose([0, 1, 0])
    pose_tiago = Pose([0, 3, 0])

    robot_pr2 = create_robot(robot_one, pose=pose_pr2)
    robot_tiago = create_robot(robot_two, pose=pose_tiago)
    rospy.sleep(3)
    print("pr2 actions")
    with simulated_robot(robot_pr2):
        actions(park=True)

    rospy.sleep(3)
    print("tiago actions")
    with simulated_robot(robot_tiago):
        actions(park=True, torso=True)

    with simulated_robot(robot_pr2):
        actions(torso=True)


def multirobot_demo_binder():
    display(HTML('<img src="https://i.gifer.com/XVo6.gif" alt="Hourglass animation" width="50">'))
    multirobot_demo(demo=DEMOS.PR2_TIAGO_SIMPLE)


def multirobot_demo_kitchen(robot_one: ROBOTS, robot_two: ROBOTS):

    # Robot poses
    pose_pr2 = Pose([1.3, 3, 0])
    pose_tiago = Pose([4, 3, 0])

    # Environment
    current_environment = set_environment(ENVIRONMENTS.APARTMENT)

    # Spawn Robots
    robot_pr2 = create_robot(robot_one, pose=pose_pr2)
    robot_tiago = create_robot(robot_two, pose=pose_tiago)
    rospy.sleep(3)

    # Actions
    print("pr2 actions")
    with simulated_robot(robot_pr2):
        actions(park=True)

    rospy.sleep(3)
    print("tiago actions")
    with simulated_robot(robot_tiago):
        actions(park=True, torso=True)

    with simulated_robot(robot_pr2):
        actions(torso=True)


def multirobot_demo(demo: DEMOS,
                    mode: WorldMode = WorldMode.DIRECT,
                    robot_one: ROBOTS = ROBOTS.PR2,
                    robot_two: ROBOTS = ROBOTS.TIAGO):
    world = BulletWorld(mode)
    viz = VizMarkerPublisher() if mode == WorldMode.DIRECT else None

    if demo == DEMOS.PR2_TIAGO_SIMPLE:
        multirobot_demo_simple(robot_one=robot_one, robot_two=robot_two)
    elif demo == DEMOS.PR2_TIAGO_KITCHEN:
        multirobot_demo_kitchen(robot_one=robot_one, robot_two=robot_two)


if __name__ == "__main__":
    multirobot_demo(demo=DEMOS.PR2_TIAGO_KITCHEN,
                    mode=WorldMode.GUI,
                    robot_one=ROBOTS.TIAGO,
                    robot_two=ROBOTS.PR2)
