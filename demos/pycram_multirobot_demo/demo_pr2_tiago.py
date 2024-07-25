import rospy
from IPython.core.display_functions import display
from ipywidgets import HTML

from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld
from setup import ROBOTS, actions, create_robot


def multirobot_demo_simple(mode: WorldMode = WorldMode.DIRECT):
    world = BulletWorld(mode)
    viz = VizMarkerPublisher() if mode == WorldMode.DIRECT else None

    pose_pr2 = Pose([0, 1, 0])
    pose_tiago = Pose([0, 3, 0])

    print("Set first robot")
    robot_pr2 = create_robot(ROBOTS.PR2, pose=pose_pr2)
    # current_environment = set_environment(ENVIRONMENTS.KITCHEN)
    rospy.sleep(5)
    print("Set second robot")
    robot_tiago = create_robot(ROBOTS.TIAGO, pose=pose_tiago)
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
    multirobot_demo_simple(mode=WorldMode.DIRECT)


if __name__ == "__main__":
    multirobot_demo_simple(mode=WorldMode.DIRECT)
