import rospy

from demos.pycram_multirobot_demo.setup.actions import actions
from demos.pycram_multirobot_demo.setup.enums import ROBOTS
from demos.pycram_multirobot_demo.setup.object_spawner import create_robot
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot


def move_and_park(robot_one: ROBOTS, robot_two: ROBOTS):
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
