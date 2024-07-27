import rospy
from IPython.core.display_functions import display
from ipywidgets import HTML

from pycram import robot_description
from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import WorldMode, ObjectType
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import NavigateAction, LookAtAction, DetectAction, TransportAction, \
    PickUpAction, GraspingAction, PlaceAction
from pycram.designators.object_designator import BelieveObject

from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.robot_manager import get_robot_description
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
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


def multirobot_demo_apartment(robot_one: ROBOTS, robot_two: ROBOTS):
    # Robot poses
    pose_pr2 = Pose([1.5, 3, 0])
    pose_tiago = Pose([4, 3, 0])

    # Environment
    current_environment = set_environment(ENVIRONMENTS.APARTMENT)
    milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([0.5, 3, 1.02], orientation=[0, 0, 1, 0]),
                  color=Color(1, 0, 0, 1))
    milk_BO = BelieveObject(names=["milk"])
    cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                    pose=Pose([0.5, 3.3, 1.05]), color=Color(0, 1, 0, 1))
    spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([0.4, 3.2, 0.85]),
                   color=Color(0, 0, 1, 1))
    bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([0.5, 3.2, 1.02]),
                  color=Color(1, 1, 0, 1))

    # Spawn Robots
    robot_pr2 = create_robot(robot_one, pose=pose_pr2)
    robot_tiago = create_robot(robot_two, pose=pose_tiago)
    rospy.sleep(3)

    # Actions
    print("pr2 actions")
    with simulated_robot(robot_pr2):
        actions(park=True)

        NavigateAction(target_locations=[Pose([1.3, 3, 0], orientation=[0, 0, 1, 0])]).resolve().perform()

        PickUpAction(object_designator_description=milk_BO,
                     arms=["left"],
                     grasps=["front"]).resolve().perform()

        actions(park=True)

        NavigateAction(target_locations=[Pose([1.9, 3, 0], orientation=[0, 0, 0, 1])]).resolve().perform()

        PlaceAction(milk_BO, [Pose([2.75, 3, 1.02], orientation=[0, 0, 0, 1])], ["left"]).resolve().perform()

        actions(park=True)

    rospy.sleep(3)
    print("tiago actions")
    with simulated_robot(robot_tiago):
        actions(park=True)

        # NavigateAction(target_locations=[Pose([3.5, 3, 0], orientation=[0, 0, 1, 0])]).resolve().perform()
        NavigateAction(target_locations=[Pose([3.6, 3, 0], [0, 0, 1, 0])]).resolve().perform()

        milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.75, 3, 1.02], orientation=[0, 0, 1, 0]),
                      color=Color(1, 0, 0, 1))

        milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([3.3, 3.30, 0.62], orientation=[0, 0, 1, 0]),
                      color=Color(1, 0, 0, 1))

        robot_tiago.attach(milk)

        rospy.sleep(2)

        NavigateAction(target_locations=[Pose([4, 4, 0], [0, 0, 0, 1])]).resolve().perform()

        rospy.sleep(2)

        robot_tiago.detach(milk)

        milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([4.9, 3.8, 0.82], orientation=[0, 0, 1, 0]),
                      color=Color(1, 0, 0, 1))


def multirobot_demo(demo: DEMOS,
                    mode: WorldMode = WorldMode.DIRECT,
                    robot_one: ROBOTS = ROBOTS.PR2,
                    robot_two: ROBOTS = ROBOTS.TIAGO):
    world = BulletWorld(mode)
    viz = VizMarkerPublisher() if mode == WorldMode.DIRECT else None

    if demo == DEMOS.PR2_TIAGO_SIMPLE:
        multirobot_demo_simple(robot_one=robot_one, robot_two=robot_two)
    elif demo == DEMOS.PR2_TIAGO_KITCHEN:
        multirobot_demo_apartment(robot_one=robot_one, robot_two=robot_two)


if __name__ == "__main__":
    multirobot_demo(demo=DEMOS.PR2_TIAGO_KITCHEN,
                    mode=WorldMode.DIRECT,
                    robot_one=ROBOTS.PR2,
                    robot_two=ROBOTS.TIAGO)
