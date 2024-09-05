import rospy
from gym.pybullet_robots.xarm.xarm import table_pos

from demos.pycram_multirobot_demo.setup.actions import actions
from demos.utils.enums import ROBOTS, ENVIRONMENTS
from demos.utils.launcher import launch_robot
from demos.utils.object_spawner import set_environment, create_robot
from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import ObjectType, Arms, Grasp
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import NavigateAction, TransportAction, LookAtAction, DetectAction
from pycram.designators.object_designator import BelieveObject
from pycram.process_module import real_robot, with_real_robot
from pycram.world_concepts.world_object import Object

@with_real_robot
def move_and_detect(obj_type, navigation_pose, pick_pose):
    NavigateAction(target_locations=[navigation_pose]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

    return object_desig

@with_real_robot
def hsrb_transport_object_on_turtle():
    actions(park=True)

    navigation_pose = Pose([0, 0, 0])
    object_pose = Pose([0.5, 3, 1.02])
    turtle_surface_pose = Pose([0, 0, 0])

    milk_desig = move_and_detect(ObjectType.MILK, navigation_pose, object_pose)
    TransportAction(milk_desig, [Arms.LEFT], [turtle_surface_pose]).resolve().perform()

@with_real_robot
def turtle_navigation():
    pass


def hsrb_transport_object():
    pass


def main():
    # Environment
    current_environment = set_environment(ENVIRONMENTS.SUTURO)
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
    pose_hsrb = Pose([1.5, 3, 0])
    pose_turtle = Pose([4, 3, 0])

    hsrb = create_robot(ROBOTS.HSRB, pose=pose_hsrb)
    turtlebot = create_robot(ROBOTS.TURTLE, pose=pose_turtle)
    rospy.sleep(3)

    # HSR transporting on turtle
    print(f"{hsrb.name} actions")
    with real_robot(hsrb):
        hsrb_transport_object_on_turtle()

    rospy.sleep(3)
    # Turtle moving towards goal table
    print(f"{turtlebot.name} actions")
    with real_robot(turtlebot):
        turtle_navigation()

    with real_robot(hsrb):
        hsrb_transport_object()


if __name__ == '__main__':
    main()