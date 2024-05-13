from enum import Enum, auto

import rospy

from pycram.bullet_world import BulletWorld, Object
from pycram.designators.action_designator import NavigateAction, ParkArmsAction, MoveTorsoAction
from pycram.enums import ObjectType, Arms
from pycram.pose import Pose
from pycram.process_module import simulated_robot


class ROBOTS(Enum):
    PANDA = auto()
    PR2 = auto()
    HSRB = auto()
    TIAGO = auto()
    BOXY = auto()
    UR5 = auto()


class ENVIRONMENTS(Enum):
    APARTMENT = auto()
    KITCHEN = auto()


def set_robot(robot: ROBOTS):
    # Works, stands still
    if robot == ROBOTS.PANDA:
        return Object("panda", ObjectType.ROBOT, "panda.urdf", pose=Pose([1, 2, 0]))

    # Not working, flying randomly around
    elif robot == ROBOTS.PR2:
        return Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([0, 1, 0]), ignoreCachedFiles=True)

    # Not able to launch yet
    elif robot == ROBOTS.HSRB:
        return Object("hsrb", ObjectType.ROBOT, "hsrb.urdf", pose=Pose([0, 1, 0]))

    # Somewhat works, arms are not being affected by gravity
    elif robot == ROBOTS.TIAGO:
        return Object("tiago", ObjectType.ROBOT, "tiago_dual.urdf", pose=Pose([0, 1, 0]))

    # Torso falling down, arms are slowly moving downwards
    elif robot == ROBOTS.BOXY:
        return Object("boxy", ObjectType.ROBOT, "boxy.urdf", pose=Pose([0, 1, 0]))

    # Not fully tested yet, does not explode like pr2
    elif robot == ROBOTS.UR5:
        return Object("ur5", ObjectType.ROBOT, "ur5_robotiq.urdf", pose=Pose([0, 1, 0]))

    else:
        raise Exception("No known Robot defined for world")


def set_environment(environment: ENVIRONMENTS):

    if environment == ENVIRONMENTS.APARTMENT:
        return Object("apartment", ObjectType.ENVIRONMENT, "apartment.urdf")
    elif environment == ENVIRONMENTS.KITCHEN:
        return Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")

    else:
        raise Exception("No known Environment defined for world")


def actions():
    with simulated_robot:
        print("Parking arms")
        rospy.sleep(2)
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        print("Moving Torso")
        rospy.sleep(2)
        MoveTorsoAction([0.33]).resolve().perform()
        print("Navigating")
        rospy.sleep(2)
        NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()
        print("done")


def simulating():
    print("simulating world")
    world.simulate(10, real_time=True)


world = BulletWorld()
current_robot = set_robot(ROBOTS.PR2)
#current_environment = set_environment(ENVIRONMENTS.KITCHEN)

#milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])
rospy.sleep(2)
print("now running simulation")
world.set_gravity([0, 0, -9.8])
world.set_realtime(True)

rospy.sleep(5)

simulating()

rospy.sleep(2)

actions()
