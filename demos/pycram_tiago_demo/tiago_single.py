from demos.utils.enums import ROBOTS
from demos.utils.launcher import launch_robot
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

world = BulletWorld(WorldMode.GUI)
viz = VizMarkerPublisher()
robot = Object("tiago_dual", ObjectType.ROBOT, "tiago_dual.urdf", pose=Pose([1, 2, 0]))
apartment = Object("apartment", ObjectType.ENVIRONMENT, "apartment.urdf")

milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]))
cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([2.5, 2.4, 1.05]))
spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([2.4, 2.2, 0.85]))
bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.5, 2.2, 1.02]))

#milk = Object("milk", "milk", "milk.stl", pose=Pose([4.8, 4.2, 0.8]), color=[1, 0, 0, 1])
#cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([4.8, 4, 0.8]), color=[0, 1, 0, 1])
#spoon = Object("spoon", "spoon", "spoon.stl", pose=Pose([4.8, 3.5, 0.8]), color=[0, 0, 1, 1])
#bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([4.8, 3.7, 0.8]), color=[1, 1, 0, 1])

apartment.attach(spoon, 'cabinet10_drawer_top')

pick_pose_milk = Pose([2.7, 2.15, 1])
pick_pose_cereal = Pose([2.7, 2.35, 1])


robot_desig = BelieveObject(names=["tiago_dual"])
apartment_desig = BelieveObject(names=["apartment"])

@with_simulated_robot
def move_and_detect(obj_type, pick_pose):
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

    return object_desig


with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.25]).resolve().perform()

    # milk
    milk_desig = move_and_detect(ObjectType.MILK, pick_pose_milk)

    NavigateAction([Pose([1.7, 1.5, 0], [0, 0, 0, 1])]).resolve().perform()

    TransportAction(milk_desig, [Arms.LEFT], [Pose([4.8, 4.2, 0.8])]).resolve().perform()

    # cereal
    cereal_desig = move_and_detect(ObjectType.BREAKFAST_CEREAL, pick_pose_cereal)

    NavigateAction([Pose([1.7, 1.9, 0], [0, 0, 0, 1])]).resolve().perform()

    TransportAction(cereal_desig, [Arms.LEFT], [Pose([4.8, 4, 0.8], [0, 0, 0, 1])]).resolve().perform()

    # bowl
    bowl_desig = move_and_detect("bowl", pick_pose_cereal)

    NavigateAction([Pose([1.7, 2.5, 0], [0, 0, 0, 1])]).resolve().perform()

    PickUpAction.Action(bowl_desig, "right", "front").perform()

    ParkArmsAction.Action(Arms.BOTH).perform()

    NavigateAction([Pose([4.1, 3.8, 0], [0, 0, 0, 1])]).resolve().perform()

    PlaceAction(bowl_desig, ["right"], ["front"], [Pose([4.8, 3.7, 0.8], [0, 0, 0, 1])]).resolve().perform()

    ParkArmsAction.Action(Arms.BOTH).perform()