from pycram.datastructures.enums import TorsoState
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
from pycram.process_module import simulated_robot



def navigate_simple_example():
    navigate_pose = Pose([4, 4, 0])

    with simulated_robot:
        MoveTorsoAction([TorsoState.HIGH]).resolve().perform()

        NavigateAction([navigate_pose]).resolve().perform()

