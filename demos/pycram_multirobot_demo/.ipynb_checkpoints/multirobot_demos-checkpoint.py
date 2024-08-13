from IPython.core.display_functions import display
from ipywidgets import HTML

from demos.pycram_multirobot_demo.scenarios.move_and_park import move_and_park
from demos.pycram_multirobot_demo.scenarios.transporting_apartment import transporting_apartment
from demos.utils.enums import DEMOS, ROBOTS
from pycram.datastructures.enums import WorldMode

from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld


def multirobot_demo(robot_one: ROBOTS = ROBOTS.PR2, robot_two: ROBOTS = ROBOTS.TIAGO):
    demo = DEMOS.PR2_TIAGO_KITCHEN
    mode = WorldMode.GUI

    world = BulletWorld(mode)
    viz = VizMarkerPublisher() if mode == WorldMode.DIRECT else None

    if demo == DEMOS.PR2_TIAGO_SIMPLE:
        move_and_park(robot_one=robot_one, robot_two=robot_two)
    elif demo == DEMOS.PR2_TIAGO_KITCHEN:
        transporting_apartment(robot_one=robot_one, robot_two=robot_two)


def multirobot_demo_binder(robot_one, robot_two, environment):
    print(robot_one)
    print(robot_two)
    print(environment)
    display(HTML('<img src="https://i.gifer.com/XVo6.gif" alt="Hourglass animation" width="50">'))
    multirobot_demo()
