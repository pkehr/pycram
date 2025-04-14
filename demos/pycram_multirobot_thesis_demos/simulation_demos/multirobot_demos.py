from typing import List

from IPython.core.display_functions import display
from ipywidgets import HTML

from demos.pycram_multirobot_thesis_demos.simulation_demos.scenarios.basic_actions.basic_actions import move_and_park
from demos.pycram_multirobot_thesis_demos.simulation_demos.scenarios.multithreaded_execution.multithreaded_execution import multithreaded_testing
from demos.pycram_multirobot_thesis_demos.simulation_demos.scenarios.party.party_apartment import party_apartment
from demos.pycram_multirobot_thesis_demos.simulation_demos.scenarios.transporting.transporting_apartment import transporting_apartment
from demos.pycram_multirobot_thesis_demos.simulation_demos.scenarios.transporting.transporting_kitchen import transporting_kitchen
from demos.pycram_multirobot_thesis_demos.simulation_demos.scenarios.load_n_robots.load_n_robots import triple_robots
from demos.utils.enums import DEMOS
from pycram.datastructures.enums import WorldMode, ROBOTS
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld


def multirobot_demo(robots: List[ROBOTS], demo=DEMOS.APARTMENT, mode=WorldMode.GUI, launch_robots=False):
    world = BulletWorld(mode)
    viz = VizMarkerPublisher() if mode == WorldMode.DIRECT else None

    if demo == DEMOS.N_ROBOTS:
        move_and_park(robots=robots, launch_robots=launch_robots)
    elif demo == DEMOS.TRANSPORTING_APARTMENT:
        transporting_apartment(robots=robots, launch_robots=launch_robots)
    elif demo == DEMOS.TRANSPORTING_KITCHEN:
        transporting_kitchen(robots=robots, launch_robots=launch_robots)
    elif demo == DEMOS.N_ROBOTS:
        triple_robots(robots=robots, launch_robots=launch_robots)
    elif demo == DEMOS.THREADED_TEST:
        multithreaded_testing(robots=robots, launch_robots=launch_robots)
    elif demo == DEMOS.PARTY:
        party_apartment(robots=robots, launch_robots=launch_robots)


def multirobot_demo_binder(robots, environment, mode=WorldMode.DIRECT, launch_robots=False):
    display(HTML('<img src="https://i.gifer.com/XVo6.gif" alt="Hourglass animation" width="50">'))
    multirobot_demo(robots=robots, demo=environment, mode=mode, launch_robots=launch_robots)


def start_party_demo():
    robots = [
        ROBOTS.PR2,
        ROBOTS.TIAGO,
        ROBOTS.JUSTIN,
        ROBOTS.ICUB
    ]
    demo = DEMOS.PARTY
    mode = WorldMode.DIRECT

    multirobot_demo_binder(robots, environment=demo, mode=mode, launch_robots=False)


if __name__ == '__main__':
    robots = [
        ROBOTS.PR2,
        ROBOTS.TIAGO,
        ROBOTS.JUSTIN,
        #ROBOTS.ICUB
    ]

    demo = DEMOS.N_ROBOTS
    mode = WorldMode.GUI

    multirobot_demo(robots=robots, demo=demo, mode=mode)
