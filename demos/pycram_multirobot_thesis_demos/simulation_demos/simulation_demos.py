from typing import List

from IPython.core.display_functions import display
from ipywidgets import HTML

from scenarios.basic_actions.basic_actions import move_and_park
from scenarios.multithreaded_execution.multithreaded_execution import multithreaded_testing
from scenarios.party.party_apartment import party_apartment
from scenarios.transporting.transporting_apartment import transporting_apartment
from scenarios.transporting.transporting_kitchen import transporting_kitchen
from scenarios.load_n_robots.load_n_robots import triple_robots
from demos.pycram_multirobot_thesis_demos.utils.enums import SIMULATED_DEMOS
from pycram.datastructures.enums import WorldMode, ROBOTS
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld


def multirobot_demo(robots: List[ROBOTS], demo=SIMULATED_DEMOS.TRANSPORTING_APARTMENT, mode=WorldMode.GUI,
                    launch_robots=False):
    world = BulletWorld(mode)
    viz = VizMarkerPublisher() if mode == WorldMode.DIRECT else None

    if demo == SIMULATED_DEMOS.N_ROBOTS:
        move_and_park(robots=robots, launch_robots=launch_robots)
    elif demo == SIMULATED_DEMOS.TRANSPORTING_APARTMENT:
        transporting_apartment(robots=robots, launch_robots=launch_robots)
    elif demo == SIMULATED_DEMOS.TRANSPORTING_KITCHEN:
        transporting_kitchen(robots=robots, launch_robots=launch_robots)
    elif demo == SIMULATED_DEMOS.N_ROBOTS:
        triple_robots(robots=robots, launch_robots=launch_robots)
    elif demo == SIMULATED_DEMOS.THREADED_TEST:
        multithreaded_testing(robots=robots, launch_robots=launch_robots)
    elif demo == SIMULATED_DEMOS.PARTY:
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
    demo = SIMULATED_DEMOS.PARTY
    mode = WorldMode.DIRECT

    multirobot_demo_binder(robots, environment=demo, mode=mode, launch_robots=False)


if __name__ == '__main__':
    robots = [
        ROBOTS.PR2,
        ROBOTS.TIAGO,
        ROBOTS.JUSTIN,
        # ROBOTS.ICUB
    ]

    demo = SIMULATED_DEMOS.N_ROBOTS
    mode = WorldMode.GUI

    multirobot_demo(robots=robots, demo=demo, mode=mode)
