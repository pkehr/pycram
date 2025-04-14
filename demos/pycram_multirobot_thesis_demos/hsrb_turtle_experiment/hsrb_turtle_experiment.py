from scenarios.hsrb_turtle_execution import hsrb_turtle_demo
from scenarios.hsrb_turtle_threaded import hsrb_turtle_threaded_demo
from scenarios.single_robot_execution import single_robot_demo
from demos.pycram_multirobot_thesis_demos.utils.enums import REAL_DEMOS
from pycram.datastructures.enums import ExecutionType, WorldMode


def select_demo(demo: REAL_DEMOS, execution_type: ExecutionType, world_mode: WorldMode):
    if demo == REAL_DEMOS.HSRB_TURTLE:
        hsrb_turtle_demo(execution_type, world_mode)
    elif demo == REAL_DEMOS.HSRB_TURTLE_THREADED:
        hsrb_turtle_threaded_demo(execution_type, world_mode)
    elif demo == REAL_DEMOS.HSRB_SINGLE:
        single_robot_demo(execution_type, world_mode)


if __name__ == '__main__':
    demo = REAL_DEMOS.HSRB_TURTLE
    execution_type = ExecutionType.SEMI_REAL
    world_mode = WorldMode.DIRECT

    select_demo(demo, execution_type, world_mode)
