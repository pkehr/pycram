from .pr2_process_modules import Pr2Manager
from .boxy_process_modules import BoxyManager
from .donbot_process_modules import DonbotManager
from .hsr_process_modules import HSRBManager
from .default_process_modules import DefaultManager
from .stretch_process_modules import StretchManager
from ..robot_manager import RobotManager

Pr2Manager()
BoxyManager()
DonbotManager()
HSRBManager()
DefaultManager()
StretchManager()

RobotManager()
