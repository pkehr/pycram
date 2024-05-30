import logging

from .boxy_description import BoxyDescription
from .donbot_description import DonbotDescription
from .hsr_description import HSRDescription
from .pr2_description import PR2Description
from .ur5_description import UR5Description
from .tiago_description import TiagoDescription
from .stretch_description import StretchDescription

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
