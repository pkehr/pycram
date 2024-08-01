from enum import Enum, auto

class ROBOTS(Enum):
    PANDA = 'panda'
    PR2 = 'pr2'
    HSRB = 'hsrb'
    TIAGO = 'tiago'
    BOXY = 'boxy'
    UR5 = 'ur5'
    TURTLE = 'turtle'
    DONBOT = 'donbot'
    STRETCH = 'stretch'

    ARMAR = 'armar'


class ENVIRONMENTS(Enum):
    APARTMENT = auto()
    APARTMENT_SMALL = auto()
    KITCHEN = auto()


class DEMOS(Enum):
    PR2_TIAGO_SIMPLE = auto()
    PR2_TIAGO_KITCHEN = auto()