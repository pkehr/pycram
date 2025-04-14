from enum import Enum, auto


class ENVIRONMENTS(Enum):
    APARTMENT = auto()
    APARTMENT_SMALL = auto()
    KITCHEN = auto()
    SUTURO = auto()


class DEMOS(Enum):
    BASIC_ACTIONS = auto()
    TRANSPORTING_APARTMENT = auto()
    TRANSPORTING_KITCHEN = auto()
    N_ROBOTS = auto()
    THREADED_TEST = auto()
    PARTY = auto()
