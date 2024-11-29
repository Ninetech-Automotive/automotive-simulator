from enum import Enum

class EdgeStatus(Enum):
    UNKNOWN = 100
    OBSTRUCTED = 80
    FREE = 50
    POTENTIALLY_OBSTRUCTED = 90
    POTENTIALLY_FREE = 55