from enum import Enum

class EdgeStatus(int, Enum):
    UNKNOWN = 80
    OBSTRUCTED = 100
    FREE = 0
    POTENTIALLY_OBSTRUCTED = 90
    POTENTIALLY_FREE = 10