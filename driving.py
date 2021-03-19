# determines possible navigation options
# interfaces with maze solver to find which directions to go
# interfaces with odometry to give it an action update
# stays between the walls

class Turn_Direction_t:
    NORTH = "NORTH"
    SOUTH = "SOUTH"
    EAST = "EAST"
    WEST = "WEST"

class Drive_State_t:
    IDLE = "IDLE"
    # TODO: possible entering mode?
    HALLWAY_FOLLOWING = "HALLWAY_FOLLOWING"
    INTERSECTION = "INTERSECTION"
