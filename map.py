# keeps track of significant points
#   un-traveled path points
#   traveled path points
#   dead ends
#   hazards and their type
#   not perdiodically ran, just on a recording basis
wall_width = 40 # in cm


class Intersection:
    def __init__(self, point, turn_directions_turned):
        self.point = point
        self.turn_directions_turned = turn_directions_turned
    def __str__(self):
        return str(self.point) + " " + str(self.turn_directions_turned)
    def containtsDirection(self, turn_direction):
        in_list = False
        for dir in self.turn_directions_turned:
            if(turn_direction in dir):
                in_list = True
        return in_list
    def addDirection(self, turn_direction):
        if(not self.containtsDirection(turn_direction)):
            self.turn_directions_turned.append(turn_direction)

# relative to field position
class Turn_Direction_Field_t:
    FWD = 0
    LFT = 90
    RHT = -90
    BCK = 180

# relative to robot position
class Turn_Direction_Robot_t:
    FWD = 0
    LFT = 90
    RHT = -90
    BCK = 180
        

        