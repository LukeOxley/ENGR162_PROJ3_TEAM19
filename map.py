from tkinter.constants import W, X, Y


class Point:
    def __init__(self, x, y):
        self.x = x 
        self.y = y 
    
    def __str__(self):
        return "X: "+str(self.x)+" Y: "+str(self.y)

class Rotation:
    def __init__(self, deg):
        self.deg = deg
    def __str__(self):
        return str(self.deg)+" degrees"

class Pose:
    def __init__(self, point, rotation):
        self.point = point
        self.rotation = rotation
    def __str__(self):
        return str(self.point) + " at " + str(self.rotation)
    
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

class Turn_Direction_t:
    NORTH = "NORTH"
    SOUTH = "SOUTH"
    WEST = "WEST"
    EAST = "EAST"


