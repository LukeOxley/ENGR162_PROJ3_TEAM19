from transforms import RigidTransform2d, Rotation2d, Translation2d

# keeps track of significant points
#   un-traveled path points
#   traveled path points
#   dead ends
#   hazards and their type
#   not perdiodically ran, just on a recording basis

class Intersection:
    def __init__(self, transform2d):
        self.transform2d = transform2d
    def __str__(self):
        return str(self.transform2d)

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
        
intersection_points = []

def logIntersection(intersection):
    global intersection_points
    intersection_points.append(intersection)
