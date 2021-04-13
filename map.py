from transforms import RigidTransform2d, Rotation2d, Translation2d
import math

# keeps track of significant points
#   un-traveled path points
#   traveled path points
#   dead ends
#   hazards and their type
#   not perdiodically ran, just on a recording basis

# 1 - path taken
# 5 - origin
# 2 - heat source
# 3 - magnetic source
# 4 - exit point

class Waypoint:
    def __init__(self, transform2d, number=1):
        self.transform2d = transform2d
        self.number = number
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
        
waypoints = []

def logIntersection(translation):
    waypoints.append(Waypoint(translation, 1))

def logStartPoint(translation):
    waypoints.append(Waypoint(translation, 5))

def logEndPoint(translation):
    waypoints.append(Waypoint(translation, 4))

def reset():
    global waypoints
    waypints = []

# need to fit the points to a grid, then fill in gaps with 1

grid_origin_to_robot = Translation2d(0,0)
wall_width_s = 40
# sets the translation from robot start to grid origin
def setTranslation(wall_width, behind_start_dist, start_x, start_y):
    global wall_width_s
    global grid_origin_to_robot
    wall_width_s = wall_width
    horizontal = wall_width * start_x
    verticle = wall_width * start_y - behind_start_dist - (wall_width / 2)
    grid_origin_to_robot = Translation2d(horizontal, verticle)

waypoints_grid = []
# transforms logged waypoints to grid coordinates
def transformToGridCoords():
    for point in waypoints:
        translation = point.transform2d.translateBy(grid_origin_to_robot)
        translation = Translation2d(round(translation.getX() / wall_width_s), 
                                    round(translation.getY() / wall_width_s))
        num = point.number
        waypoints_grid.append(Waypoint(translation, num)) 

grid = [[]]
# fills in the grid with the correct number based on the waypoints
def fillGrid():
    global grid
    # find maximum x and y value
    max_x = 0
    max_y = 0
    for point in waypoints_grid:
        if(point.transform2d.getX() > max_x):
            max_x = point.transform2d.getX()
        if(point.transform2d.getY() > max_y):
            max_y = point.transform2d.getY()
    # Convert to size of grid
    max_x += 1
    max_y += 1
    grid = []
    for i in range(max_y):
        row = [0] * max_x
        grid.append(row)
    
    for point in waypoints_grid:
        x = point.transform2d.getX()
        y = max_y - 1 - point.transform2d.getY()
        num = point.number
        grid[y][x] = num

    # Fill in gaps
    for i in range(len(waypoints_grid) - 1):
        curr = waypoints_grid[i]
        next = waypoints_grid[i+1]
        currx = curr.transform2d.getX()
        curry = curr.transform2d.getY()
        nextx = next.transform2d.getX()
        nexty = next.transform2d.getY()
        if(nextx - currx != 0):
            # change in x
            d = abs(nextx - currx) - 1
            dir = 1
            if(nextx - currx < 0):
                dir = -1
            # to get in between distance
            for j in range(1, d+1):
                grid[max_y - 1 - curry][currx + j*dir] = 1
        elif(nexty - curry != 0):
            # change in y
            d = abs(nexty - curry) - 1
            dir = 1
            if(nexty - curry < 0):
                dir = -1
            # to get in between distance
            for j in range(1, d+1):
                grid[max_y - 1 - (curry + j*dir)][currx] = 1

        
# calls all commands associated with making grid and returns grid
def makeGrid(wall_width, behind, startx, starty):
    global grid
    setTranslation(wall_width, behind, startx, starty)
    transformToGridCoords()
    fillGrid()
    return grid


    



