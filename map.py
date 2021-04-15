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
    def __init__(self, transform2d, number=1, value=0):
        self.transform2d = transform2d
        self.number = number
        self.value = value
    def __str__(self):
        return str(self.transform2d) + " " + str(self.number)

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

grid_origin_to_robot = Translation2d(0,0)
wall_width_s = 40
# sets the translation from robot start to grid origin
def setTranslation(wall_width, behind_start_dist, start_x, start_y):
    global wall_width_s
    global startx
    global starty
    startx = start_x
    starty = start_y
    global grid_origin_to_robot
    wall_width_s = wall_width
    horizontal = wall_width * start_x
    verticle = wall_width * start_y - behind_start_dist - (wall_width / 2)
    grid_origin_to_robot = Translation2d(horizontal, verticle)       
    print(grid_origin_to_robot)

# transforms a coordinate to grid coordinates (aka to increments of 40)
def transformCoord(translation):
    new_translation = translation.translateBy(grid_origin_to_robot)
    new_translation = Translation2d(round(new_translation.getX() / wall_width_s), 
                                round(new_translation.getY() / wall_width_s))
    return new_translation

# reverts from grid coords to point
def revertCoord(translation):
    return Translation2d(translation.getX() * wall_width_s, \
                        translation.getY() * wall_width_s) \
                        .translateBy(grid_origin_to_robot.inverse())

waypoints = []

def isPointTaken(translation):
    for point in waypoints:
        # snaps to grid
        existing_loc = revertCoord(transformCoord(point.transform2d))
        if(math.fabs(translation.getX() - existing_loc.getX()) < wall_width_s/2.0 and
            math.fabs(translation.getY() - existing_loc.getY()) < wall_width_s/2.0):
            # the thing exists, pass 
            print("Map error: Point already exists")
            return True
    return False


def logIntersection(translation):
    if(not isPointTaken(translation)):
        waypoints.append(Waypoint(translation, 1))

def logStartPoint(translation):
    if(not isPointTaken(translation)):
        waypoints.append(Waypoint(translation, 5))

def logEndPoint(translation):
    if(not isPointTaken(translation)):
        waypoints.append(Waypoint(translation, 4))

def logHeatSource(translation, value):
    if(not isPointTaken(translation)): 
        waypoints.append(Waypoint(translation, 2, value))

def logMagneticSource(translation, value):
    if(not isPointTaken(translation)):
        waypoints.append(Waypoint(translation, 3, value))

def reset():
    global waypoints
    waypoints = []

# need to fit the points to a grid, then fill in gaps with 1

waypoints_grid = []
# transforms logged waypoints to grid coordinates
def transformToGridCoords():
    for point in waypoints:
        translation = transformCoord(point.transform2d)
        waypoints_grid.append(Waypoint(translation, point.number, point.value)) 
        print(waypoints_grid[-1])

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
def makeGrid():
    global grid
    transformToGridCoords()
    fillGrid()
    return grid

def exportGrid(map_number=0, notes=""):
    global grid
    global startx
    global starty
    makeGrid()
    filename = "team_19_map.csv"
    with open(filename, 'w') as file:
        file.write("Team: 19\n")
        file.write("Map: {:d}\n".format(map_number))
        file.write("Unit Length: {:d}\n".format(int(wall_width_s)))
        file.write("Unit: cm\n")
        file.write("\"Origin: ({:d}, {:d})\"\n".format(startx, starty))
        file.write("Notes: {:s}\n".format(notes))
        for row in grid:
            for entry in row:
                file.write(str(entry)+',')
            file.write('\n')
def exportHazards(map_number=0, notes=""):
    global waypoints
    filename = "team19_hazards.csv"
    with open(filename, 'w') as file:
        file.write("Team: 19\n")
        file.write("Map: {:d}\n".format(map_number))
        file.write("Notes: {:s}\n".format(notes))
        hazard_grid = []
        heading = ['Resource Type', 'Parameter of Interest', \
                   'Resource X Coordinate', 'Resource Y Coordinate']
        hazard_grid.append(heading)
        hazard_types = ["Heat Source", "Mag Source"]#TODO: Fix hazard naming
        param_of_intersts = ["Heat", "Mag (uT)"] #TODO: FIx params for hazards
        for point in waypoints:
            if(point.number == 2 or point.number == 3):
                hazard_grid.append([hazard_types[point.number - 2], \
                                    param_of_intersts[point.number - 2], 
                                    point.transform2d.getX(),
                                    point.transform2d.getY()])
        for row in hazard_grid:
            for entry in row:
                file.write(str(entry)+',')
            file.write('\n')


    

    



