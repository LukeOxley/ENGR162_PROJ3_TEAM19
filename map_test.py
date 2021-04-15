from transforms import Translation2d
import map

wall_width = 40
behind = 20

startx = 1
starty = 0

# map.logStartPoint(Translation2d(0,35.0))
# map.logIntersection(Translation2d(0,130))
# map.logIntersection(Translation2d(59,110))
# map.logEndPoint(Translation2d(40,170))
map.setTranslation(wall_width, behind, startx, starty)

map.logStartPoint(Translation2d(0 * wall_width, 0 * wall_width + behind + wall_width / 2))
map.logHeatSource(Translation2d(0 * wall_width, 3 * wall_width + behind + wall_width / 2), 10)
map.logMagneticSource(Translation2d(1 * wall_width, 3 * wall_width + behind + wall_width / 2), 20)
map.logIntersection(Translation2d(0 * wall_width, 2 * wall_width + behind + wall_width / 2))
map.logIntersection(Translation2d(2 * wall_width, 2 * wall_width + behind + wall_width / 2))
map.logIntersection(Translation2d(2 * wall_width, 4 * wall_width + behind + wall_width / 2))
map.logIntersection(Translation2d(-1 * wall_width, 4 * wall_width + behind + wall_width / 2))
map.logEndPoint(Translation2d(-1 * wall_width, 3 * wall_width + behind + wall_width / 2))

for point in map.waypoints:
    print(point)

map.exportGrid()
map.exportHazards()

