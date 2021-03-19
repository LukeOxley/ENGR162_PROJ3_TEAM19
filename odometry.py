import map
import robot
import sensors
import math

# updates robot position on the coordinate plane
# periodically ran
# coordinate axes: forward positive y, right positive x

current_pos = map.Pose(map.Point(0,0), map.Rotation(0))
last_left_encoder_reading = 0
last_right_encoder_reading = 0
dt = robot.Robot.cycle_time

def initOdometry():
    current_pos = map.Pose(map.Point(0,0), map.Rotation(0))
    last_left_encoder_reading = 0
    last_right_encoder_reading = 0

def updateOdometry():
    # integrate average of wheel velocities * cos (theta) or sin(theta) for x and y
    # use gyro to find change in angle, or you can use the difference in wheel speeds
    # see: http://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf pp. 3

    delta_left = sensors.getLeftWheelDistance() - last_left_encoder_reading
    delta_right = sensors.getRightWheelDistance() - last_right_encoder_reading
    delta_theta = current_pos.rotation.deg - sensors.getHeading()

    # TODO: check if theta turns in the correct direction ...
    delta_x = 0.5*(delta_left + delta_right) * math.sin(delta_theta * 3.14159 / 180)
    delta_y = 0.5*(delta_left + delta_right) * math.cos(delta_theta * 3.14159 / 180)

    current_pos.point.x += delta_x
    current_pos.point.y += delta_y
    current_pos.rotation.deg += delta_theta

def getCurrentPos():
    return current_pos