from odometry import Odometry
from transforms import *
import sensors
import time
import math

#navigate to specified point while not 
# passing within specified radius of sources, 

print("X is horizontal, robot starts at 90 pose along +Y")

width = 40 #cm

print("Input coords are in grid coords")
start_x_g = float(input("Input the start x coord"))
start_y_g = float(input("Input the start y coord"))
goal_x_g = float(input("Input the desired x coord"))
goal_y_g = float(input("Input the desired y coord"))


# first implement move to point
# then implement hazard avoidance

dt = 0.02

odometry = Odometry(dt)
sensors.initSensors()

goal_location = Translation2d(goal_x_g * width, goal_y_g * width)

odometry.current_pos = RigidTransform2d(Translation2d(start_x_g*width, \
                        start_y_g*width), Rotation2d.fromDegrees(0))

# left A Right B
navigating = True
kP = 0.05
kMP = 0.003
resting_mag = 100
error_mag = 130
speed = 0.15
error_radius = 3 #cm
try:
    while(navigating):

        sensors.updateSensors()
        odometry.updateOdometry()

        current = odometry.getFieldToVehicle().getTranslation()
        goal_offset = current.inverse().translateBy(goal_location)
        heading_to_goal = Rotation2d(goal_offset.getX(), goal_offset.getY(), True)
        current_heading = odometry.getFieldToVehicle().getRotation()

        heading_error = current_heading.inverse().rotateBy(heading_to_goal).getDegrees()

        mag = sensors.getMagneticMagnitude()
        mag_error = 0
        if(mag > error_mag):
            mag_error = mag - resting_mag
            print("mag error")



        sensors.setLeftMotor(speed - heading_error*kP*speed - mag_error*kMP)
        sensors.setRightMotor(speed + heading_error*kP*speed + mag_error*kMP)

        #detect if within radius
        if(goal_offset.norm() <= error_radius):
            print("Navigating to point")
            sensors.setMotorOff()
            navigating = False

        time.sleep(dt)
except:
    sensors.shutdown()

sensors.shutdown()



