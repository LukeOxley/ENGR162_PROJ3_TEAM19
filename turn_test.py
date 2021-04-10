from odometry import Odometry
from transforms import *
import sensors
import time
import math

print("CCW is the positive direction!")
angle = float(input("Input the desired turn angle-> "))

dt = 0.02

odometry = Odometry(dt)
sensors.initSensors()

# left A Right B

turn_speed = 0.15
sensors.setLeftMotor(-turn_speed * angle / math.fabs(angle))
sensors.setRightMotor(turn_speed * angle / math.fabs(angle))

start_heading = odometry.getFieldToVehicle().getRotation()

print("Turning {:.2f} degrees".format(angle))

turning = True
while(turning):

    sensors.updateSensors()
    odometry.updateOdometry()

    if(math.fabs(start_heading.inverse().rotateBy(odometry.getFieldToVehicle().getRotation()).getDegrees()) >= math.fabs(angle)):
        print("Done Turning")
        turning = False

    time.sleep(dt)

sensors.shutdown()
