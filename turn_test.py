from odometry import Odometry
from transforms import *
import sensors
import time
import math

print("CCW is the positive direction!")
angle = float(input("Input the desired turn angle-> "))

dt = 0.01

odometry = Odometry(dt)
sensors.initSensors()

heading_setpoint = Rotation2d.fromDegrees(0)
# left A Right B
def setHeading(offset):
    global heading_setpoint
    sensors.updateSensors()
    deg = odometry.getFieldToVehicle().getRotation().getDegrees()
    heading_setpoint = Rotation2d.fromDegrees(deg).rotateBy(Rotation2d.fromDegrees(offset))
    print("Heading set to: " + str(heading_setpoint))

def getHeadingError():
    current_heading = odometry.getFieldToVehicle().getRotation()
    return current_heading.inverse().rotateBy(heading_setpoint).getDegrees()

turn_speed = 0.15
sensors.setLeftMotor(-turn_speed * angle / math.fabs(angle))
sensors.setRightMotor(turn_speed * angle / math.fabs(angle))

print("Turning {:.2f} degrees".format(angle))
setHeading(angle)

turning = True
while(turning):

    sensors.updateSensors()
    odometry.updateOdometry()

    error = getHeadingError()
    print(error)

    if(math.fabs(getHeadingError()) <= 3):
        print("Done Turning")
        turning = False

    time.sleep(dt)

sensors.shutdown()
