import sensors
import time

sensors.initSensors()
# left A Right B
sensors.setLeftMotor(0.3)
sensors.setRightMotor(-0.3)

while(True):


    time.sleep(0.2)


