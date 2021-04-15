import sensors
import time

sensors.initSensors()

sensors.setRampAngle(0)
time.sleep(3)
sensors.setRampAngle(60)
time.sleep(60)

sensors.shutdown()
