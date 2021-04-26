import sensors
import time

file_name = 'max_mag.txt'

sensors.initSensors()

max_val = 0
sensors.setLeftMotor(-0.15)
sensors.setRightMotor(0.15)

try:
    while True:
        sensors.updateSensors()
        curr = sensors.getMagneticMagnitude()
        max_val = max(curr, max_val)
        time.sleep(0.01)
except KeyboardInterrupt:
    print("Done Scanning")
    print("Max Val: ")
    print(max_val)
    sensors.shutdown()
    with open(file_name, 'w') as f:
        f.write(str(max_val))

