# Goal is to periodically update raw sensor data
# instead of multiple times in one loop cycle
# This prevents spamming of the sensors
# prolly want some sort of try catch on each sensor read
# for the raw data, use like updateHeadingRaw -> this will
# read the actual sensor and update the corresponding variable
# all of the raw functions will be called in updateSensors()
from MPU9250 import MPU9250
import brickpi3 # import the BrickPi3 drivers
import grovepi
import math
from transforms import Rotation2d
import time
import hazard_detection

BP = brickpi3.BrickPi3()
mpu = MPU9250()
heading_offset = 0

def initSensors():
    global BP
    global heading_offset

    BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.EV3_GYRO_ABS)
    BP.set_sensor_type(BP.PORT_3, BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)
    #BP.set_motor_limits(BP.PORT_C, 25, 800)
    #Reset Left wheel encoder to 0
    try:
      BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
    except:
      pass
    #Reset Right wheel encoder to 0
    try:
      BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    except:
      pass
    try:
      BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))
    except:
      pass
    # reset gyro heading to 0
    print("Calibrating gyro")
    success = False
    while(not success):
      try:
        heading_offset = 90 + BP.get_sensor(BP.PORT_2)
        success = True
      except brickpi3.SensorError as error:
        #print(error)
        pass
    print("Gyro Calibrated")

counter_max = 6
counter = 0
mag = {'x':0,'y':0,'z':0}

def updateSensors():
    #Gives program wide scope
    global heading, leftEncoder, rightEncoder, leftWallDistance
    global rightWallDistance, frontWallDistance, mag
    global irLevelLeft, irLevelRight, touchSensor
    global counter, counter_max

    global BP
    global mpu

    #set based on robot (grove pi ports)
    leftUltraPin = 2
    rightUltraPin = 7
    frontUltraPin = 4
    irPinLeft = 0
    irPinRight = 1

    period = 0 #0.002

    #Plug in the left motor to port A, the right to port B, touch sensor
    #to port 1, Gyro to port 2 and if required the NXT ultra sonic to port 3
    try:
      leftWallDistance = grovepi.ultrasonicRead(leftUltraPin)
    except Exception as e:
      print ("Error:{}".format(e))

    #time.sleep(period)

    try:
      rightWallDistance = grovepi.ultrasonicRead(rightUltraPin)
    except Exception as e:
      print ("Error:{}".format(e))

    #time.sleep(period)

    try:
      frontWallDistance = grovepi.ultrasonicRead(frontUltraPin)
    except Exception as e:
      print ("Error:{}".format(e))

    #time.sleep(period)

    try:
      leftEncoder = BP.get_motor_encoder(BP.PORT_A)
    except IOError as error:
      print("Left Encoder: " + str(error))

    #time.sleep(period)

    try:
      rightEncoder = BP.get_motor_encoder(BP.PORT_B)
    except IOError as error:
      print("Right Encoder: " + str(error))

    #time.sleep(period)

    
    if(counter < counter_max):
        counter += 1
    else:
        counter = 0
        try:
          mag = mpu.readMagnet()
        except:     #not sure what exception error for this would be
          pass

    #time.sleep(period)

    try:
      irLevelLeft = grovepi.analogRead(irPinLeft); #reads the left IR sensor
    except brickpi3.SensorError as error:
      print("Left IR: " + str(error))

    #time.sleep(period)

    try:
      irLevelRight = grovepi.analogRead(irPinRight); #reads the right IR sensor
    except brickpi3.SensorError as error:
      print("Right IR: " + str(error))

    #time.sleep(period)

    try:
      heading = BP.get_sensor(BP.PORT_2) - heading_offset
    except brickpi3.SensorError as error:
      heading = 0
      print("Heading:" + str(error))

    #time.sleep(period)

    # try:
    #   touchSensor = BP.get_sensor(BP.PORT_1) #1 or 0
    # except brickpi3.SensorError as error:
    #   print(error)

def getHeading():
    return Rotation2d.fromDegrees(float(-heading))

def getLeftWheelDistance():
    radius = 2.75 #radius of the wheels (cm)
    theta = float(leftEncoder) * 40.0 / 24.0 # degrees (40 to 24 ratio)
    rotations = theta / 360
    distance = rotations * radius * 2 * math.pi #cm
    return distance; # cm

def getRightWheelDistance():
    radius = 2.75 #radius of the wheels (cm)
    theta = float(rightEncoder) * 40.0 / 24.0 # degrees (40 to 24 ratio)
    rotations = theta / 360
    distance = rotations * radius * 2 * math.pi
    return distance; # cm

def getLeftWallDistance():
    global leftWallDistance
    return float(leftWallDistance) # cm

def getForwardWallDistance():
    global frontWallDistance
    return float(frontWallDistance) # cm

def getRightWallDistance():
    global rightWallDistance
    return float(rightWallDistance) # cm

def getIRLevelLeft():
    global irLevelLeft
    return float(irLevelLeft); # not sure on the units
    #return 0

def getIRLevelRight():
    global irLevelRight
    return float(irLevelRight); # not sure on the units
    #return 0

def getMagneticLevel():
    global mag
    return float(mag['x']), float(mag['y']), float(mag['z']) #returns them into component form

def getMagneticMagnitude():
    x, y, z = getMagneticLevel()
    return math.sqrt(math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2))

# def getTouchSensor():
#     return int(touchSensor); #1 for pressed 0 for not pressed

def setLeftMotor(speed):
    BP.set_motor_dps(BP.PORT_A, 1050*speed)

def setRightMotor(speed):
    BP.set_motor_dps(BP.PORT_B, 1050*speed)

def setRampAngle(angle):
  BP.set_motor_position(BP.PORT_C, -angle)

def setMotorOff():
    setLeftMotor(0)
    setRightMotor(0)
  
def shutdown():
  print("Turning Off Sensors")
  setMotorOff()
  BP.reset_all()

if __name__ == '__main__':
  import time
  initSensors()
  while(True):
    updateSensors()

    # Clear screen to keep positions constant
    print(chr(27)+'[2j')
    print('\033c')
    print('\x1bc')

    print("------Sonics------")
    print("Left: {:6.2f}, Front: {:6.2f}, Right: {:6.2f}".format(getLeftWallDistance(), getForwardWallDistance(), getRightWallDistance()))
    print("--------Mag-------")
    print("Magnitude: {:.3f}".format(getMagneticMagnitude()))
    x, y, z = getMagneticLevel()
    print("X: {:10.3f}, Y: {:10.3f}, Z: {:10.3f}".format(x, y, z))
    print("Distance: {:10.3f}".format(hazard_detection.getMagneticDistanceFromReading(getMagneticMagnitude())))
    print("--------IR--------")
    print("Left: {:6.2f}, Right: {:6.2f}".format(getIRLevelLeft(), getIRLevelRight()))
    print("Distance: {:10.3f}".format(hazard_detection.getIRDistanceFromReading(getIRLevelLeft())))
    print("-------GYRO-------")
    print("Heading: {:6.2f}".format(getHeading().getDegrees()))
    time.sleep(0.01)#.25
