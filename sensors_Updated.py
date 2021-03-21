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

def initSensors():
    #Reset Left wheel encoder to 0
    try:
      offset_motor_encoder(self, PORT_A, BP.get_motor_encoder(BP.PORT_A))
    except:
      pass
    #Reset Right wheel encoder to 0
    try:
      offset_motor_encoder(self, PORT_B, BP.get_motor_encoder(BP.PORT_B))
    except:
      pass
    # reset gyro heading to 0
    try:
      BP.offset_sensor(self, BP.PORT_1, BP.get_sensor(BP.PORT_1))
    except brickpi3.SensorError as error:
      print(error)

def updateSensors():
    #Gives program wide scope
    global heading, leftEncoder, rightEncoder, leftWallDistance
    global rightWallDistance, frontWallDistance, mag
    global irLevelLeft, irLevelRight, touchSensor
    
    #set based on robot (grove pi ports)
    leftUltraPin = 1
    rightUltraPin = 2
    frontUltraPin = 3
    irPinLeft = 4
    irPinRight = 5
    
    #Plug in the left motor to port A, the right to port B, touch sensor
    #to port 1, Gyro to port 2 and if required the NXT ultra sonic to port 3
    try:
      leftWallDistance = grovepi.ultrasonicRead(leftUltraPin)
    except Exception as e:
      print ("Error:{}".format(e))

    try:
      rightWallDistance = grovepi.ultrasonicRead(rightUltraPin)
    except Exception as e:
      print ("Error:{}".format(e))

    try:
      frontWallDistance = grovepi.ultrasonicRead(frontUltraPin)
      #If we need to use the NXT on the brick pi
      #BP.get_sensor(BP.PORT_3)
    except Exception as e:
      print ("Error:{}".format(e))

    try:
      leftEncoder = BP.get_motor_encoder(BP.PORT_A)
    except IOError as error:
      print(error)

    try:
      rightEncoder = BP.get_motor_encoder(BP.PORT_B)
    except IOError as error:
      print(error)

    try:
      mag = MPU9250.readMagnet()
    except:     #not sure what exception error for this would be
      pass

    try:
      irLevelLeft = grovepi.analogRead(irPinLeft)[0]; #reads the left IR sensor
    except brickpi3.SensorError as error:
      print(error)

    try:
      irLevelRight = grovepi.analogRead(irPinRight)[0]; #reads the right IR sensor
    except brickpi3.SensorError as error:
      print(error)

    try:
      heading = BP.get_sensor(BP.PORT_2);
    except brickpi3.SensorError as error:
      print(error)

    try:
      touchSensor = BP.get_sensor(BP.PORT_1) #1 or 0
    except brickpi3.SensorError as error:
      print(error)

def getHeading():
    return map.Rotation(float(heading)); #rotation (deg)

def getLeftWheelDistance():
    radius = 0 #radius of the wheels (cm)
    theta = float(leftEncoder) # degrees
    rotations = theta / 360;
    distance = rotations * radius * 2 * math.pi #cm
    return distance; # cm

def getRightWheelDistance():
    radius = 0 #radius of the wheels (cm)
    theta = float(rightEncoder) # degrees
    rotations = theta / 360;
    distance = rotations * radius * 2 * math.pi 
    return distance; # cm

def getLeftWallDistance():
    return float(leftWallDistance) # cm

def getForwardWallDistance():
    return float(frontWallDistance) # cm

def getRightWallDistance():
    return float(rightWallDistance) # cm

def getIRLevelLeft():
    return float(irLevelLeft); # not sure on the units

def getIRLevelRight():
    return float(irLevelRight); # not sure on the units

def getMagneticLevel():
    return float(mag['x']), float(mag['y']), float(mag['z']) #returns them into component form

def getTouchSensor():
    return int(touchSensor); #1 for pressed 0 for not pressed
