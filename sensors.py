# Goal is to periodically update raw sensor data
# instead of multiple times in one loop cycle
# This prevents spamming of the sensors 
# prolly want some sort of try catch on each sensor read
# for the raw data, use like updateHeadingRaw -> this will
# read the actual sensor and update the corresponding variable
# all of the raw functions will be called in updateSensors()


def initSensors():
    # reset encoders
    # reset gyro heading to 0
    pass

def updateSensors():
    pass

def getHeading():
    return map.Rotation(0) #rotation (deg)
def getLeftWheelDistance():
    return 0 # cm
def getRightWheelDistance():
    return 0 # cm
def getLeftWallDistance():
    return 0 # cm
def getForwardWallDistance():
    return 0 # cm
def getRightWallDistance():
    return 0 # cm
def getIRLevel():
    return 0
def getMagneticLevel():
    return 0