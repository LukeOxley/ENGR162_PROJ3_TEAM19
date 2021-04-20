import math
from transforms import RigidTransform2d, Rotation2d, Translation2d
import sensors
import map

mag_possible_threshold = 250 # THE Z COMP THRESH
ir_possible_threshold = 3

class HazardDirection:
    maxIRReading = 0
    maxIRReadingHeading = Rotation2d.fromDegrees(0)
    maxMagReading = 0
    maxMagReadingMagnitude = 0
    maxMagReadingHeading = Rotation2d.fromDegrees(0)
    class DetectionState:
        UNKOWN = 0
        NOTPRES = 1
        PRES = 2
    def __init__(self, wall):
        if(wall):
            self.possible_mag = self.DetectionState.NOTPRES
            self.possible_ir = self.DetectionState.NOTPRES
        else:
            self.possible_mag = self.DetectionState.UNKOWN
            self.possible_ir = self.DetectionState.UNKOWN
    max_counts = 25 # PARAMATER TO TUNE (dt is 0.01), 25 is 1/4 s
    counts_since_ir_max = 0
    counts_since_mag_max = 0
    def updateScan(self, IR, MAG_Z, MAG_MAG, heading):
        if(self.possible_ir == self.DetectionState.UNKOWN):
            self.possible_ir = self.DetectionState.NOTPRES
        if(self.possible_mag == self.DetectionState.UNKOWN):
            self.possible_mag = self.DetectionState.NOTPRES
        
        if(IR > self.maxIRReading):
            self.maxIRReading = IR
            self.maxIRReadingHeading = heading
            self.counts_since_ir_max = 0
            if(self.maxIRReading > ir_possible_threshold and \
                self.possible_ir != self.DetectionState.PRES):
                self.possible_ir = self.DetectionState.PRES
                print("New IR hazard found in scan")
        else:
            self.counts_since_ir_max += 1
        if(MAG_Z > self.maxMagReading):
            self.maxMagReading = MAG_Z
            self.maxMagReadingMagnitude = MAG_MAG
            self.maxMagReadingHeading = heading
            self.counts_since_mag_max = 0
            if(self.maxMagReading > mag_possible_threshold and \
                self.possible_mag != self.DetectionState.PRES):
                print("New Mag hazard found in scan")
                self.possible_mag = self.DetectionState.PRES
        else:
            self.counts_since_mag_max += 1
    def doneScanning(self):
        return (self.counts_since_ir_max > self.max_counts) and \
               (self.counts_since_mag_max > self.max_counts)

def magHazardExists():
    x, y, z = sensors.getMagneticLevel()
    return math.fabs(z) > mag_possible_threshold

def getMagneticDistanceFromReading(reading):
    return 378.15 * math.pow(reading, -0.563)
def getMagneticDistance():
    return getMagneticDistanceFromReading(sensors.getMagneticMagnitude())

def irHazardExists():
    return sensors.getIRLevelLeft() + sensors.getIRLevelRight() > ir_possible_threshold

def getIRDistanceFromReading(reading):
    if(reading > 0):
        return -30.04 * math.log(reading) + 164.58
    else:
        return 0
    
def getIRDistanceLeft():
    return getIRDistanceFromReading(sensors.getIRLevelLeft())

def startDirectionalScan(left_wall, fwd_wall, right_wall):
    sensors.updateSensors()
    global left_haz, fwd_haz, right_haz
    left_haz = HazardDirection(left_wall)
    fwd_haz = HazardDirection(fwd_wall)
    right_haz = HazardDirection(right_wall)
    if(not magHazardExists()):
        fwd_haz.possible_mag = HazardDirection.DetectionState.NOTPRES
    if(not irHazardExists()):
        fwd_haz.possible_ir = HazardDirection.DetectionState.NOTPRES
    
def needToScanLeft():
    global left_haz
    return left_haz.possible_mag == HazardDirection.DetectionState.UNKOWN \
            or left_haz.possible_ir == HazardDirection.DetectionState.UNKOWN
    
def needToScanRight():
    global right_haz
    return right_haz.possible_mag == HazardDirection.DetectionState.UNKOWN \
            or right_haz.possible_ir == HazardDirection.DetectionState.UNKOWN

def needToScanFwd():
    global fwd_haz
    return fwd_haz.possible_mag == HazardDirection.DetectionState.UNKOWN \
            or fwd_haz.possible_ir == HazardDirection.DetectionState.UNKOWN
    
def leftHazardPresent():
    global left_haz
    return left_haz.possible_mag == HazardDirection.DetectionState.PRES\
            or left_haz.possible_ir == HazardDirection.DetectionState.PRES

def rightHazardPresent():
    global right_haz
    return right_haz.possible_mag == HazardDirection.DetectionState.PRES \
            or right_haz.possible_ir == HazardDirection.DetectionState.PRES

def frontHazardPresent():
    global fwd_haz
    return fwd_haz.possible_mag == HazardDirection.DetectionState.PRES \
            or fwd_haz.possible_ir == HazardDirection.DetectionState.PRES

def updateLeftScan(IR, Mag_z, Mag_Mag, heading):
    global left_haz
    left_haz.updateScan(IR, Mag_z, Mag_Mag, heading)
    return left_haz.doneScanning()

def updateRightScan(IR, Mag_z, Mag_Mag, heading):
    global right_haz
    right_haz.updateScan(IR, Mag_z, Mag_Mag, heading)
    return right_haz.doneScanning()

def updateFwdScan(IR, Mag_z, Mag_Mag, heading):
    global fwd_haz
    fwd_haz.updateScan(IR, Mag_z, Mag_Mag, heading)
    return fwd_haz.doneScanning()

scan_range = 30 # 30 for each direction from the center line
def updateAllScans(heading, start_heading):
    IR = sensors.getIRLevelLeft()
    x, y, Mag_z = sensors.getMagneticLevel()
    Mag_Mag = sensors.getMagneticMagnitude()
    heading_delta = start_heading.inverse().rotateBy(heading)

    if(math.fabs(start_heading.inverse().rotateBy(heading).getDegrees()) < scan_range):
        # In range for front
        updateFwdScan(IR, Mag_z, Mag_Mag, heading_delta)
        #print("Updating FRONT SCAN")
    elif(math.fabs(start_heading.rotateBy(Rotation2d(0,1,False)).inverse()\
        .rotateBy(heading).getDegrees()) < scan_range):
        # In range for right
        updateLeftScan(IR, Mag_z, Mag_Mag, heading_delta)
        #print("Updating LEFT SCAN")
    elif(math.fabs(start_heading.rotateBy(Rotation2d(0,-1,False)).inverse()\
        .rotateBy(heading).getDegrees()) < scan_range):
        # In range for right
        updateRightScan(IR, Mag_z, Mag_Mag, heading_delta)
        #print("Updating RIGHT SCAN")

def logAllScans(start_rigid):
    global fwd_haz
    global left_haz
    global right_haz
    if(frontHazardPresent()):
        if(fwd_haz.possible_ir == HazardDirection.DetectionState.PRES):
            map.logHeatSource(start_rigid.transfromBy(
                RigidTransform2d(Translation2d(getIRDistanceFromReading(fwd_haz.maxIRReading), 0), fwd_haz.maxIRReadingHeading)).getTranslation(), 
                fwd_haz.maxIRReading)
        if(fwd_haz.possible_mag == HazardDirection.DetectionState.PRES):
            map.logHeatSource(start_rigid.transfromBy(
                RigidTransform2d(Translation2d(getMagneticDistanceFromReading(fwd_haz.maxMagReadingMagnitude), 0), fwd_haz.maxMagReadingHeading)).getTranslation(), 
                fwd_haz.maxMagReadingMagnitude)
    if(leftHazardPresent()):
        if(left_haz.possible_ir == HazardDirection.DetectionState.PRES):
            map.logHeatSource(start_rigid.transfromBy(
                RigidTransform2d(Translation2d(getIRDistanceFromReading(left_haz.maxIRReading), 0), left_haz.maxIRReadingHeading)).getTranslation(), 
                left_haz.maxIRReading)
        if(left_haz.possible_mag == HazardDirection.DetectionState.PRES):
            map.logHeatSource(start_rigid.transfromBy(
                RigidTransform2d(Translation2d(getMagneticDistanceFromReading(left_haz.maxMagReadingMagnitude), 0), left_haz.maxMagReadingHeading)).getTranslation(), 
                left_haz.maxMagReadingMagnitude)
    if(rightHazardPresent()):
        if(right_haz.possible_ir == HazardDirection.DetectionState.PRES):
            map.logHeatSource(start_rigid.transfromBy(
                RigidTransform2d(Translation2d(getIRDistanceFromReading(fwd_haz.maxIRReading), 0), fwd_haz.maxIRReadingHeading)).getTranslation(), 
                fwd_haz.maxIRReading)
        if(fwd_haz.possible_mag == HazardDirection.DetectionState.PRES):
            map.logHeatSource(start_rigid.transfromBy(
                RigidTransform2d(Translation2d(getMagneticDistanceFromReading(fwd_haz.maxMagReadingMagnitude), 0), fwd_haz.maxMagReadingHeading)).getTranslation(), 
                fwd_haz.maxMagReadingMagnitude)
    
