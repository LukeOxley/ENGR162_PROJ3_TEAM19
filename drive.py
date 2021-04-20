import map
import sensors
import math
import odometry
from transforms import RigidTransform2d, Rotation2d, Translation2d
import time
import hazard_detection
#from robot import Robot
# determines possible navigation options
# interfaces with maze solver to find which directions to go
# interfaces with odometry to give it an action update
# stays between the walls

class Drive():

    hallway_width = 40 #cm
    robot_width = 21 # cm
    robot_length = 25 #cm
    robot_b = 10 #14 #12 # cm distance between front and side sensor
    robot_p = 0 # cm distance between center and pivot point (make <0 if pivot infront of center)
    hallway_speed = 0.15
    kP = 0.4 #0.05 # % of speed to add per cm in error
    kLimit = 0.2 # max % of speed to add (error * kP)
    kPHeading = 0.015 #0.008

    min_side_int_dist = hallway_width - 5
    min_front_int_dist = hallway_width

    int_front_fwd_dist = (robot_length + hallway_width)/2 + robot_p
    int_side_fwd_dist = (robot_length + hallway_width)/2-robot_b+robot_p
    intersection_forward_dist = 0
    int_exit_distance = hallway_width - int_front_fwd_dist + robot_b + 2 # where 2 is cm past the clearing of the side sensors

    intersection_enabled = True
    entering_enabled = True

    started_entering = False

    class Drive_State_t:
        IDLE = "IDLE"
        ENTERING = "ENTERING"
        HALLWAY_FOLLOWING = "HALLWAY_FOLLOWING"
        INTERSECTION_ENTERING = "INTERSECTION_ENTERING"
        HAZARD_SCANNING = "HAZARD_SCANNING"
        INTERSECTION_EXITING = "INTERSECTION_EXITING"
        TURNING_AROUND = "TURNING_AROUND" # Added just for if it sees a hazard in hallway
        DELIVERING = "DELIVERING"

    state = Drive_State_t.IDLE

    def __init__(self, robot, gui):
        self.robot = robot
        self.gui = gui

    def initDrive(self):
        self.state = self.Drive_State_t.ENTERING
        self.started_entering = False
        self.int_enter_state = 0
        self.hzd_scan_state = 0
        self.int_exit_state = 0
        self.turn_around_state = 0
        self.deliver_state = 0

    def updateDrive(self):
        if(self.state == self.Drive_State_t.IDLE):
            self.handleIdle()
        elif(self.state == self.Drive_State_t.ENTERING):
            self.handleEntering()
        elif(self.state == self.Drive_State_t.HALLWAY_FOLLOWING):
            self.handleHallwayFollowing()
        elif(self.state == self.Drive_State_t.INTERSECTION_ENTERING):
            self.handleIntersectionEntering()
        elif(self.state == self.Drive_State_t.HAZARD_SCANNING):
            self.handleHazardScanning()
        elif(self.state == self.Drive_State_t.INTERSECTION_EXITING):
            self.handleIntersectionExiting()
        elif(self.state == self.Drive_State_t.TURNING_AROUND):
            self.handleTurningAround()
        elif(self.state == self.Drive_State_t.DELIVERING):
            self.handleDelivering()

    def setHeading(self, offset):
        sensors.updateSensors()
        deg = self.robot.odometry.getFieldToVehicle().getRotation().getDegrees()
        deg = round(deg / 90) * 90  
        self.heading_setpoint = Rotation2d.fromDegrees(deg).rotateBy(Rotation2d.fromDegrees(offset))
        print("Heading set to: " + str(self.heading_setpoint))

    def getHeadingError(self):
        current_heading = self.robot.odometry.getFieldToVehicle().getRotation()
        return current_heading.inverse().rotateBy(self.heading_setpoint).getDegrees()
    
    def handleIdle(self):
        pass

    heading_setpoint = Rotation2d.fromDegrees(0)
    
    entering_fwd_speed = hallway_speed #0.15
    def handleEntering(self):
        if(self.entering_enabled):
            if(not self.started_entering):
                sensors.setLeftMotor(self.entering_fwd_speed)
                sensors.setRightMotor(self.entering_fwd_speed)
                self.started_entering = True
            else:
                if(not self.getLeftAvailable() and not self.getRightAvailable()):
                    # drive until left and right see the walls
                    # motor off may be removed
                    sensors.setMotorOff()
                    print("Entered Maze")
                    self.gui.log_message("Entered Maze")
                    map.logStartPoint(self.robot.odometry.getFieldToVehicle().transformBy( \
                                    RigidTransform2d(Translation2d(self.hallway_width/2, 0), \
                                    Rotation2d(1, 0, False))).getTranslation())
                    self.state = self.Drive_State_t.HALLWAY_FOLLOWING
                    self.started_entering = False
                    self.setHeading(0)
        else:
            print("Entering bypassed")
            self.gui.log_message("Entering Bypassed")
            self.state = self.Drive_State_t.HALLWAY_FOLLOWING
            map.logStartPoint(self.robot.odometry.getFieldToVehicle().getTranslation())
            self.setHeading(0)

    def handleHallwayFollowing(self):

        if(self.intersection_enabled):
            if(hazard_detection.irHazardExists() or hazard_detection.magHazardExists()):
                # jeepers, get outta here!
                if(hazard_detection.irHazardExists()):
                    map.logHeatSource(self.robot.odometry.getFieldToVehicle().transformBy(
                                    RigidTransform2d(Translation2d(hazard_detection.getIRDistanceLeft(), 0), 
                                                Rotation2d(1,0,False))).getTranslation(), sensors.getIRLevelLeft())
                    self.gui.log_message("IR Detected, turning around")
                if(hazard_detection.magHazardExists()):
                    map.logMagneticSource(self.robot.odometry.getFieldToVehicle().transformBy(
                                    RigidTransform2d(Translation2d(hazard_detection.getIRDistanceLeft(), 0), 
                                                Rotation2d(1,0,False))).getTranslation(), sensors.getMagneticMagnitude())
                    self.gui.log_message("Mag Detected, turning around")
                self.state = self.Drive_State_t.TURNING_AROUND
                sensors.setMotorOff()
                time.sleep(1)
                return
            if(self.getLeftAvailable() or self.getRightAvailable()):
                self.intersection_forward_dist = self.int_side_fwd_dist
                self.state = self.Drive_State_t.INTERSECTION_ENTERING
                self.gui.log_message("Left or right path open")
                sensors.setMotorOff()
                time.sleep(1)
                return
            elif(not self.getFrontAvailable()):
                self.intersection_forward_dist = self.int_front_fwd_dist
                self.state = self.Drive_State_t.INTERSECTION_ENTERING
                self.gui.log_message("Forward closed")
                sensors.setMotorOff()
                time.sleep(1)
                return
            

        # >0 if too far to the right, <0 if too far left
        # if error >0 => correct by increasing right speed
        error = sensors.getLeftWallDistance() - sensors.getRightWallDistance()
        # use error to control difference in motor speed
        pError = error*self.kP*self.hallway_speed
        if(math.fabs(pError) > self.kLimit):
            pError = self.kLimit*(pError/math.fabs(pError))
        leftSpeed = self.hallway_speed
        rightSpeed = leftSpeed
        if(pError > 0):
            rightSpeed += pError
        else:
            leftSpeed -= pError
        # leftSpeed = self.hallway_speed - pError
        # rightSpeed = self.hallway_speed + pError

        heading_error = self.getHeadingError()
        lSpeed = self.hallway_speed - heading_error*self.kPHeading 
        rSpeed = self.hallway_speed + heading_error*self.kPHeading 

        #sensors.setLeftMotor(leftSpeed)
        #sensors.setRightMotor(rightSpeed)
        sensors.setLeftMotor(lSpeed)
        sensors.setRightMotor(rSpeed)

    int_enter_state = 0
    int_st_speed = hallway_speed
    int_turn_speed = 0.15
    int_turn_tolerance = 2 #3

    def handleIntersectionEntering(self):
        if(self.int_enter_state == 0):
            # first call
            self.gui.log_message("Centering into square")
            # begin driving forward
            sensors.setLeftMotor(self.int_st_speed)
            sensors.setRightMotor(self.int_st_speed)
            self.int_start_pos = self.robot.odometry.getFieldToVehicle()
            self.int_enter_state += 1
        elif(self.int_enter_state == 1):
            if(self.int_start_pos.inverse().transformBy(self.robot.odometry.getFieldToVehicle()).getTranslation().norm() >= self.intersection_forward_dist):
                # the current position should be approximately the center of the intersection
                sensors.setMotorOff()
                sensors.updateSensors()
                time.sleep(1)
                self.gui.log_message("Centered, Searching For Hazards")
                # Log current pos as an intersection point
                map.logIntersection(self.robot.odometry.getFieldToVehicle().getTranslation())

                # Transition to hazard search
                self.state = self.Drive_State_t.HAZARD_SCANNING
                self.int_enter_state = 0
                     
    hzd_scan_state = 0
    def handleHazardScanning(self):
        if(self.hzd_scan_state == 0):
            # begins looking for hazards in all three directions, goal is to narrow down which directions need to be scanned
            sensors.updateSensors()
            self.hzd_lt_avail = self.getLeftAvailable()
            self.hzd_rt_avail = self.getRightAvailable()
            self.hzd_ft_avail = self.getFrontAvailable()

            hazard_detection.startDirectionalScan(not self.getLeftAvailable(), not self.getFrontAvailable(), not self.getRightAvailable())
            self.hzd_start_rigid = self.robot.odometry.getFieldToVehicle()
            self.hzd_start_heading = self.robot.odometry.getFieldToVehicle().getRotation()
            self.hzd_scan_state += 1
        else:
            # Updates all scan directions, chooses which to update based on if heading is in correct direction
            hazard_detection.updateAllScans(self.robot.odometry.getFieldToVehicle().getRotation(), self.hzd_start_heading)
        
        if(self.hzd_scan_state == 1):
            # see whats left to scan
            sensors.updateSensors()

            #if(hazard_detection.needToScanFwd()):

            if(hazard_detection.needToScanLeft()):
                # turn left 90
                self.hzd_turn_direction = map.Turn_Direction_Robot_t.LFT
                self.gui.log_message("Scanning Left")
            elif(hazard_detection.needToScanRight()):
                # turn right 90
                self.hzd_turn_direction = map.Turn_Direction_Robot_t.RHT
                self.gui.log_message("Scanning Right")
            else:
                #completely done scanning!!!
                if(self.hzd_rt_avail and not hazard_detection.rightHazardPresent()):
                    self.hzd_turn_direction = map.Turn_Direction_Robot_t.RHT
                    self.gui.log_message("Turning Right")
                elif(self.hzd_ft_avail and not hazard_detection.frontHazardPresent()):
                    self.hzd_turn_direction = map.Turn_Direction_Robot_t.FWD
                    self.gui.log_message("Turning Forward")
                elif(self.hzd_lt_avail and not hazard_detection.leftHazardPresent()):
                    self.hzd_turn_direction = map.Turn_Direction_Robot_t.LFT
                    self.gui.log_message("Turning Left")
                else:
                    # no available dirs, turn around
                    self.hzd_turn_direction = map.Turn_Direction_Robot_t.BCK
                    self.gui.log_message("Turning Around")

            heading_offset = self.hzd_start_heading.inverse().rotateBy(self.robot.odometry.getFieldToVehicle().getRotation()).getDegrees()
            heading_change = self.hzd_turn_direction - heading_offset
            if(math.fabs(heading_change) > 45):
                # still need to change direction
                self.setHeading(heading_change)
                self.hzd_scan_state += 1
                mult = 1
                if(heading_change < 0):
                    mult = -1
                sensors.setLeftMotor(-self.int_turn_speed * mult)
                sensors.setRightMotor(self.int_turn_speed * mult)
            else:
                # we are already at the deired location!
                # move to last step in hazard scanning
                self.hzd_scan_state += 2

        elif(self.hzd_scan_state == 2):
            # Turning
            if(math.fabs(self.getHeadingError()) <= self.int_turn_tolerance):
                # turning complete
                self.gui.log_message("Scan Turn Complete")
                sensors.setMotorOff()
                self.hzd_scan_state = 1

        elif(self.hzd_scan_state == 3):
            # log the hazards and scram to the exit intersection mode
            self.gui.log_message("Done with hazard detection")
            hazard_detection.logAllScans(self.hzd_start_rigid)
            self.state = self.Drive_State_t.INTERSECTION_EXITING
            self.hzd_scan_state = 0
        
    int_exit_state = 0
    def handleIntersectionExiting(self):
        if(self.int_exit_state == 0):
            # drive forward to exit the intersection
            self.gui.log_message("Exiting Intersection")
            sensors.setLeftMotor(self.int_st_speed - self.getHeadingError()*self.kPHeading)
            sensors.setRightMotor(self.int_st_speed + self.getHeadingError()*self.kPHeading)
            self.int_start_pos = self.robot.odometry.getFieldToVehicle()
            self.left_was_available = self.getLeftAvailable()
            self.right_was_available = self.getRightAvailable()
            self.int_exit_state += 1
        elif(self.int_exit_state == 1):
            # Exiting Intersection State

            sensors.setLeftMotor(self.int_st_speed - self.getHeadingError()*self.kPHeading)
            sensors.setRightMotor(self.int_st_speed + self.getHeadingError()*self.kPHeading)
            # continue to check for forward wall
            if(not self.getFrontAvailable()):
                self.intersection_forward_dist = self.int_front_fwd_dist
                self.state = self.Drive_State_t.INTERSECTION_ENTERING
                self.gui.log_message("Forward closed while exiting intersection")
                sensors.setMotorOff()
                self.int_exit_state = 0
                time.sleep(1)
                return
            # check for a change in state of the sonic from wall to no wall (sides)
            if((self.left_was_available and not self.getLeftAvailable()) or
                (self.right_was_available and not self.getRightAvailable())):
                self.intersection_forward_dist = self.int_side_fwd_dist
                self.state = self.Drive_State_t.INTERSECTION_ENTERING
                self.gui.log_message("Left or right path open while exiting intersection")
                sensors.setMotorOff()
                self.int_exit_state = 0
                time.sleep(1)
                return
            # distance between current pos and start pos
            if(self.int_start_pos.inverse().transformBy(self.robot.odometry.getFieldToVehicle()).getTranslation().norm() >= self.int_exit_distance):
                sensors.setMotorOff()
                sensors.updateSensors()
                # check to ensure we are still in a maze
                if(self.getLeftAvailable() or self.getRightAvailable()):
                    # we have exited the maze lol yeet
                    self.gui.log_message("Maze Exited")
                    map.logEndPoint(self.robot.odometry.getFieldToVehicle().transformBy( \
                                    RigidTransform2d(Translation2d(-self.hallway_width/2, 0), \
                                    Rotation2d(1, 0, False))).getTranslation())
                    self.state = self.Drive_State_t.DELIVERING
                else:
                    # Keep going, you can do it little buddy!
                    self.gui.log_message("Continuing Hallway Following")
                    self.int_exit_state = 0
                    self.setHeading(0)
                    self.state = self.Drive_State_t.HALLWAY_FOLLOWING

    turn_around_state = 0
    def handleTurningAround(self):
        # turn around if hazard detected in hallway
        if(self.turn_around_state == 0):
            self.setHeading(180)
            sensors.setLeftMotor(-self.int_turn_speed)
            sensors.setRightMotor(self.int_turn_speed)
            self.turn_around_state += 1
        else:
            if(math.fabs(self.getHeadingError()) <= self.int_turn_tolerance):
                # turning complete
                self.gui.log_message("Turn Around Complete")
                sensors.setMotorOff()
                self.turn_around_state = 0
                self.state = self.Drive_State_t.HALLWAY_FOLLOWING

    deliver_state = 0
    deliver_distance = 5
    def handleDelivering(self):
        # Deliver the goods
        if(self.deliver_state == 0):
            sensors.setRampAngle(60)
            self.deliver_start_pos = self.robot.odometry.getFieldToVehicle()
            sensors.setLeftMotor(self.int_st_speed)
            sensors.setRightMotor(self.int_st_speed)
            self.deliver_state = 1
        elif(self.deliver_state == 1):
            if(self.deliver_start_pos.inverse().transformBy(self.robot.odometry.getFieldToVehicle()).getTranslation().norm() >= self.deliver_distance):
                sensors.setMotorOff()
                sensors.setRampAngle(0)
                self.deliver_state = 0
                self.gui.log_message("Package Delivered")
                self.state = self.Drive_State_t.IDLE
        
    def getLeftAvailable(self):
        return sensors.getLeftWallDistance() > self.min_side_int_dist
    def getRightAvailable(self):
        return sensors.getRightWallDistance() > self.min_side_int_dist
    def getFrontAvailable(self):
        return sensors.getForwardWallDistance() > self.min_front_int_dist

    def setEnableIntersection(self, state):
        self.intersection_enabled = state

    def setEnableEntering(self, state):
        self.entering_enabled = state
    
