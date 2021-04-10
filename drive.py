import map
import sensors
import math
import odometry
from transforms import RigidTransform2d, Rotation2d, Translation2d
import time
#from robot import Robot
# determines possible navigation options
# interfaces with maze solver to find which directions to go
# interfaces with odometry to give it an action update
# stays between the walls

class Drive():

    hallway_width = 40 #cm
    robot_width = 21 # cm
    robot_length = 25 #cm
    robot_b = 12 # cm distance between front and side sensor
    robot_p = 0 # cm distance between center and pivot point (make <0 if pivot infront of center)
    hallway_speed = 0.15
    kP = 0.1 #0.05 # % of speed to add per cm in error

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
        INTERSECTION = "INTERSECTION"
        DELIVERING = "DELIVERING"

    state = Drive_State_t.IDLE

    def __init__(self, robot, gui):
        self.robot = robot
        self.gui = gui

    def initDrive(self):
        self.state = self.Drive_State_t.ENTERING
        self.started_entering = False

    def updateDrive(self):
        if(self.state == self.Drive_State_t.IDLE):
            self.handleIdle()
        elif(self.state == self.Drive_State_t.ENTERING):
            self.handleEntering()
        elif(self.state == self.Drive_State_t.HALLWAY_FOLLOWING):
            self.handleHallwayFollowing()
        elif(self.state == self.Drive_State_t.INTERSECTION):
            self.handleIntersection()
        elif(self.state == self.Drive_State_t.DELIVERING):
            self.handleDelivering()
    
    def handleIdle(self):
        pass
    
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
        else:
            print("Entering bypassed")
            self.gui.log_message("Entering Bypassed")
            self.state = self.Drive_State_t.HALLWAY_FOLLOWING


    def handleHallwayFollowing(self):

        if(self.intersection_enabled):
            if(self.getLeftAvailable() or self.getRightAvailable()):
                self.intersection_forward_dist = self.int_side_fwd_dist
                self.state = self.Drive_State_t.INTERSECTION
                self.gui.log_message("Left or right path open")
                sensors.setMotorOff()
                return
            elif(not self.getFrontAvailable()):
                self.intersection_forward_dist = self.int_front_fwd_dist
                self.state = self.Drive_State_t.INTERSECTION
                self.gui.log_message("Forward closed")
                sensors.setMotorOff()
                time.sleep(5)
                return

        # >0 if too far to the right, <0 if too far left
        # if error >0 => correct by increasing right speed
        error = sensors.getLeftWallDistance() - sensors.getRightWallDistance()
        # use error to control difference in motor speed
        leftSpeed = self.hallway_speed - error*self.kP*self.hallway_speed
        rightSpeed = self.hallway_speed + error*self.kP*self.hallway_speed

        sensors.setLeftMotor(leftSpeed)
        sensors.setRightMotor(rightSpeed)

    int_state = 0
    int_st_speed = hallway_speed
    int_turn_speed = 0.15

    def handleIntersection(self):
        if(self.intersection_enabled):
            if(self.int_state == 0):
                # first call
                self.gui.log_message("Centering into square")
                # begin driving forward
                sensors.setLeftMotor(self.int_st_speed)
                sensors.setRightMotor(self.int_st_speed)
                self.int_start_pos = self.robot.odometry.getFieldToVehicle()
                self.int_state += 1
            elif(self.int_state == 1):
                # distance between current pos and start pos
                if(self.int_start_pos.inverse().transformBy(self.robot.odometry.getFieldToVehicle()).getTranslation().norm() >= self.intersection_forward_dist):
                    # the current position should be approximately the center of the intersection
                    sensors.setMotorOff()
                    self.gui.log_message("Centered")
                    time.sleep(10)
                    sensors.updateSensors()
                    # Log current pos as an intersection point
                    map.logIntersection(self.robot.odometry.getFieldToVehicle().getTranslation())
                    # determine what the best direction to go is
                    # ensure you also re-look at possible directions
                    self.int_turn_direction = map.Turn_Direction_Robot_t.BCK
                    if(self.getRightAvailable()):
                        self.int_turn_direction = map.Turn_Direction_Robot_t.RHT
                        self.gui.log_message("Turning Right")
                        sensors.setLeftMotor(self.int_turn_speed)
                        sensors.setRightMotor(-self.int_turn_speed)
                    elif(self.getFrontAvailable()):
                        self.gui.log_message("Moving Forward")
                        self.int_turn_direction = map.Turn_Direction_Robot_t.FWD
                    elif(self.getLeftAvailable()):
                        self.int_turn_direction = map.Turn_Direction_Robot_t.LFT
                        self.gui.log_message("Turning Left")
                        sensors.setLeftMotor(-self.int_turn_speed)
                        sensors.setRightMotor(self.int_turn_speed)
                    else:
                        self.int_turn_direction = map.Turn_Direction_Robot_t.BCK
                        self.gui.log_message("Turning Around")
                        sensors.setLeftMotor(-self.int_turn_speed)
                        sensors.setRightMotor(self.int_turn_speed)

                    # If necessary, turn, if not, skip over
                    if(self.int_turn_direction == map.Turn_Direction_Robot_t.FWD):
                        self.int_state += 2 # skip turning
                    else:
                        self.int_start_heading = self.robot.odometry.getFieldToVehicle().getRotation()
                        print("Start Heading: {:.2f}".format(self.int_start_heading.getDegrees()))
                        self.int_state += 1

            elif(self.int_state == 2):
                if(math.fabs(self.int_start_heading.inverse().rotateBy(self.robot.odometry.getFieldToVehicle().getRotation()).getDegrees()) >= math.fabs(self.int_turn_direction)):
                    print("End heading: {:.2f}".format(self.robot.odometry.getFieldToVehicle().getRotation().getDegrees()))
                    # turning complete
                    self.gui.log_message("Turning Complete")
                    sensors.setMotorOff()
                    self.int_state += 1

            elif(self.int_state == 3):
                # drive forward to exit the intersection
                self.gui.log_message("Exiting Intersection")
                sensors.setLeftMotor(self.int_st_speed)
                sensors.setRightMotor(self.int_st_speed)
                self.int_start_pos = self.robot.odometry.getFieldToVehicle()
                self.int_state += 1

            elif(self.int_state == 4):
                # distance between current pos and start pos
                if(self.int_start_pos.inverse().transformBy(self.robot.odometry.getFieldToVehicle()).getTranslation().norm() >= self.int_exit_distance):
                    sensors.setMotorOff()
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
                        self.state = self.Drive_State_t.HALLWAY_FOLLOWING
        else:
            self.state = self.Drive_State_t.HALLWAY_FOLLOWING

    def handleDelivering(self):
        # Deliver the goods
        pass
        
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
    