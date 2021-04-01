import map
import sensors
import math
import odometry
from transforms import RigidTransform2d, Rotation2d, Translation2d
from robot import Robot
# determines possible navigation options
# interfaces with maze solver to find which directions to go
# interfaces with odometry to give it an action update
# stays between the walls

class Drive():

    hallway_width = 40 #cm
    robot_width = 10 # cm
    robot_length = 20 #cm
    robot_b = 5 # cm distance between front and side sensor
    robot_p = 0 # cm distance between center and pivot point (make <0 if pivot infront of center)
    hallway_speed = 0.4
    kP = 0.05 # % of speed to add per cm in error

    min_side_int_dist = hallway_width - 5
    min_front_int_dist = hallway_width

    int_front_fwd_dist = (robot_length + hallway_width)/2 + robot_p
    int_side_fwd_dist = (robot_length + hallway_width)/2-robot_b+robot_p
    intersection_forward_dist = 0
    int_exit_distance = hallway_width - int_front_fwd_dist + robot_b + 2 # where 2 is cm past the clearing of the side sensors

    intersection_enabled = True
    entering_enabled = True

    class Drive_State_t:
        IDLE = "IDLE"
        ENTERING = "ENTERING"
        HALLWAY_FOLLOWING = "HALLWAY_FOLLOWING"
        INTERSECTION = "INTERSECTION"
        DELIVERING = "DELIVERING"

    state = Drive_State_t.IDLE

    def initDrive(self):
        state = self.Drive_State_t.IDLE

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
    
    entering_fwd_speed = 0.3
    started_entering = False
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
                    self.state = self.Drive_State_t.HALLWAY_FOLLOWING
                    self.started_entering = False
        else:
            self.state = self.Drive_State_t.HALLWAY_FOLLOWING


    def handleHallwayFollowing(self):

        if(self.getLeftAvailable() or self.getRightAvailable()):
            self.intersection_forward_dist = self.int_side_fwd_dist
            self.state = self.Drive_State_t.INTERSECTION
            sensors.setMotorOff()
            return
        elif(not self.getFrontAvailable()):
            self.intersection_forward_dist = self.int_front_fwd_dist
            self.state = self.Drive_State_t.INTERSECTION
            sensors.setMotorOff()
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
    int_turn_speed = 0.2

    def handleIntersection(self):
        if(self.intersection_enabled):
            if(self.int_state == 0):
                # first call
                # begin driving forward
                sensors.setLeftMotor(self.int_st_speed)
                sensors.setRightMotor(self.int_st_speed)
                self.int_start_pos = Robot.odometry.getFieldToVehicle()
                self.int_state += 1
            elif(self.int_state == 1):
                # distance between current pos and start pos
                if(Robot.odometry.getFieldToVehicle().inverse().transformBy(self.int_start_pos).getTranslation().norm() >= self.intersection_forward_dist):
                    # the current position should be approximately the center of the intersection
                    sensors.setMotorOff()
                    self.int_state += 1
                    # Log current pos as an intersection point
                    map.logIntersection(map.Intersection(Robot.odometry.getFieldToVehicle()))
                    # determine what the best direction to go is
                    # ensure you also re-look at possible directions
                    self.int_turn_direction = map.Turn_Direction_Robot_t.BCK
                    if(self.getRightAvailable()):
                        self.int_turn_direction = map.Turn_Direction_Robot_t.RHT
                        sensors.setLeftMotor(self.int_turn_speed)
                        sensors.setRightMotor(-self.int_turn_speed)
                    elif(self.getFrontAvailable()):
                        self.int_turn_direction = map.Turn_Direction_Robot_t.FWD
                    elif(self.getLeftAvailable()):
                        self.int_turn_direction = map.Turn_Direction_Robot_t.LFT
                        sensors.setLeftMotor(-self.int_turn_speed)
                        sensors.setRightMotor(self.int_turn_speed)

                    # If necessary, turn, if not, skip over
                    if(self.int_turn_direction == map.Turn_Direction_Robot_t.FWD):
                        self.int_state += 2 # skip turning
                    else:
                        self.int_start_heading = Robot.odometry.getFieldToVehicle().getRotation().getDegrees()
                        self.int_state += 1

            elif(self.int_state == 2):
                if(math.fabs(self.int_start_heading - Robot.odometry.getFieldToVehicle().getRotation().getDegrees()) >= self.int_turn_angle ):
                    # turning complete
                    sensors.setMotorOff()
                    self.int_state += 1

            elif(self.int_state == 3):
                # drive forward to exit the intersection
                sensors.setLeftMotor(self.int_st_speed)
                sensors.setRightMotor(self.int_st_speed)
                self.int_start_pos = Robot.odometry.getFieldToVehicle()
                self.int_state += 1

            elif(self.int_state == 4):
                # distance between current pos and start pos
                if(Robot.odometry.getFieldToVehicle().inverse().transformBy(self.int_start_pos).getTranslation().norm() >= self.int_exit_distance):
                    sensors.setMotorOff()
                    # check to ensure we are still in a maze
                    if(self.getLeftAvailable() or self.getRightAvailable()):
                        # we have exited the maze lol yeet
                        self.state = self.Drive_State_t.DELIVERING
                    else:
                        # Keep going, you can do it little buddy!
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
    