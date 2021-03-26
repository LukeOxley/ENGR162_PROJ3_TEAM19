from transforms import RigidTransform2d, Rotation2d, Translation2d
import map
import robot
import sensors
import math

# updates robot position on the coordinate plane
# periodically ran
# coordinate axes: forward positive x, right +/- y (idk, yet)
# theta = 0 means forward

class Odometry:
    def __init__(self):
        self.current_pos = RigidTransform2d(Translation2d(0,0), Rotation2d(0,0,False))
        self.last_left_encoder_reading = 0
        self.last_right_encoder_reading = 0
        self.dt = robot.Robot.cycle_time 

    def reset(self):
        self.current_pos = RigidTransform2d(Translation2d(0,0), Rotation2d(0,0,False))


    def updateOdometry(self):

        left_distance = sensors.getLeftWheelDistance()
        right_distance = sensors.getRightWheelDistance()

        delta_left = left_distance - self.last_left_encoder_reading
        delta_right = right_distance - self.last_right_encoder_reading
        theta = Rotation2d.fromDegrees(sensors.getHeading)

        self.last_left_encoder_reading = left_distance
        self.last_right_encoder_reading = right_distance

        self.current_pos = self.integrateForwardKinematics(self.current_pos, delta_left, delta_right, theta)


    def forwardKinematics(left, right, rads):
        return RigidTransform2d.Delta((left + right)/2, 0, rads)

    def integrateForwardKinematics(self, current_pose, left, right, heading):
        with_gyro = self.forwardKinematics(left, right, 
                                    current_pose.getRotation().inverse().rotateBy(heading).getRadians())
        return current_pose.transformBy(RigidTransform2d.fromVelocity(with_gyro))

    def getFieldToVehicle(self):
        return self.current_pos

    def getCurrentDirection(self):
            heading = self.current_pos.getRotation().getDegrees()
            if(heading <= 45 and heading >= -45):
                return map.Turn_Direction_t.NORTH
            elif(heading > 45 and heading <= 135):
                return map.Turn_Direction_t.EAST
            elif(heading < -45 and heading >= -135):
                return map.Turn_Direction_t.WEST
            else:
                return map.Turn_Direction_t.SOUTH
