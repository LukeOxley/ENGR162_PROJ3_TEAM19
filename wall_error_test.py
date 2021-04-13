from odometry import Odometry
from transforms import *
import sensors
import time
import math

sensors.initSensors()

while(True):
    sensors.updateSensors()

    # Clear screen to keep positions constant
    print(chr(27)+'[2j')
    print('\033c')
    print('\x1bc')

    wall_width = 40
    robot_width = 22


    l_prime = 1 + sensors.getLeftWallDistance()
    r_prime = 1 + sensors.getRightWallDistance()
    print(l_prime - r_prime)
    temp = (wall_width-robot_width)/(l_prime+r_prime)
    print(temp)

    print("L Prime: {:.2f}".format(l_prime))
    print("R Prime: {:.2f}".format(r_prime))
    if(math.fabs(temp) <= 1):
        theta = math.acos((wall_width-robot_width)/(l_prime+r_prime))
        l = l_prime * math.cos(theta)  
        r = r_prime * math.cos(theta)

        print("Theta: {:.2f}".format(theta*180/3.14159))
        print("L: {:.2f}".format(l))
        print("R: {:.2f}".format(r))

    time.sleep(0.25)
