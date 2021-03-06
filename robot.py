import time
import threading
from tkinter import *
from transforms import RigidTransform2d, Rotation2d, Translation2d
import gui
from odometry import Odometry
from drive import Drive
import sensors
import map

# List of things TODO:
# zero position once it enters the maze?
# delivery sequence?
# test sensors are outputting correctly
# implement hazard avoidance
# implement hazard tracking

# combines everything together except for the gui
# in charge of the periodic updating
# loop start and stop controlled by gui
class Robot(object):
    cycle_time = 0.01
    enabled = False
    odometry = Odometry(cycle_time)

    def __init__(self, gui):
        self.gui = gui
        self.drive = Drive(self, gui)
        

    def initialize(self):
        sensors.initSensors()

    def startLoop(self):
        if(not self.enabled):
            self.gui.log_message("Starting Main Robot Loop")
            self.enabled = True
            self.initialize()
            self.drive.initDrive()
            self.odometry.reset()
            self.drive.setEnableIntersection(self.gui.getIntersectionEnabled())
            self.drive.setEnableEntering(self.gui.getEnteringEnabled())
            self.main_thread = threading.Thread(target = self.loop)
            self.main_thread.start()

    def stopLoop(self):
        if(self.enabled):
            self.gui.log_message("Stopping Main Robot Loop")
            self.enabled = False

    def loop(self):
        loop_counter = 0
        self.current = RigidTransform2d(Translation2d(0, 0), Rotation2d.fromDegrees(0))

        while(self.enabled):
            loop_counter += 1

            #self.gui.log_message("robot main")
            
            # Able to decrease sensor update frequency
            if(loop_counter % 1 == 0):
                sensors.updateSensors()

            self.odometry.updateOdometry()
            self.drive.updateDrive()

            if(loop_counter % 1 == 0):
                self.current = self.odometry.getFieldToVehicle()
                
                self.gui.log_pos(self.current)
                self.gui.log_sonics(sensors.getLeftWallDistance(), sensors.getForwardWallDistance(), sensors.getRightWallDistance())
                self.gui.log_mag(sensors.getMagneticMagnitude())
                self.gui.log_ir(0.0, 0.0)
                self.gui.log_state(self.drive.state)


            if(loop_counter >= 1000):
                loop_counter = 0
            
            time.sleep(self.cycle_time)
        
        sensors.shutdown()
