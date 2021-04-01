import time
import threading
from tkinter import *
from transforms import RigidTransform2d, Rotation2d, Translation2d
import gui
#from odometry import *
#from drive import *
#import sensors

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
    cycle_time = 0.02
    enabled = False
    #odometry = Odometry()
    #drive = Drive()

    def __init__(self, gui):
        self.gui = gui
        

    def initialize(self):
        #sensors.initSensors()
        pass

    def startLoop(self):
        if(not self.enabled):
            self.gui.log_message("Starting Main Robot Loop")
            self.enabled = True
            self.initialize()
            #self.drive.setEnableIntersection(self.gui.getIntersectionEnabled())
            #self.drive.setEnableEntering(self.gui.getEnteringEnabled())
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
                #sensors.updateSensors()
                pass

            #self.drive.updateDrive()
            #self.odometry.updateOdometry()
            self.current = self.current.transformBy(RigidTransform2d(Translation2d(1,1),Rotation2d.fromDegrees(0.05)))

            if(loop_counter % 1 == 0):
                #location = self.odometry.getFieldToVehicle()
                
                self.gui.log_pos(self.current)
                self.gui.log_sonics(10, 40, 10)
                self.gui.log_mag(0.0)
                self.gui.log_ir(0.0, 0.0)


            if(loop_counter >= 1000):
                loop_counter = 0
            
            time.sleep(self.cycle_time)
