import time
import threading

class Robot():
    cycle_time = 0.5#0.02
    enabled = False

    def __init__(self):
        #idk
        pass

    def startLoop(self):
        if(not self.enabled):
            print("Starting Main Robot Loop")
            self.enabled = True
            self.main_thread = threading.Thread(target = self.loop)
            self.main_thread.start()
    
    def stopLoop(self):
        if(self.enabled):
            print("Stopping Main Robot Loop")
            self.enabled = False

    def loop(self):

        while(self.enabled):
            print("robot main")
            time.sleep(self.cycle_time)
