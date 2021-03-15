
from tkinter import *
from tkinter import ttk
from tkinter import font as tkFont
from robot import Robot
import sensors

class GUI(Frame): # Extending Frame

    def __init__(self):
        # calling Frame's init
        super().__init__() 
        self.master.title("GEARS")
        self.initUI()
        self.createButtons()

        self.robot = Robot()

    def startButtonAction(self,event):
        self.robot.startLoop()
        self.startButton["state"] = "disabled"
        self.stopButton["state"] = "normal"
    
    def stopButtonAction(self,event):
        self.robot.stopLoop()
        self.startButton["state"] = "normal"
        self.stopButton["state"] = "disabled"
        
    def createButtons(self):
        control_frame = Frame(self.master)

        start_font = tkFont.Font(family='Helvetica', size=15, weight='bold')
        self.startButton = Button(control_frame,text='Start Run',width=15, fg='green',font=start_font)
        self.startButton.grid(row=0, column=0)
        self.startButton.bind('<Button-1>', self.startButtonAction)

        stop_font = tkFont.Font(family='Helvetica', size=15, weight='bold')
        self.stopButton = Button(control_frame,text='Stop Run',width=15, fg='red', font=stop_font)
        self.stopButton.grid(row=1, column=0)
        self.stopButton.bind('<Button-1>', self.stopButtonAction)
        self.stopButton["state"] = "disabled"

        control_frame.grid(row=0, column=0)

    def initUI(self):

        #self.pack(fill=BOTH, expand=1)

        canvas = Canvas(self.master, bg="white", height=200, width=200)
        
        self.line = canvas.create_line(0, 0, 200, 25)
        canvas.grid(row=0, column=1)
        #canvas.pack(fill=BOTH, expand=1)

def main():
    master = Tk()
    master.geometry("400x200")
    gui = GUI()
    master.mainloop()

if __name__ == '__main__':
    main()