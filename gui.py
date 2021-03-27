
from tkinter import *
from tkinter import ttk
from tkinter import font as tkFont
from tkinter import scrolledtext
import robot
from transforms import *

class GUI(Frame): # Extending Frame

    def __init__(self):
        # calling Frame's init
        super().__init__() 
        self.robot = robot.Robot(self)
        self.master.title("GEARS")
        self.initUI()
        self.createButtons()
        self.log_message("GEARS Console")
        self.log_message("Press 'Start Run' to enable!")

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

        self.chk_state = BooleanVar()
        self.chk_state.set(True)
        self.chk = Checkbutton(control_frame, text='Enable Intersection', var=self.chk_state)
        self.chk.grid(row=2, column=0)

        self.pos_lbl = Label(control_frame, text='Pose: ')
        self.pos_lbl.grid(row=3, column = 0)

        self.txt_box = scrolledtext.ScrolledText(control_frame, width=40, height=10)
        self.txt_box.grid(row=4, column=0)

        control_frame.grid(row=0, column=0)

    def initUI(self):

        #self.pack(fill=BOTH, expand=1)
        self.canvas_width = 400

        self.canvas = Canvas(self.master, bg="white", height=self.canvas_width, width=self.canvas_width)
        wall_width = 40
        self.real_width = 200 
        box_count = int(self.real_width / wall_width )
        spacing = int(self.canvas_width / box_count)

        for i in range(0,self.canvas_width,spacing):
            self.canvas.create_line(i,0,i,self.canvas_width)
            self.canvas.create_line(0,i,self.canvas_width,i)
            


        self.line = self.canvas.create_line(0, 0, 200, 25)
        self.canvas.grid(row=0, column=1)
        #canvas.pack(fill=BOTH, expand=1)
    
    def log_message(self, message):
        self.txt_box.insert(INSERT, message + '\n')
        self.txt_box.see(INSERT)
    
    def plot_location(self, translation):
        x = translation.getX()
        y = translation.getY()
        x = x * (self.canvas_width / self.real_width)
        y = self.canvas_width - y * (self.canvas_width / self.real_width)
        self.canvas.create_oval(x - 1, y - 1, x + 1, y + 1, fill='blue')

    def log_pos(self, rigid_transform):
        self.plot_location(rigid_transform.getTranslation())
        self.pos_lbl.configure(text="Pose: "+str(rigid_transform))

    

        


def main():
    master = Tk()
    master.geometry("750x410")
    gui = GUI()
    master.mainloop()

if __name__ == '__main__':
    main()