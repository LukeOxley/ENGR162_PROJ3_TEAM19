
from tkinter import *
from tkinter import ttk
from tkinter import font as tkFont
from tkinter import scrolledtext
import robot
from transforms import *
import map

class GUI(Frame): # Extending Frame

    #Grid params
    wall_width_cm = 40
    num_cols = 5
    canv_width_px = 400
    grid_width_px = 300
    behind_distance = wall_width_cm / 2
    coord_to_grid_origin = Translation2d(0,0)

    def __init__(self):
        # calling Frame's init
        super().__init__() 
        self.robot = robot.Robot(self)
        self.master.title("GEARS")
        self.createButtons()
        self.initUI()
        self.initSensorPannel()
        self.log_message("GEARS Console")
        self.log_message("Press 'Start Run' to enable!")

    def startButtonAction(self,event):
        self.makeGrid()
        map.reset()
        map.setTranslation(self.wall_width_cm, self.behind_distance, \
                            float(self.x_start_spin.get()), \
                            float(self.y_start_spin.get()))
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

        chk_frame = Frame(control_frame)
        self.chk_state_intersection_enabled = BooleanVar()
        self.chk_state_intersection_enabled .set(True)
        self.chk_int = Checkbutton(chk_frame, text='Enable Intersection', var=self.chk_state_intersection_enabled )
        self.chk_int.grid(row=0, column=0)
        self.chk_state_entering_enabled = BooleanVar()
        self.chk_state_entering_enabled .set(True)
        self.chk_ent = Checkbutton(chk_frame, text='Enable Entering', var=self.chk_state_entering_enabled )
        self.chk_ent.grid(row=0, column=1)
        chk_frame.grid(row=2, column=0)

        self.pos_lbl = Label(control_frame, text='Current Pose: ')
        self.pos_lbl.grid(row=3, column = 0)

        spin_frame = Frame(control_frame)
        Label(spin_frame, text='Start Position').grid(row=1, column=0)
        Label(spin_frame, text='X:').grid(row=1, column=1)
        self.x_start_spin = Spinbox(spin_frame, from_=0, to=10, width=5)
        self.x_start_spin.grid(row=1, column=2)
        Label(spin_frame, text='Y:').grid(row=1, column=3)
        self.y_start_spin = Spinbox(spin_frame, from_=0, to=10, width=5)
        self.y_start_spin.grid(row=1, column=4)
        Label(spin_frame, text='Num Cols').grid(row=2, column=0)
        col_var = IntVar()
        col_var.set(self.num_cols)
        self.num_cols_spin = Spinbox(spin_frame, from_=0, to=10, width=5, textvariable=col_var)
        self.num_cols_spin.grid(row=2, column=1)
        spin_frame.grid(row=4, column=0)

        Label(spin_frame, text='Behind Start Dist').grid(row=2, column=2)
        behind_var = IntVar()
        behind_var.set(int(self.behind_distance))
        self.behind_spin = Spinbox(spin_frame, from_=0, to=self.wall_width_cm, width=5, textvariable=behind_var)
        self.behind_spin.grid(row=2, column=3)
        spin_frame.grid(row=4, column=0)

        sub_buttom_frame = Frame(control_frame)
        self.refresh = Button(sub_buttom_frame,text='Refresh Grid',width=10)
        self.refresh.grid(row=0, column=0)
        self.refresh.bind('<Button-1>', self.makeGridButtonEvent)
        Label(sub_buttom_frame, text='Map').grid(row=0, column=1)
        self.map_num_spin = Spinbox(sub_buttom_frame, from_=0, to=50, width=5)
        self.map_num_spin.grid(row=0, column=2)
        self.export_grid = Button(sub_buttom_frame,text='Export Grid',width=10)
        self.export_grid.grid(row=0, column=3)
        self.export_grid.bind('<Button-1>', self.exportGridEvent)
        sub_buttom_frame.grid(row=5, column=0)

        self.txt_box = scrolledtext.ScrolledText(control_frame, width=40, height=10)
        self.txt_box.grid(row=6, column=0)

        control_frame.grid(row=0, column=0)

    def initUI(self):

        #self.pack(fill=BOTH, expand=1)
        self.canvas = Canvas(self.master, bg="white", height=self.canv_width_px, width=self.canv_width_px)
        self.makeGrid()
        self.canvas.grid(row=0, column=1)

    def initSensorPannel(self):
        sensor_frame = Frame(self.master)

        Label(sensor_frame, text='---Sonic Readings---').grid(row=0, column=0)
        self.front_pos_lbl = Label(sensor_frame, text='FD: ') 
        self.front_pos_lbl.grid(row=1, column=0)
        side_frame = Frame(sensor_frame)
        self.left_pos_lbl = Label(side_frame, text='LD: ') 
        self.left_pos_lbl.grid(row=0, column=0)
        self.right_pos_lbl = Label(side_frame, text='RD: ') 
        self.right_pos_lbl.grid(row=0, column=1)
        side_frame.grid(row=2, column=0)

        Label(sensor_frame, text='Magnetometer Reading').grid(row=3, column=0)
        self.mag_lbl = Label(sensor_frame, text='Mag:')
        self.mag_lbl.grid(row=4, column=0)

        Label(sensor_frame, text='----IR Readings----').grid(row=5, column=0)
        ir_side_frame = Frame(sensor_frame)
        self.ir_left_lbl = Label(ir_side_frame, text='Left:')
        self.ir_left_lbl.grid(row=1, column=0)
        self.ir_right_lbl = Label(ir_side_frame, text='Right:')
        self.ir_right_lbl.grid(row=1, column=1)
        ir_side_frame.grid(row=6, column=0)

        Label(sensor_frame, text='-----Drive State-----').grid(row=7, column=0)
        self.drive_lbl = Label(sensor_frame, text='Drive State:')
        self.drive_lbl.grid(row=8, column=0)

        sensor_frame.grid(row=0, column=2)

    def makeGridButtonEvent(self, event):
        self.makeGrid()
    
    def makeGrid(self):
        self.canvas.delete("all")
        self.num_cols = int(self.num_cols_spin.get())
        wall_width_px = self.cmToPixels(self.wall_width_cm)
        self.behind_distance = int(self.behind_spin.get())
        behind_distance_px = self.cmToPixels(self.behind_distance)
        b = int((self.canv_width_px - self.grid_width_px) / 2)
        p = wall_width_px / 2
        self.coord_to_grid_origin.setX(b + p + float(self.x_start_spin.get()) * wall_width_px)
        self.coord_to_grid_origin.setY(b + float(self.y_start_spin.get()) * wall_width_px - behind_distance_px)

        spacing = int(wall_width_px)

        for i in range(b, self.grid_width_px + spacing, spacing):
            self.canvas.create_line(i,b,i,self.canv_width_px - b)
            self.canvas.create_line(b,i,self.canv_width_px - b,i)
    
    def exportGridEvent(self, event):
        # width, behind, startX, startY
        map_num = float(self.map_num_spin.get())
        map.exportGrid(map_number=map_num, notes="Map data recorded by GEARS")
        map.exportHazards(map_number=map_num, notes="Hazard data recorded by GEARS")
        for row in map.grid:
            print(row)
    
    def log_message(self, message):
        self.txt_box.insert(INSERT, message + '\n')
        self.txt_box.see(INSERT)
    
    def log_sonics(self, left, front, right):
        self.front_pos_lbl.configure(text="FD: {:.2f}".format(front))
        self.left_pos_lbl.configure(text="LD: {:.2f}".format(left))
        self.right_pos_lbl.configure(text="RD: {:.2f}".format(right))
    
    def log_mag(self, mag):
        self.mag_lbl.configure(text="Mag: {:.2f}".format(mag))

    def log_ir(self, left, right):
        self.ir_left_lbl.configure(text="Left: {:.2f}".format(left))
        self.ir_right_lbl.configure(text="Right: {:.2f}".format(right))

    def log_state(self, drive_state):
        self.drive_lbl.configure(text="Drive State: {:s}".format(drive_state))

    
    def plot_location(self, translation):
        translation_px = Translation2d(self.cmToPixels(translation.getX()), 
                                       self.cmToPixels(translation.getY()))
        translation_px = translation_px.translateBy(self.coord_to_grid_origin)
        x = translation_px.getX()
        y = self.canv_width_px - translation_px.getY()
        oS = 2
        self.canvas.create_oval(x - oS, y - oS, x + oS, y + oS, fill='blue')

    def log_pos(self, rigid_transform):
        self.plot_location(rigid_transform.getTranslation())
        self.pos_lbl.configure(text="Current Pose: "+str(rigid_transform))
    
    def getIntersectionEnabled(self):
        return self.chk_state_intersection_enabled.get()
    
    def getEnteringEnabled(self):
        return self.chk_state_entering_enabled.get()
    
    def cmToPixels(self, l):
        return l * self.grid_width_px / (self.wall_width_cm  * self.num_cols)

def main():
    master = Tk()
    master.geometry("1000x410")
    gui = GUI()
    master.mainloop()

if __name__ == '__main__':
    main()
