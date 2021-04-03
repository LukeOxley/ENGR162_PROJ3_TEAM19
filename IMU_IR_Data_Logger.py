#Used to collect raw data from know distances and output them to a CSV for future analysis
#Does this for both IR Sensors and the IMU, just call the correct function to do with distances
#As either a magnitude or in component form

import sensors

ir_spacing = 9.5 # cm

sensors.initSensors()

def baseOnDistanceIR(): #Magnitude
    #initialization
    distLeft = 0
    distRight = 0
    leftDists = []
    rightDists = []
    leftData = []
    rightData = []
    entries = 0

    #Collects Data
    while(distLeft != -1):
        distLeft = float(input("please enter the distance away from the hazard the left IR sensor is or enter -1 to end: "))
        if(distLeft != -1):
            distRight = float(input("please enter the distance away from the hazard the Right IR sensor is: "))
            leftDists.append(distLeft)
            rightDists.append(distRight)
            sensors.updateSensors()
            leftData.append(sensors.getIRLevelLeft())
            rightData.append(sensors.getIRLevelRight())
            entries = entries + 1

    #output to CSV
    data = open("IR_Distance_Magnitude_Data.csv", "w")
    data.write("Left IR Distance,")
    data.write("Left IR Reading,")
    data.write("Right IR Distance,")
    data.write("Right IR Reading")

    i = 0
    while (i < entries):
        data.write("%.1f cm," %leftDists[i])
        data.write("%.3f ," %leftData[i])
        data.write("%.1f cm," %rightDists[i])
        data.write("%.3f \n" %rightData[i])
        i = i + 1

def baseOnDistanceComponentsIR(): #Components
    #initialization
    distLeftX = 0
    distLeftY = 0
    distRightX = 0
    distRightY = 0
    leftDistsX = []
    leftDistsY = []
    rightDistsX = []
    rightDistsY = []
    leftData = []
    rightData = []
    entries = 0

    #Collects Data
    while(distLeftX != -1):
        distLeftX = float(input("please enter the X (horizontal, robot left = +) component distance away from the hazard the left IR sensor is or enter -1 to end: "))
        if(distLeftX != -1):
            distLeftY = float(input("please enter the Y component distance away from the hazard the left IR sensor is or enter -1 to end: "))
            distRightX = distLeftX - ir_spacing # input("please enter the X component distance away from the hazard the Right IR sensor is: ")
            distRightY = distLeftY # input("please enter the Y component distance away from the hazard the Right IR sensor is: ")
            leftDistsX.append(distLeftX)
            leftDistsY.append(distLeftY)
            rightDistsX.append(distRightX)
            rightDistsY.append(distRightY)
            sensors.updateSensors()
            leftData.append(sensors.getIRLevelLeft())
            rightData.append(sensors.getIRLevelRight())
            entries = entries + 1

    #output to CSV
    data = open("IR_Distance_Component_Data.csv", "w")
    data.write("Left IR Distance (X),")
    data.write("Left IR Distance (Y),")
    data.write("Left IR Reading,")
    data.write("Right IR Distance (X),")
    data.write("Right IR Distance (Y),")
    data.write("Right IR Reading\n")

    i = 0
    while (i < entries):
        data.write("%.1f cm," %leftDistsX[i])
        data.write("%.1f cm," %leftDistsY[i])
        data.write("%.3f ," %leftData[i])
        data.write("%.1f cm," %rightDistsX[i])
        data.write("%.1f cm," %rightDistsY[i])
        data.write("%.3f \n" %rightData[i])
        i = i + 1

def baseOnDistanceIMU(): #Magnitude
    #initialization
    distMag = 0
    magDataX = [] #x component reading
    magDataY = [] #y component reaing
    magDataMag = [] #magnitude reading
    dists = []
    entries = 0

    #Collects Data
    while(distMag != -1):
        distMag = float(input("please enter the distance away from the hazard the IMU is or enter -1 to end: "))
        if(distMag != -1):
            dists.append(distMag)
            sensors.updateSensors()
            x, y, z = sensors.getMagneticLevel() #x, y, amd z component magnetic Readings
            magDataX.append(x)
            magDataY.append(y)
            entries = entries + 1

    #output to CSV
    data = open("IMU_Distance_Magnitude_Data.csv", "w")
    data.write("IMU distance,")
    data.write("x comp Mag reading,")
    data.write("y comp Mag reading,")
    data.write("Magnitude Mag reading\n")

    i = 0
    while (i < entries):
        data.write("%.1f cm," %dists[i])
        data.write("%.3f ," %magDataX[i])
        data.write("%.3f ," %magDataY[i])
        data.write("%.3f \n" %magDataMag[i])
        i = i + 1

def baseOnDistanceComponentsIMU(): #Magnitude
    #initialization
    distX = 0
    distY = 0
    magDataX = [] #x component reading
    magDataY = [] #y component reaing
    magDataZ = [] #z component reaing
    magDataMag = [] #magnitude reading
    distsX = []
    distsY = []
    entries = 0

    #collects data
    while(distX != -1):
        distX = float(input("please enter the X (horizontal, robot left = +) component distance away from the hazard the IMU is or enter -1 to end: "))
        if(distX != -1):
            distY = float(input("please enter the Y component distance away from the hazard the IMU is or enter -1 to end: "))
            distsX.append(distX)
            distsY.append(distY)
            sensors.updateSensors()
            x, y, z = sensors.getMagneticLevel() #x, y, amd z component magnetic Readings
            magDataX.append(x)
            magDataY.append(y)
            magDataZ.append(z)
            magDataMag.append(sensors.getMagneticMagnitude())
            entries = entries + 1

    #outputs to CSV
    data = open("IMU_Distance_Component_Data.csv", "w")
    data.write("IMU distance (X),")
    data.write("IMU distance (Y),")
    data.write("x comp Mag reading,")
    data.write("y comp Mag reading,")
    data.write("z comp Mag reading,")
    data.write("Magnitude Mag reading \n")

    i = 0
    while (i < entries):
        data.write("%.1f cm," %distsX[i])
        data.write("%.1f cm," %distsY[i])
        data.write("%.3f ," %magDataX[i])
        data.write("%.3f ," %magDataY[i])
        data.write("%.3f ," %magDataZ[i])
        data.write("%.3f \n" %magDataMag[i])
        i = i + 1

#Call any of the 4
#baseOnDistanceIR()
#baseOnDistanceComponentsIR()
#baseOnDistanceIMU()
#baseOnDistanceComponentsIMU()
