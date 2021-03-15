import matplotlib.pyplot as plt
import math
import csv
import numpy as np
from numpy.polynomial.polynomial import polyfit

N = 30 # number of times to find lines
S = 5 # number of samples to compute line
D = 10 # degree range to use for finding samples S
X = 4 # max distance reading can be away from line to be associated with line
C = 30 # max number of points that must lie on line to count as line

#def lineFinder(coords):

extraction_interval = 15 #extract data 25 degrees on left and right
# left: 180 +- interval, right: 0 +- interval
# assumes data is in degrees
def extract_left_right(polar_data):
    left_polar = []
    right_polar = []
    for polar in polar_data:
        if(polar[0]  < extraction_interval + 180 and polar[0] > 180 - extraction_interval):
            left_polar.append(polar)
        elif(polar[0] < extraction_interval and polar[0] > -extraction_interval):
            right_polar.append(polar)

    return [left_polar, right_polar]

# assumes data in degrees
def convert_to_cartesian(polar_data):
    cartesian_coords = []
    for polar in polar_data:
        theta = polar[0] * 3.14159 / 180
        r = polar[1]
        cartesian_coords.append([r*math.cos(theta), r*math.sin(theta)])

    return cartesian_coords

# return best fit line using equation x = my +b
def best_fit_line(cartesian_coords):
    x, y = zip(*cartesian_coords)
    b, m = polyfit(y, x, 1 )
    b_n = float(b)
    m_n = float(m)
    return b_n, m_n

# plots in form x = ym + b
y_range = 20
def plot_line(b, m):
    x = []
    y = []
    for i in range(-y_range, y_range):
        y.append(i)
        x.append(b + m * i)

    plt.plot(x, y)
        
    
    

polar_coords = []

file_name = 'scan6.csv'
fid = open(file_name, 'r')
csvFile = csv.reader(fid)
for line in csvFile:
    polar_coords.append([-float(line[0])+90, float(line[1])])

fid.close()

cartes_coords = convert_to_cartesian(polar_coords)

left_polar, right_polar = extract_left_right(polar_coords)
left_cart = convert_to_cartesian(left_polar)
right_cart = convert_to_cartesian(right_polar)


left_b, left_m = best_fit_line(left_cart)
right_b, right_m = best_fit_line(right_cart)

center_b = (left_b + right_b) / 2
center_m = (left_m + right_m) / 2


plt.scatter(*zip(*cartes_coords))
plt.scatter(0,0)
plt.scatter(*zip(*left_cart))
plt.scatter(*zip(*right_cart))

plot_line(left_b, left_m)
plot_line(right_b, right_m)
plot_line(center_b, center_m)
#plt.plot(right_b + right_m * right_cart[1], right_cart[1])

plt.grid(True)
plt.show()

