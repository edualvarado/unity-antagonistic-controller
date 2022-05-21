import matplotlib.pyplot as plt
import time
import numpy as np
from scipy.interpolate import make_interp_spline, BSpline
from matplotlib import rc

idx = []
rHandX1Max, rHandY1Max, rHandZ1Max = [], [], []
rForeArmX1Max, rForeArmY1Max, rForeArmZ1Max = [], [], []
rArmX1Max, rArmY1Max, rArmZ1Max = [], [], []

rHandX1Min, rHandY1Min, rHandZ1Min = [], [], []
rForeArmX1Min, rForeArmY1Min, rForeArmZ1Min = [], [], []
rArmX1Min, rArmY1Min, rArmZ1Min = [], [], []

#minLimitX = -2
#maxLimitX = 2
#minLimitY = -2
#maxLimitY = 2

with open("torques-1-max.txt") as f:
    for index, line in enumerate(f):
        values = [float(s) for s in line.split(",")]
        idx.append(index)
        rHandX1Max.append(values[0])
        rHandY1Max.append(values[1])
        rHandZ1Max.append(values[2])
        rForeArmX1Max.append(values[3])
        rForeArmY1Max.append(values[4])
        rForeArmZ1Max.append(values[5])
        rArmX1Max.append(values[6])
        rArmY1Max.append(values[7])
        rArmZ1Max.append(values[8])

with open("torques-1-min.txt") as f:
    for index, line in enumerate(f):
        values = [float(s) for s in line.split(",")]
        idx.append(index)
        rHandX1Min.append(values[0])
        rHandY1Min.append(values[1])
        rHandZ1Min.append(values[2])
        rForeArmX1Min.append(values[3])
        rForeArmY1Min.append(values[4])
        rForeArmZ1Min.append(values[5])
        rArmX1Min.append(values[6])
        rArmY1Min.append(values[7])
        rArmZ1Min.append(values[8])
###

# 1. Plot Total Forces

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
n = 200

plt.scatter(rForeArmX1Max, rForeArmY1Max, rForeArmZ1Max, c='red', marker='+')
plt.scatter(rForeArmX1Min, rForeArmY1Min, rForeArmZ1Min, c='blue', marker='.')

#

plt.xlabel('Xlabel')
plt.ylabel('Ylabel')

plt.title('Stiffness Variation during Interactions')
plt.legend(loc = "upper left")

#plt.xlim([minLimitX, maxLimitX])
#plt.ylim([minLimitY, maxLimitY])

plt.grid()

###

plt.show()