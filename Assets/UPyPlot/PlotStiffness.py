import matplotlib.pyplot as plt
import time
import numpy as np
from scipy.interpolate import make_interp_spline, BSpline
from matplotlib import rc

idx = []
stiffnessLeft, stiffnessRight = [],[]
stiffnessNeutral = []

minLimitX = 0
maxLimitX = 12.5
minLimitY = 0
maxLimitY = 1.1

with open("plot-stiffness-faster-01.txt") as f:
    for index, line in enumerate(f):
        values = [float(s) for s in line.split(",")]
        idx.append(index/10)
        stiffnessLeft.append(values[0])
        stiffnessRight.append(values[1])
        stiffnessNeutral.append(0.5)

###

# 1. Plot Total Forces
plt.plot(idx, stiffnessLeft, '-', label='Left Arm', color="blue")
plt.plot(idx, stiffnessRight, '-', label='Right Arm', color="red")
plt.plot(idx, stiffnessNeutral, '--', label='Default', color="grey")

#x_sm = np.array(idx)
#y_sm = np.array(stiffnessLeft)

#X_Y_Spline = make_interp_spline(x_sm, y_sm)

#X_ = np.linspace(x_sm.min(), x_sm.max(), 1000)
#Y_ = X_Y_Spline(X_)

#plt.plot(X_, Y_, color="red")

#

plt.ylabel('Stiffness Multiplier')
plt.gca().set_ylabel(r'Stiffness $(k_{L} - k_{Lmin})/(k_{Lmax} - k_{Lmin}) $')


plt.xlabel('Time [s]')

plt.title('Stiffness Variation during Interactions')
plt.legend(loc = "upper left")

plt.xlim([minLimitX, maxLimitX])
plt.ylim([minLimitY, maxLimitY])

plt.grid()

###

plt.show()