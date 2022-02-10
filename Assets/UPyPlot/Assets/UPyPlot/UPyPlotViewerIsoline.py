import matplotlib.pyplot as plt
import time
import numpy as np
from scipy.interpolate import make_interp_spline, BSpline

idx = []
slopeX = []
x, y = [],[]

minLimitX = 0
maxLimitX = 1
minLimitY = 0
maxLimitY = 1

with open("..\..\plotting_cache\\plot-gains.txt") as f:
    for index, line in enumerate(f):
        values = [float(s) for s in line.split(",")]
        idx.append(index)
        slopeX.append(values[0])
        x.append(index)
        y.append(values[0] * index)

# Create just a figure and only one subplot
fig, ax = plt.subplots()
fig.tight_layout()

###

ax.plot(x, y, label='line', color="midnightblue")

ax.set_ylabel('Y')
ax.set_xlabel('X')

ax.set_xlim([minLimitX, maxLimitX])
ax.set_ylim([minLimitY, maxLimitY])
ax.grid()

###

plt.show()