import numpy as np
import csv
import math
import os


# definition
f = 100 #frequency
r = 20 #radius in m
v = 0.2513 #speed
xoffset = 25
yoffset = 25
#calculation
time = 2 * math.pi * r/v
print(time)
rad = (math.pi/180)*360/(time*f) # rad per step
steps = int(time*f)
timestep = 1/f

t = [0]*steps
x = [0]*steps
y = [0]*steps

#calculation of position
for i in range(steps):
    t[i] = i*timestep
    x[i] = math.sin(rad*i)*r + xoffset
    y[i] = math.cos(rad*i)*r + yoffset

tmp = os.path.dirname(__file__)

with open((os.path.join(tmp, "test.csv")), mode="a", newline="") as f:
            writer = csv.writer(f, delimiter = ",")
            for i in range(steps):
                writer.writerow([t[i], x[i], y[i], -1])