#!/usr/bin/env python

"""
Small Code to simulate created data to check creationcode
"""
import matplotlib.pyplot as plt
import numpy as np
from config_class import config
from tqdm import tqdm




v = config.v
vn = config.vn
t_lin = config.t_lin


x0 = np.array([0,0])
xn0 = np.array([0,0])
v0 = np.array([0,0])
vn0 = np.array([0,0])
tn0 = 0
t0 = 0
fac = config.f_ac
fimu = config.f_pre
n = fac/fimu
xreal = []
xnoise = []
yreal = []
ynoise = []
treal = []
tnoise = []
# xReal
for i in tqdm(range(len(t_lin)), ncols = 100):
    t = t_lin[i]
    dt = t-t0
    vreal = np.array(v[i])
    x = x0 + vreal*dt 
    
    xreal.append(x[0])
    yreal.append(x[1])
    treal.append(t)

    x0 = x
    t0 = t

# xNoise
for i in tqdm(range(len(t_lin)), ncols= 100):
    if i%n == 0:
            t = t_lin[i]
            dt = t-tn0
            vnoise = np.array(vn[i])
            x = xn0 + vnoise*dt
            xnoise.append(x[0])
            ynoise.append(x[1])
            tnoise.append(t)
            xn0 = x
            tn0 = t



plt.scatter(xreal, yreal, color = "b")
plt.scatter(xnoise, ynoise, color = "g")

plt.show()