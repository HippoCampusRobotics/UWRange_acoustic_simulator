#!/usr/bin/env python

import numpy as np
import csv
from tqdm import tqdm
from config_class import config

selection = True # int(input("Pls Select a Model: 1 Kinematic, 2 static, 3 circle: "))

if selection == False: #linear
    """ Linear - daten 
    kinematic model: 
        x = x0 + v0 * dt + 0.5*(dt**2)*a
        x0 = x
        v0 = v0 + a*dt
    """

    f = config.f_ac # Hz
    dt = 1/f # s
    tconst = 160
    stepsconst = tconst*f
    vreal=np.zeros((stepsconst,3))
    vn = np.zeros((stepsconst,3))
    t = np.zeros(stepsconst)
    


    v = np.array([0.2, 0, 0]) # m/s


    for i in tqdm(range(stepsconst), ncols=100): 
        # without noise
        vreal[i] = v

        # with noise
        vn[i]= v + np.random.normal(0,0.2,3)
        
        # time stamp
        t1= i*dt+dt
        t[i]=np.round(t1,8)

   
    with open("venv/Simulation/LineareModel.csv", "w", newline="") as f:
        writer = csv.writer(f, delimiter =",")
        writer.writerow(["Velocity without noise x", "Velocity without noise y", "Velocity without noise z", "Velocity with noise x","Velocity with noise y", "Velocity with noise z", "global time"])
        for i in range(len(vreal)):
            writer.writerow([vreal[i][0], vreal[i][1], vreal[i][2], vn[i][0], vn[i][1], vn[i][2], t[i]])

if selection == False: #stationary
    """ stationary - data
    kinematic model: 
        x = x0 + v0 * dt + 0.5*(dt**2)*a
        x0 = x
        v0 = v0 + a*dt
    """

    f = config.f_ac  # Hz
    dt = 1/f  # s
    

    tstatio = 250


    steps = tstatio * f
    vreal = np.zeros((steps,3))
    vn = np.zeros((steps,3))
    t = np.zeros(steps)
    t1 = 0
    v = np.array([0,0,0]) # m/s


    for i in tqdm(range(steps), ncols=100): 
        # without noise 
        vreal[i] = v

        # with noise 
        vn[i]= np.array(v) + np.random.normal(0,0.2,3)
        
        # time stamp 
        t[i]=np.round(t1,8)
        t1= i*dt+dt

    with open("venv/Simulation/StationaryModel.csv", "w", newline="") as f:
        writer = csv.writer(f, delimiter =",")
        writer.writerow(["Velocity without noise x", "Velocity without noise y","Velocity with noise z","Velocity with noise y", "Velocity with noise z", "global time"])
        for i in range(len(vreal)):
            writer.writerow([vreal[i][0], vreal[i][1], vreal[i][2], vn[i][0], vn[i][1], vn[i][2], t[i]])

if selection == True: #circular
    """ Circle Model - Data
    basic kinematic model: 
        x = x0 + v0 * dt + 0.5*(dt**2)*a
        x0 = x
        v0 = v0 + a*dt
    """

    f = config.f_ac  # Hz
    dt = 1/f  # s
    

    tstatio = 120


    steps = tstatio * f
    vreal = np.zeros((steps,3))
    vn = np.zeros((steps,3))
    t = np.zeros(steps)
    t1 = 0
    w = 0.1
    radius = 1


    for i in tqdm(range(steps), ncols=100):
        # without noise 
        v1 = 0.2*np.array([radius*np.cos(w*t1), radius*np.sin(w*t1), 0])
        
        vreal[i] = v1
        
        # with noise 
        vn[i]= np.array(vreal[i]) + np.random.normal(0,0.2,3)
        
        # time stamp
        t[i]=np.round(t1,8)
        
        t1= i*dt+dt
        


    with open("venv/Simulation/CircleModel.csv", "w", newline="") as f:
        writer = csv.writer(f, delimiter =",")
        writer.writerow(["Velocity without noise x", "Velocity without noise y","Velocity with noise z","Velocity with noise y", "Velocity with noise z", "global time"])
        for i in range(len(vreal)):
            writer.writerow([vreal[i][0], vreal[i][1], vreal[i][2], vn[i][0], vn[i][1], vn[i][2], t[i]])