#!/usr/bin/env python

import csv
import numpy as np
import yaml
import os 

tmp = os.path.dirname(__file__)


class dataLoader():
    """ Config file inclued all scalable params of UKF, Acoustic Simulation, 
    and global frame params"""

        
#### Database ####
    data = []
    v = []
    vn = []
    t = []
    with open(os.path.join(tmp,"Data/LineareModel.csv"), "r") as f:
    #with open(os.path.join(tmp,"Data/StationaryModel.csv"), "r") as f:
    #with open(os.path.join(tmp,"Data/CircleModel.csv"), "r") as f:
        reader = csv.reader(f, delimiter = ",")
        next(reader, None)
        for row in reader:
            data.append({"Velocity without noise x": row[0], "Velocity without noise y": row[1],"Velocity without noise z": row[2],"Velocity with noise x": row[3], "Velocity with noise y": row[4], "Velocity with noise z": row[5], "global time": row[6]})
            
    for i in range(len(data)):
        v.append([float(data[i].get("Velocity without noise x")), float(data[i].get("Velocity without noise y")), float(data[i].get("Velocity without noise z"))])
        vn.append([float(data[i].get("Velocity with noise x")), float(data[i].get("Velocity with noise y")), float(data[i].get("Velocity with noise z"))])
        t.  append(float(data[i].get("global time")))
