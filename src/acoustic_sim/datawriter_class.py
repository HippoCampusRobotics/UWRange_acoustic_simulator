import csv
import numpy as np
import os

class datawriter():
    def __init__(self):
        self.x = []
        self.y = []
        self.z = []
        self.t_state = []
        self.dist = []
        self.realDist = []
        self.ID = []
        self.t_meas = []
        self.tr = []
        self.Error = []
        self.errMeas = []
        self.zhat = []
        self.tmp = os.path.dirname(__file__)
    
    def fillState(self, x,y,z,time):
        self.x = x
        self.y = y
        self.z = z
        self.t_state = time
    
    def fillMeas(self, time, tr, dist, realdist, ID, error):
        self.t_meas = time
        self.dist = dist
        self.realDist = realdist
        self.ID = ID
        self.tr = tr
        self.Error = error
    
    def fillErr(self, err, zhat):
        self.errMeas = err
        self.zhat = zhat
    
    def writeCSVState(self):
        with open((os.path.join(self.tmp, "Data/Fabian/100Hz_Finkenwerder_PRR100_STD0_broadcast_state.csv")), mode="a", newline="") as f:
            writer = csv.writer(f, delimiter = ",")
            writer.writerow([self.t_state, self.x, self.y, self.z])
    
    def writeCSVMeas(self):
        with open((os.path.join(self.tmp, "Data/Fabian/100Hz_Finkenwerder_PRR100_STD0_broadcast_meas.csv")), mode="a", newline="") as f:
            writer = csv.writer(f, delimiter = ",")
            writer.writerow([self.t_meas, self.tr, self.ID, self.dist, self.realDist, self.Error])

