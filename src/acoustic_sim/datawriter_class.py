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
        self.ID = []
        self.t_meas = []
        self.errMeas = []
        self.zhat = []
        self.tmp = os.path.dirname(__file__)
    
    def fillState(self, x,y,z,time):
        self.x = x
        self.y = y
        self.z = z
        self.t_state = time
    
    def fillMeas(self, time, dist, ID):
        self.t_meas = time
        self.dist = dist
        self.ID = ID
    
    def fillErr(self, err, zhat):
        self.errMeas = err
        self.zhat = zhat
    
    def writeCSVState(self):
        with open((os.path.join(self.tmp, "Data/alternating/100Hz_circle_20m_02513ms_alternating_PRR1000_STD0_PollTime1500_state.csv")), mode="a", newline="") as f:
            writer = csv.writer(f, delimiter = ",")
            writer.writerow([self.t_state, self.x, self.y, self.z])
    
    def writeCSVMeas(self):
        with open((os.path.join(self.tmp, "Data/alternating/100Hz_circle_20m_02513ms_alternating_PRR1000_STD0_PollTime1500_meas.csv")), mode="a", newline="") as f:
            writer = csv.writer(f, delimiter = ",")
            writer.writerow([self.t_meas, self.dist, self.zhat, self.ID, self.errMeas])

