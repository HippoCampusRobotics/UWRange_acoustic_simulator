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
        with open((os.path.join(self.tmp, "Data/broadcast/100Hz_circle_20m_02513m_s_PRR100_STD0_A2Delay10ms_state.csv")), mode="a", newline="") as f:
            writer = csv.writer(f, delimiter = ",")
            writer.writerow([self.t_state, self.x, self.y, self.z])
    
    def writeCSVMeas(self):
        with open((os.path.join(self.tmp, "Data/broadcast/100Hz_circle_20m_02513m_s_PRR100_STD0_A2Delay10ms_meas.csv")), mode="a", newline="") as f:
            writer = csv.writer(f, delimiter = ",")
            writer.writerow([self.t_meas, self.dist, self.zhat, self.ID, self.errMeas])

