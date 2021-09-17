#!/usr/bin/env python

from datetime import time
from tqdm import tqdm
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.axes as ax
from .UKF_class import UKF
from .EKF_class import EKF
import rospy
from .process_model_class import ProcessModelVelecitiesGlobal
from .meas_model_class import MeasurementModelDistances
from collections import deque
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import FluidPressure
import os
import json

class localisationSimulation():
    def __init__(self):

        tmp = os.path.dirname(__file__)
        file_path_filter = os.path.join(tmp, '../../config/acoustic_config.json')
        f = open(file_path_filter)
        self.acoustic_config = json.load(f)
        f.close()

        file_path_filter = os.path.join(tmp, '../../config/filter_config.json')
        f = open(file_path_filter)
        self.filter_config = json.load(f)
        f.close()
        
        
        
        self.t0 = 0
        self.dt = 0
        self.t = self.t0
        self.last_t = self.t0
        self.z = None
        self.x = None
        self.x_est = None
        self.statePre = None
        self.covarPre = None
        self.p_mat = None 
    
        # settings from Configfile
        self.packetLengthResponse = self.acoustic_config["config"][0]["PacketLengthResponse"]
        self.publishDelay = self.acoustic_config["config"][0]["PublishDelay"]
        self.filter = self.filter_config["config"][0]["filterTyp"]
        if self.filter == "UKF":
            self.x0 = np.array(self.filter_config["config"][1]["settings"]["InitState"])
            self.p_mat_0 = np.array(self.filter_config["config"][1]["settings"]["InitCovar"])
        elif self.filter == "EKF":
            self.x0 = np.array(self.filter_config["config"][2]["settings"]["InitState"])
            self.p_mat_0 = np.array(self.filter_config["config"][2]["settings"]["InitCovar"])

        else: print("[Localisation_sim] Wrong Filter selected")
        self.w_mat_depth = self.filter_config["config"][2]["settings"]["Qt_depth"]
        self.w_mat_dist = self.filter_config["config"][2]["settings"]["Qt_dist"]
        
        # List to handle data
        self.dataBag = deque([])
        self.lenDataBag = self.filter_config["config"][0]["lengthDatabag"]
        
        # List for simulation
        self.xest = []
        self.yest = []
        self.zest = []
        self.timeest = []

        # filter instances
        self.measurement_model = MeasurementModelDistances(1,1,1,1)
        self.process_model = ProcessModelVelecitiesGlobal(3) # 3 = dim_state
        if self.filter == "UKF":
            self.Kalmanfilter = UKF(self.measurement_model, self.process_model, self.x0, self.p_mat_0)  # prediction and update done in this instance

        elif self.filter == "EKF":
            self.Kalmanfilter = EKF(self.measurement_model, self.process_model, self.x0, self.p_mat_0)  # prediction and update done in this instance
     

    def fillDatabag(self, list): # list = [t, preInput, x_est, p_mat,]
        self.dataBag.append(list)
        if len(self.dataBag)> self.lenDataBag:
            self.dataBag.popleft()

    def recalculateState(self, correctedTime, measurements):
        numberIterations = 0
        for i in range(len(self.dataBag)):
            if correctedTime >= self.dataBag[-i-1][0]:
                break
            elif correctedTime < self.dataBag[-i-1][0]:
                numberIterations +=1
                if numberIterations == len(self.dataBag):
                    numberIterations = 0 # CorrectedTime is newer than every safed time
                    break
            else:
                print("Error: [Localisation_Sim]; no matching timestamp found") 

        # x_est, p_mat, t / databag: [self.t, preInput, depth, self.x_est, self.p_mat]false
        self.setFilter(self.dataBag[-numberIterations-1][3], self.dataBag[-numberIterations-1][4], self.dataBag[-numberIterations-1][0])
        self.update(correctedTime, self.dataBag[-numberIterations-1][1],measurements)
        
        for i in range(numberIterations):
            self.xest = self.predict(self.dataBag[-numberIterations+i][0], self.dataBag[-numberIterations+i][1], self.dataBag[-numberIterations+i][2])

    def setFilter (self, x_est, p_mat, t):

        self.Kalmanfilter.set_state(x_est)
        self.Kalmanfilter.set_covar(p_mat)
        self.Kalmanfilter.set_time(t)
        
    def update(self, correctedTime, preInput, measurements):
        
        self.Kalmanfilter.predict(correctedTime, preInput)  # launch prediction step with time stamp and noisy velocity; 
        self.x_est, self.p_mat, z = self.Kalmanfilter.update_dist(measurements, self.w_mat_dist) # launch update step with published data; return: self.x_est = updated state, z = delta between z and zhat       

    def predict(self, t, preInput, depth): # just a function for debugging and to have a camparison
        
        self.x_est, self.p_mat = self.Kalmanfilter.predict(t, preInput)  # launch prediction step with time stamp and noisy velocity; return: x = predicted state, p = predicted covariance
        self.x_est, self.p_mat = self.Kalmanfilter.update_depth(depth, self.w_mat_depth)
            

    def locate(self, preInput, t, depth, meas):
        
        self.t = t
        self.dt = self.t-self.last_t
        self.last_t = self.t
                
        # If a range is published a prediction and update step will be launched
        if meas is not None:
            # meas: Beacon Index (int), Beacon coordinates (array), measured distance (float), time stamp (float)
            correctedTime = meas["time_published"] - meas["PacketLengthResponse"]  # get time stamp
            beacon = meas["ModemPos"]
            dist = meas["dist"]
            measurements = [beacon, dist] # Position Beacon, Distance
            self.recalculateState(correctedTime, measurements)

        else:# n = frequency of acoustic simulation / frequency of prediction steps
            self.predict(self.t, preInput, depth)
        
        list = [self.t, preInput, depth, self.x_est, self.p_mat]
        self.fillDatabag(list)
        return self.x_est
    
    def getBeaconPos(self, BeaconIndex):
        for i in self.acoustic_config["config"]:
            if i["type"] == "anchor":
                if i["modem"]["id"] == BeaconIndex:
                    return i["position"]