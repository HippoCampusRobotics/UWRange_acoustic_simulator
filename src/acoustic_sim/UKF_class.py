#!/usr/bin/env python
"""
The implementation of this UKF is based on the explanation published in the book: "Probabilistic Robotics", 
written by Sebastian Thrun, Wolfram Burgard, Dieter Fox (following RB). It can be found at chapter 3.4.

Inputs:
    for prediction:
        UKF.UKF(IMUData, meas, dt) meas == np.array([None])
    for prediction and update:
        UKF.UKF(IMUData, meas, dt) meas == measuring data (np.array([Beaconx,Beacony,distance])
"""

import numpy as np
from numpy.core.shape_base import atleast_2d
from scipy import linalg
import scipy
import os
import json

class UKF():

    def __init__(self, measurement_model, process_model, x0, p0_mat):
        """
        Param: 
            self.x (np.array) - 2x1 state
            self.n_dim (int) - dimension of state
            self.n_sig (int) - dimension of sigma points
            self.CovarMat (np.array) - 2x2 covarianz matrix
            self.x_pre  (np.array) - predicted state
            self.CovarMat_pre (np.array) - 2x2 predicted covarianz matrix
            self.R (np.array) - 2x2 Processnoise Rt
            self.Q (np.array) - 1x1 Covarianzmatrix of Rt (Ot)
        """
        tmp = os.path.dirname(__file__)
        file_path_filter = os.path.join(tmp, '../../config/filter_config.json')
        f = open(file_path_filter)
        self.filter_config = json.load(f)
        f.close()

        self.x = x0 #config.InitState.T
        self.x_pre = self.x
        self.CovarMat = np.atleast_2d(p0_mat)
        self.CovarMat_pre = self.CovarMat
        self.n_dim = len(self.x)   
        self.n_sig = 2*self.n_dim+1 
        self.R = np.array(self.filter_config["config"][1]["settings"]["ProcessNoiseRt"])
        self.Q_depth = np.array(self.filter_config["config"][1]["settings"]["Qt_depth"])
        self.Q_dist = np.array(self.filter_config["config"][1]["settings"]["Qt_dist"])
        self.time = 0

        self.process_model = process_model
        self.measurement_model = measurement_model

        # tuning params for UKF
        k = self.filter_config["config"][1]["settings"]["k"]
        beta = self.filter_config["config"][1]["settings"]["beta"]
        alpha = self.filter_config["config"][1]["settings"]["alpha"]

        # params of UKF
        self.lamda = (alpha**2)*(self.n_dim+k)-self.n_dim
        self.covar_weights = np.zeros((self.n_sig))
        self.mean_weights = np.zeros((self.n_sig))

        self.mean_weights[0] = self.lamda/(self.n_dim+self.lamda)
        self.covar_weights[0] = self.lamda/(self.n_dim+self.lamda) + (1 - (alpha)**2 + beta)
        for i in range(1, self.n_sig):
            self.covar_weights[i] = 1 / (2*(self.n_dim + self.lamda))
            self.mean_weights[i] = 1 / (2*(self.n_dim + self.lamda))

        # Inital Sigmapoints
        self.SigmaPoints = self.getSigmaPoints()       
        
    def getSigmaPoints(self):
        """ computes sigma points
        Arg:
            self.x (np.array): shape 2x1 contains x,y
            self.CovarMat (np.array): shape 2x2 covariance matrix for self.x
        Returns:
            sigmaMat (np.array): shape 5x2 contains 5 sigma vektors
        """

        # take square root of CovarMat
        sigmaMat = np.zeros((self.n_dim, self.n_sig)) #2x5
        tmp = scipy.linalg.sqrtm((self.n_dim + self.lamda)*self.CovarMat) #2x5
       
        # first vektor in sigmaMat is self.x
        sigmaMat[:,0] = self.x

        # compute missing sigma points
        for i in range(self.n_dim):
            sigmaMat[:,i+1] = self.x + tmp[:,i]
            sigmaMat[:,i+1+self.n_dim] = self.x - tmp[:,i]

        self.SigmaPoints = sigmaMat

        return sigmaMat

    def predict(self, t, data):   
        dt = t- self.time
        self.time = t
        """ SigPoints through model """
        sigmas_out = np.zeros((self.n_sig, self.n_dim))
        for i in range(self.n_sig):
            sigmas_out[i] = self.process_model.f(self.SigmaPoints[:,i], dt, data)
        sigmas_out = sigmas_out.T
        """ X (mean) prediction """
        x_out = np.zeros(self.n_dim)

        # for each variable in X
        for i in range(self.n_dim):
            # the mean of that variable is the sum of the weighted values of that variable for each iterated sigma point
            x_out[i] = sum((self.mean_weights[j] * sigmas_out[i][j] for j in range(self.n_sig)))
        
        """ Covarianz prediction """
        p_out = np.zeros((self.n_dim, self.n_dim))
        
        # for each sigma point
        for i in range(self.n_sig):
            # take the distance from the mean make it a covariance by multiplying by the transpose
            # weight it using the calculated weighting factor and sum
            diff = sigmas_out.T[i] - x_out
            diff = np.atleast_2d(diff)
            p_out += self.covar_weights[i] * np.dot(diff.T, diff)

        # add process noise
        p_out += dt * self.R

        """ Update State"""
        self.x = x_out
        self.CovarMat = p_out
        self.SigmaPoints = sigmas_out
        return self.x, self.CovarMat

    def update_depth(self, measurements, w_mat_depth):
        z = [measurements]
        n_data = len(z)
        obs = np.zeros(self.n_sig)

        # pass each sigma point through sensor model
        for i in range (self.n_sig):
            meas = self.measurement_model.h_depth(self.SigmaPoints[:,i])
            obs[i]=meas
        obs = atleast_2d(obs)
        deltaz = self.update(n_data, obs, z, self.Q_depth)
        return self.x, self.CovarMat

    def update_dist(self, measurements, w_mat_dist):
        """ passes each sigma point through H-function (Rang calculation)
        Args:
            measurements (np.array): Beaconposition, distance
            sigmaMat_new (np.array): shape 5x2 contains the sigma points based on self.x_pre
            beacon (np.array): shape 2x1 contains the global x and y position of beacon
        Returns:
            obs_mat (np.array): 5x1 observation matrix contains range corresponding to each sigma point
        """ 
        
        beacon = np.array(measurements[0]) # global x and y position of Beacon
        z = [measurements[1]] # range of Beacon
        n_data = len(z)
        obs = np.zeros(self.n_sig)

        # pass each sigma point through sensor model
        for i in range (self.n_sig):
            meas = self.measurement_model.h_dist(beacon, self.SigmaPoints[:,i]) 
            obs[i]=meas
        obs = atleast_2d(obs)
        deltaz = self.update(n_data, obs, z, self.Q_dist)
        return self.x, self.CovarMat, deltaz

    def update(self, n_data, obs, z, Q):
        # zhat of Observation
        zhat = np.zeros(n_data)
        for i in range(n_data):
            # the mean of that variable is the sum of the weighted values of that variable for each iterated sigma point
            zhat[i] = sum((self.mean_weights[j] * obs[i][j] for j in range(self.n_sig)))
        deltaZ = z-zhat

        # uncertainty
        S = np.zeros((n_data, n_data))
        for i in range(self.n_sig):    
            diff = obs.T[i] - zhat
            diff = np.atleast_2d(diff)
            S += self.covar_weights[i] * np.dot(diff.T, diff)
        S += Q

        # CrossCovariance
        p_xz = np.zeros((self.n_dim,n_data))
        for i in range(self.n_sig):
            diffstate = self.SigmaPoints[:,i] - self.x
            diffstate = np.atleast_2d(diffstate)
            diffmeas = obs.T[i] - zhat
            diffmeas = np.atleast_2d(diffmeas)
            p_xz += self.covar_weights[i]*np.dot(diffstate.T,diffmeas)

        # Kalman gain
        k = np.dot(p_xz, scipy.linalg.inv(S))


        self.x += np.dot(k, (z - zhat))
        self.CovarMat -= np.dot(k, np.dot(S, k.T))
        self.sigmas = self.getSigmaPoints()
        return deltaZ

    def get_state(self):
        return self.x

    def get_covar(self):
        return self.CovarMat
    
    def set_state(self, state):
        self.x = state
        self.getSigmaPoints()

    def set_covar(self, covar):
        self.CovarMat = covar
        self.getSigmaPoints()

    def set_time(self, time):
        self.time = time

    