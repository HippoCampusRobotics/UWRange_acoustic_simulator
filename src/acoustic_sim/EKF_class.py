#!/usr/bin/env python

from __future__ import print_function
import numpy as np

import threading

from numpy.core.shape_base import atleast_2d

# ExtendedKalmanFilter
class EKF(object):
    def __init__(self, measurement_model, process_model, x0, p0_mat):
        self.x_est_0 = x0
        self.dim_state = len(self.x_est_0)
        self.x_est = self.x_est_0
        self.x_est_last = self.x_est
        self.p0_mat = p0_mat
        self.p_mat = self.p0_mat
        self.lock = threading.Lock()
        self.time = 0

        self.process_model = process_model
        self.measurement_model = measurement_model

    def get_x_est(self):
        return np.copy(self.x_est)

    def get_x_est_0(self):
        return np.copy(self.x_est_0)

    def get_p_mat(self):
        return np.copy(self.p_mat)

    def get_x_est_last(self):
        return np.copy(self.x_est_last)
    
    def set_state(self, state):
        self.x_est = state
    
    def set_covar(self, covar):
        self.p_mat = covar
    
    def set_time(self, time):
        self.time = time

    def reset(self, x_est_0=None, p0_mat=None):
        if x_est_0:
            self._x_est = x_est_0
        else:
            self._x_est = self.x_est_0
        if p0_mat:
            self._p_mat = p0_mat
        else:
            self._p_mat = self.p0_mat

    def predict(self, t, data):
        dt = t - self.time
        self.time = t
        self.x_est_last = self.x_est
        self.x_est = self.process_model.f(self.get_x_est(), dt, data)
        a_mat = self.process_model.f_jacobian(self.get_x_est(), dt, data)
        self.p_mat = np.dot(np.dot(a_mat, self.get_p_mat()),
                                a_mat.transpose()) + self.process_model.V
        """
        # reset EKF if unrealistic values
        if self.get_x_est()[0] > 5 or self.get_x_est()[1] > 5:
            print('Resetting EKF: x or y value outside tank')
            self.reset()
        
        elif not np.all(np.isfinite(self.get_x_est())):
            print('Resetting EKF: unrealistically high value')
            print('x: ', self.get_x_est())
            self.reset()
        """ 
        return self.x_est, self.p_mat

    def update_vision_data(self, measurements, detected_tags):
        # measurement is: dist, yaw to each tag

        self.x_est_last = self._x_est
        z_est_vision = self.measurement_model.h_vision_data(self.get_x_est(), detected_tags)
        h_mat_vision = self.measurement_model.h_jacobian_vision_data(self.get_x_est(), detected_tags)
        w_mat_vision = self.measurement_model.vision_dynamic_meas_model(self.get_x_est(), measurements, detected_tags)

        y_vision = measurements - z_est_vision
        # wrap yaw innovation (every second entry) so it is between -pi and pi
        y_vision[1::2] = np.arctan2(np.sin(y_vision[1::2]),
                                    np.cos(y_vision[1::2]))

        self.x_est, self.p_mat = self._update(self.get_x_est(),
                                                self.get_p_mat(), y_vision,
                                                h_mat_vision, w_mat_vision)

        return True

    def update_orientation_data(self, measurements):
        # measurement is: roll, pitch from /mavros/local_position/pose

        z_est_orient = self.measurement_model.h_orientation_data(
            self.get_x_est())
        h_mat_orient = self.measurement_model.h_jacobian_orientation_data()

        y_orient = measurements - z_est_orient
        # wrap angle innovations between -pi and pi
        y_orient = np.arctan2(np.sin(y_orient), np.cos(y_orient))

        self.x_est, self.p_mat = self._update(
            self.get_x_est(), self.get_p_mat(), y_orient, h_mat_orient,
            self.measurement_model.w_mat_orientation)
        
        return True

    def update_imu_data(self, measurements, w_mat_imu):
        # measurement is either: body rates + lin. acceleration
        #               or: only body rates

        # check which measurements we're using:
        if measurements.shape[0] == 3:
            # only using angular velocities
            using_lin_acc = False
        elif measurements.shape[0] == 6:
            using_lin_acc = True
        else:
            print("IMU measurement has unexpected size!")
            using_lin_acc = False

        z_est_imu = self.measurement_model.h_imu_data(self.get_x_est(),
                                                      using_lin_acc)
        h_mat_imu = self.measurement_model.h_jacobian_imu_data(using_lin_acc)
        y_imu = measurements - z_est_imu
        self._x_est, self._p_mat = self._update(self.get_x_est(),
                                                self.get_p_mat(), y_imu,
                                                h_mat_imu, w_mat_imu)

        return True

    def update_depth(self, measurements, w_mat_depth):
        z_est_depth = self.measurement_model.h_depth(self.get_x_est())
        h_mat_depth = self.measurement_model.h_jacobian_depth(self.get_x_est())

        y = np.array([measurements - z_est_depth])
        self.x_est, self.p_mat = self._update(self.get_x_est(), self.get_p_mat(),
                                                y, h_mat_depth, w_mat_depth )

        return self.x_est, self.p_mat       

    def update_dist(self, measurements, w_mat_dist):
        beacon = measurements[0] 
        dist = measurements[1]
        z_est_dist = self.measurement_model.h_dist(self.get_x_est(), beacon)
        h_mat_dist = self.measurement_model.h_jacobian_dist(self.get_x_est(), beacon)
        
        y = np.array([dist - z_est_dist])
        self.x_est, self.p_mat = self._update(self.get_x_est(), self.get_p_mat(),
                                                y, h_mat_dist, w_mat_dist )
        return self.x_est, self.p_mat, y

    def _update(self, x_est, p_mat, y, h_mat, w_mat):
        """ helper function for general update """
        # compute K gain
        
        tmp = np.dot(np.dot(h_mat, p_mat), h_mat.transpose()) + w_mat
        k_mat = np.dot(np.dot(p_mat, h_mat.transpose()),
                          np.linalg.inv(tmp))

        # update state
        
        x_est += np.dot(k_mat, y)
        # update covariance
        p_tmp = np.eye(self.dim_state) - np.dot(k_mat, h_mat)
        p_mat = np.dot(p_tmp, p_mat)
        return x_est, p_mat