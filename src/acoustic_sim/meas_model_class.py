#!/usr/bin/env python

from __future__ import print_function
import numpy as np
from numpy.core.fromnumeric import shape
"""
Different measurement models for the EKF for the visual localization
    - Distances: using distance and yaw angle to tag as measurement
    - ...
EKF state: [x, y, z, roll, pitch, yaw, dx, dy, dz, droll, dpitch, dyaw]
(linear and angular velocities in body frame)
"""


class MeasurementModelDistances(object):
    def __init__(self, w_mat_vision, c_penalty_dist,
                 c_penalty_yaw, w_mat_orientation):
        
        self._w_mat_vision_static = w_mat_vision
        self._c_penalty_dist = c_penalty_dist
        self._c_penalty_yaw = c_penalty_yaw
        self.w_mat_orientation = w_mat_orientation

    def h_vision_data(self, x_est, detected_tags):
        # measurement is: distance and yaw-angle to each tag
        num_tags = detected_tags.shape[0]
        dim_meas = len(detected_tags)
        z_est = np.zeros((num_tags * dim_meas, 1))

        for i, tag in enumerate(detected_tags):
            tag_pos = tag[1:4]
            dist = self.get_dist(x_est, tag_pos)
            yaw = x_est[5]

            z_est[i * self._dim_meas, 0] = dist
            z_est[i * self._dim_meas + 1, 0] = yaw

        return z_est  # dim [num_tags*dim_meas X 1]

    def h_jacobian_vision_data(self, x_est, detected_tags):
        num_tags = detected_tags.shape[0]
        dim_meas = len(x_est)
        dim_state = len(detected_tags)
        h_mat = np.zeros((num_tags * dim_meas, dim_state))

        for i, tag in enumerate(detected_tags):
            tag_pos = tag[1:4]
            dist = self.get_dist(x_est, tag_pos)

            # dh /dx = 1/2 * (dist ^2)^(-1/2) * (2 * (x1 - t1) * 1)
            h_jac_x = (x_est[0] - tag_pos[0]) / dist
            # dh /dy
            h_jac_y = (x_est[1] - tag_pos[1]) / dist
            # dh /dz
            h_jac_z = (x_est[2] - tag_pos[2]) / dist
            # dh /dyaw
            h_jac_yaw = 1.0

            h_mat[dim_meas * i, 0:3] = [h_jac_x, h_jac_y, h_jac_z]
            h_mat[dim_meas * i + 1, 5] = h_jac_yaw
            # all other derivatives are zero

        return h_mat  # dim [num_tags*dim_meas X dim_state]

    def h_orientation_data(self, x_est):
        # measurement is: roll, pitch from /mavros/local_position/pose
        z_est = np.array([x_est[3], x_est[4]]).reshape((-1, 1))
        return z_est  # dim: [2 X 1]

    def h_jacobian_orientation_data(self):
        dim_state = 5
        h_mat = np.zeros((2, dim_state))
        # all derivatives zero except for roll, pitch:
        h_mat[0, 3] = 1.0  # dh /droll
        h_mat[1, 4] = 1.0  # dh/ dpitch
        return h_mat  # dim [2 X dim_state]

    def h_imu_data(self, x_est, using_lin_acc=False):
        if not using_lin_acc:
            # measurement is: roll rate, pitch rate, yaw rate
            z_est = np.array([x_est[9], x_est[10], x_est[11]]).reshape((-1, 1))
        else:
            # measurement is: angular velocities and linear accelerations
            rospy.logfatal(
                "[%s] Using linear acceleration measurements from IMU!" +
                "Not implemented yet", rospy.get_name())
        return z_est

    def h_jacobian_imu_data(self, using_lin_acc=False):
        dim_state = 3
        if not using_lin_acc:
            h_mat = np.zeros((3, dim_state))
            # all derivatives zero except for body rates
            h_mat[0, 9] = 1.0
            h_mat[1, 10] = 1.0
            h_mat[2, 11] = 1.0
        else:
            rospy.logfatal(
                "[%s] Using linear acceleration measurements from IMU! " +
                "Not implemented yet", rospy.get_name())

        return h_mat  # dim [3 X dim_state]
    
    def h_dist(self, x_est, beacon):
        # measurement is: distance and yaw-angle to each tag
        z_est = np.linalg.norm(x_est - beacon)

        return z_est  # dim [num_tags*dim_meas X 1]

    def h_jacobian_dist(self, x_est, beacon):
        dim_state = len(x_est)
        dim_meas = len([beacon])
        h_mat = np.zeros((dim_meas, dim_state))
        
        dist = np.linalg.norm(x_est - beacon)
    
        # dh /dx = 1/2 * (dist ^2)^(-1/2) * (2 * (x1 - t1) * 1)
        h_jac_x = (x_est[0] - beacon[0]) / dist
        # dh /dy
        h_jac_y = (x_est[1] - beacon[1]) / dist
        # dh /dz
        h_jac_z = (x_est[2] - beacon[2]) / dist

        h_mat[0, 0:3] = [h_jac_x, h_jac_y, h_jac_z]

        return h_mat  # dim [num_tags*dim_meas X dim_state]

    def h_depth(self, x_est):
        
        z_est = x_est[2]
        return z_est

    def h_jacobian_depth(self, x_est):
        dim_state = len(x_est)
        h_mat = np.zeros((1, dim_state))
        h_jac_x = 0
        h_jac_y = 0
        h_jac_z = 1
        h_mat[0,0:3] = [h_jac_x, h_jac_y, h_jac_z]

        return h_mat

    def vision_dynamic_meas_model(self, x_est, measurements, detected_tags):
        # currently not using measured tag-position in camera coordinates,
        # but known tag position from calibration, since currently not possible
        # to (nicely) pass full tag-pose measurement to method

        num_tags = detected_tags.shape[0]
        # initialize dynamic W
        w_mat_dyn = np.zeros(
            (num_tags * self._dim_meas, num_tags * self._dim_meas))

        for i, tag in enumerate(detected_tags):
            # tag_pos = tag[1:4]
            # dist = sqrt((x - x_tag) ^ 2 + (y - y_tag) ^ 2 + (z - z_tag) ^ 2)
            # dist = self.get_dist(x_est, tag_pos)

            # add dynamic noise to measurement noise for distance measurement
            # w_mat_dyn[self._dim_meas * i, self._dim_meas * i] = dist / ((x_est[2] - tag_pos[2])
            #                                                  * self._c_penalty_dist) + self._w_mat_vision_static[0, 0]
            # add dynamic noise to measurement noise for yaw measurement
            # w_mat_dyn[self._dim_meas * i + 1, self._dim_meas * i + 1] = dist / ((x_est[2] - tag_pos[2])
            #                                                   * self._c_penalty_yaw) + self._w_mat_vision_static[1, 1]

            # debugging: not dynamic
            # noise for distance measurement
            w_mat_dyn[self._dim_meas * i,
                      self._dim_meas * i] = self._w_mat_vision_static[0, 0]
            # noise for yaw measurement
            w_mat_dyn[self._dim_meas * i + 1,
                      self._dim_meas * i + 1] = self._w_mat_vision_static[1, 1]

        return w_mat_dyn  # dim [num_tags*dim_meas X num_tag*dim_meas]

    def get_dist(self, x_est, tag_pos):
        # dist = sqrt((x - x_tag) ^ 2 + (y - y_tag) ^ 2 + (z - z_tag) ^ 2)
        dist = np.sqrt((x_est[0] - tag_pos[0])**2 +
                       (x_est[1] - tag_pos[1])**2 + (x_est[2] - tag_pos[2])**2)
        return dist