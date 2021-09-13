#!/usr/bin/env python

from __future__ import print_function
import numpy as np
from numpy.core.fromnumeric import shape
"""
Different process models for the EKF for the visual localization
    - simple model: no prediction, new state = old state
    - ...
EKF state: [x, y, z, roll, pitch, yaw, dx, dy, dz, droll, dpitch, dyaw]
(linear and angular velocities in body frame)
"""


class ProcessModel(object):
    """ Simple process model: no prediction """
    def __init__(self, dim_state, V):
        self._dim_state = dim_state
        self.V = V

    def f(self, x_est, dt):
        x_next = np.copy(x_est)
        return x_next

    def f_jacobian(self, x_est, dt):
        A = np.eye(self._dim_state)
        return A  # dim [dim_state X dim_state]

class ProcessModelVelecitiesGlobal(ProcessModel):
    def __init__(self, dim_state):
        self._dim_state = dim_state
        self.V = np.eye(dim_state)
        
    def f(self, x_est, dt, data):
        
        
        x_est = x_est + np.array(data) * dt
        return x_est

    def f_jacobian(self, x_est, dt, data):
        ret = np.eye(self._dim_state)
        return ret
        
class ProcessModelVelocities(ProcessModel):
    """ Simple point mass model,
        for derivation see scripts/matlab/point_mass_model"""
    def __init__(self, dim_state, V):
        super(ProcessModelVelocities, self).__init__(dim_state, V)

    def f(self, x_est, dt):
        x_next = np.copy(x_est)
        x = x_next[0]
        y = x_next[1]
        z = x_next[2]
        R = x_next[3]
        P = x_next[4]
        Y = x_next[5]
        dx = x_next[6]
        dy = x_next[7]
        dz = x_next[8]
        # rr = x_next[9]
        # pr = x_next[10]
        # yr = x_next[11]

        # see matlab script
        x_next[:3] = np.array([
            x + dt *
            (dz *
             (np.sin(R) * np.sin(Y) + np.cos(R) * np.cos(Y) * np.sin(P)) - dy *
             (np.cos(R) * np.sin(Y) - np.cos(Y) * np.sin(P) * np.sin(R)) +
             dx * np.cos(P) * np.cos(Y)), y + dt *
            (dy *
             (np.cos(R) * np.cos(Y) + np.sin(P) * np.sin(R) * np.sin(Y)) - dz *
             (np.cos(Y) * np.sin(R) - np.cos(R) * np.sin(P) * np.sin(Y)) +
             dx * np.cos(P) * np.sin(Y)),
            z + dt * (dz * np.cos(P) * np.cos(R) - dx * np.sin(P) +
                      dy * np.cos(P) * np.sin(R))
        ])

        return x_next

    def f_jacobian(self, x_est, dt):
        # x = x_est[0]
        # y = x_est[1]
        # z = x_est[2]
        R = x_est[3]
        P = x_est[4]
        Y = x_est[5]
        dx = x_est[6]
        dy = x_est[7]
        dz = x_est[8]
        # dr = x_est[9]
        # dp = x_est[10]
        # dy = x_est[11]

        A = np.array([
            [
                1, 0, 0, dt *
                (dy *
                 (np.sin(R) * np.sin(Y) + np.cos(R) * np.cos(Y) * np.sin(P)) +
                 dz *
                 (np.cos(R) * np.sin(Y) - np.cos(Y) * np.sin(P) * np.sin(R))),
                dt * np.cos(Y) * (dz * np.cos(P) * np.cos(R) - dx * np.sin(P) +
                                  dy * np.cos(P) * np.sin(R)), -dt *
                (dy *
                 (np.cos(R) * np.cos(Y) + np.sin(P) * np.sin(R) * np.sin(Y)) -
                 dz *
                 (np.cos(Y) * np.sin(R) - np.cos(R) * np.sin(P) * np.sin(Y)) +
                 dx * np.cos(P) * np.sin(Y)), dt * np.cos(P) * np.cos(Y),
                dt * np.cos(Y) * np.sin(P) * np.sin(R) -
                dt * np.cos(R) * np.sin(Y), dt *
                (np.sin(R) * np.sin(Y) + np.cos(R) * np.cos(Y) * np.sin(P)), 0,
                0, 0
            ],
            [
                0, 1, 0, -dt *
                (dy *
                 (np.cos(Y) * np.sin(R) - np.cos(R) * np.sin(P) * np.sin(Y)) +
                 dz *
                 (np.cos(R) * np.cos(Y) + np.sin(P) * np.sin(R) * np.sin(Y))),
                dt * np.sin(Y) * (dz * np.cos(P) * np.cos(R) - dx * np.sin(P) +
                                  dy * np.cos(P) * np.sin(R)), dt *
                (dz *
                 (np.sin(R) * np.sin(Y) + np.cos(R) * np.cos(Y) * np.sin(P)) -
                 dy *
                 (np.cos(R) * np.sin(Y) - np.cos(Y) * np.sin(P) * np.sin(R)) +
                 dx * np.cos(P) * np.cos(Y)), dt * np.cos(P) * np.sin(Y), dt *
                (np.cos(R) * np.cos(Y) + np.sin(P) * np.sin(R) * np.sin(Y)),
                dt * np.cos(R) * np.sin(P) * np.sin(Y) -
                dt * np.cos(Y) * np.sin(R), 0, 0, 0
            ],
            [
                0, 0, 1, dt * np.cos(P) * (dy * np.cos(R) - dz * np.sin(R)),
                -dt * (dx * np.cos(P) + dz * np.cos(R) * np.sin(P) +
                       dy * np.sin(P) * np.sin(R)), 0, -dt * np.sin(P),
                dt * np.cos(P) * np.sin(R), dt * np.cos(P) * np.cos(R), 0, 0, 0
            ],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        ],
                     dtype=float)

        return A  # dim [dim_state X dim_state]