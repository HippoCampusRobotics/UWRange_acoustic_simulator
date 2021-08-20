#!/usr/bin/env python
import matplotlib as plt
import numpy as np

def TaitBryanAngles3D(input): # input has to be 3x1; transformation from bodyfixed to global fixed
    x,y,z = input[0], input[1], input[2]
    R,P,Y = input[3], input[4], input[5]
    sinR, sinP, sinY = np.sin(R), np.sin(P), np.sin(Y)
    cosR, cosP, cosY = np.cos(R), np.sin(P), np.sin(Y)

    output = np.array([x * (cosP*cosY) + y * (sinR*sinP*cosY - cosR*sinY) + z * (cosR*sinP*cosY + sinR*sinY),
                        x * (cosP*sinY) + y * (sinR*sinP*sinY + cosR*cosY) + z * (cosR*sinP*sinY - sinR*cosY),
                        x * (-sinP) + y * (sinR*cosP) + z * (cosR*cosP)])
    
    return output

def VectorNorm(vector1, vector2): #input need to be two vectors same size
    output = np.linalg.norm(vector2-vector1)
    return output

def depth_calculator(pressure):
    pascal_per_meter = 1.0e4
    airPressure = 1.0e5
    output = -(pressure- airPressure) / pascal_per_meter
    return output
