#!/usr/bin/env python3
"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
import os
import sys

from datetime import datetime
import pandas as pd

POSE_HEADER = ['s','ns','x','y','q.x','q.y','q.z','q.w']
VESC_HEADER = ['s','ns','phi_r','phi_p', 'phi_y', 'ax', 'ay', 'az', 'wx', 'wy', 'wz', 'q.x','q.y','q.z','q.w']
ACKR_HEADER = ['s','ns','V','delta']
ODOM_HEADER = ['s', 'ns', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'x', 'y', 'q.x','q.y','q.z','q.w']
SCAN_HEADER = ['s', 'ns', 'amax', 'amin', 'ai', 'ti', 'st', 'rmin', 'rmax']

r = []
I = []
for i in range(1080):
    r += ['r' + str(i)]
    I += ['I' + str(i)]
SCAN_HEADER += r + I

if __name__ == '__main__':
    dataSetNumber = 1
    dataSetPath = '../../../state_logs/DS'+str(dataSetNumber)
    print(os.getcwd())
    print(os.listdir('../../../state_logs/DS1'))
    pfPose  = np.loadtxt(dataSetPath+'/pfPoseData.csv', delimiter=',', skiprows=1)[:,1:]
    pfOdom  = np.loadtxt(dataSetPath+'/pfOdomData.csv', delimiter=',', skiprows=1)[:,1:]
    egoOdom = np.loadtxt(dataSetPath+'/odomData.csv', delimiter=',', skiprows=1)[:,1:]
    imuData = np.loadtxt(dataSetPath+'/imuData.csv', delimiter=',', skiprows=1)[:,1:]
    cmdData = np.loadtxt(dataSetPath+'/commandData.csv', delimiter=',', skiprows=1)[:,1:]

    x_loc   = pfPose[:, 2]
    y_loc   = pfPose[:, 3]
    plt.scatter(x_loc, y_loc)
    plt.show()
    print("HELLO THERE")
    