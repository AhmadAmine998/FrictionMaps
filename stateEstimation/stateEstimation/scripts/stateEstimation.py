#!/usr/bin/env python3
"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import matplotlib.pyplot as plt
import numpy as np
from stateEstimator import StateEstimator
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

def index_from_time(inputTime, possibleTimes):
    '''
    Matches a given state timestamp to an index

    input: stateTime: float, timestamp of the state
    input: cmdTimes: nx1 float array, timestamps of all possible timestamps

    output: idx: integer, index of the command that matches this state
    '''
    return np.argmin(np.abs(inputTime - possibleTimes))

if __name__ == '__main__':
    dataSetNumber = 2
    dataSetPath = 'DataSets/DS'+str(dataSetNumber)
    pfPose  = np.loadtxt(dataSetPath+'/pfPoseData.csv', delimiter=',', skiprows=1)[:,1:]
    pfOdom  = np.loadtxt(dataSetPath+'/pfOdomData.csv', delimiter=',', skiprows=1)[:,1:]
    egoOdom = np.loadtxt(dataSetPath+'/odomData.csv', delimiter=',', skiprows=1)[:,1:]
    imuData = np.loadtxt(dataSetPath+'/imuData.csv', delimiter=',', skiprows=1)[:,1:]
    cmdData = np.loadtxt(dataSetPath+'/commandData.csv', delimiter=',', skiprows=1)[:,1:]

    TRFC_Filter = StateEstimator()

    stateTime =  pfPose[:, POSE_HEADER.index('s')] +  pfPose[:, POSE_HEADER.index('ns')]*10**(-9)
    imuTime   = imuData[:, VESC_HEADER.index('s')] + imuData[:, VESC_HEADER.index('ns')]*10**(-9)
    cmdTime   = cmdData[:, ACKR_HEADER.index('s')] + cmdData[:, ACKR_HEADER.index('ns')]*10**(-9)

    # State Initialization for speed EKF
    deltaT      = 0
    v_x         = 0
    v_y         = 0
    sigma_v_k_m = np.diag([0.4, 0.4])

    # Estimates used for plotting
    v_hat_x     = np.zeros_like(stateTime)
    v_hat_y     = np.zeros_like(stateTime)
    sigma_v_k   = np.zeros((stateTime.shape[0], 2, 2))

    for i, t in enumerate(stateTime):
        # Recursive after initialization
        if i > 0:
            v_x             = v_hat_x[i-1]
            v_y             = v_hat_y[i-1]
            sigma_v_k_m     = sigma_v_k[i-1].copy()
            deltaT = stateTime[i] - stateTime[i-1]

        # IMU Data
        a_x   = imuData[index_from_time(t, imuTime), VESC_HEADER.index('ax')]
        a_y   = imuData[index_from_time(t, imuTime), VESC_HEADER.index('ay')]
        ohm_z = imuData[index_from_time(t, imuTime), VESC_HEADER.index('wz')]

        # Command input to VESC
        v_x_m = cmdData[index_from_time(t, cmdTime), ACKR_HEADER.index('V')]

        # EKF Update Step
        v_hat_x[i], v_hat_y[i], sigma_v_k[i] = TRFC_Filter.estimate_velocities(deltaT, v_x, v_y, a_x, a_y, ohm_z, v_x_m, sigma_v_k_m)

    plt.plot(stateTime)

    plt.figure()
    v_x =  pfOdom[:, ODOM_HEADER.index('vx')]
    plt.subplot(2,1,1)
    plt.plot(v_x, c='green')
    plt.subplot(2,1,2)
    plt.plot(v_hat_x, c='red')
    plt.title("Vx")

    plt.figure()
    v_x =  pfOdom[:, ODOM_HEADER.index('vx')]
    plt.plot(v_x, c='green')
    plt.plot(v_hat_x, c='red')
    plt.title("Vx Superimposed")

    plt.figure()
    v_y =  pfOdom[:, ODOM_HEADER.index('vy')]
    plt.subplot(2,1,1)
    plt.plot(v_y, c='green')
    plt.subplot(2,1,2)
    plt.plot(v_hat_y, c='red')
    plt.title("Vy")

    plt.figure()
    v_y =  pfOdom[:, ODOM_HEADER.index('vy')]
    plt.plot(v_y, c='green')
    plt.plot(v_hat_y, c='red')
    plt.title("Vy Superimposed")
    plt.show()
    
    x_loc   = pfPose[:, 2]
    y_loc   = pfPose[:, 3]
    plt.figure()
    plt.scatter(x_loc, y_loc)
    plt.show()
    print("HELLO THERE")
    