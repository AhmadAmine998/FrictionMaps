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
CORE_HEADER = ['s', 'ns', 'cM', 'cI', 'DC', 'RPM', 'VI', 'ED', 'ER']

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

def padArray(A, size):
    t = size - len(A)
    if t >= 0:
        return np.pad(A, pad_width=(t, 0), mode='constant')
    else: 
        return A[:t]

Idx1 = [1000,   50,   50,   50,   50]
Idx2 = [2000, 1800, 1200, 1000, 1000]

if __name__ == '__main__':
    dataSetNumber = 5
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
    pfTime    = pfOdom[:, ODOM_HEADER.index('s')] + pfOdom[:, ODOM_HEADER.index('ns')]*10**(-9)

    if dataSetNumber == 5:
        # Motor RPM only available for dataset 5
        coreData= np.loadtxt(dataSetPath+'/coreData.csv', delimiter=',', skiprows=1)[:,1:]
        coreTime  = coreData[:, CORE_HEADER.index('s')] + coreData[:, CORE_HEADER.index('ns')]*10**(-9)
        
    # State Initialization for speed EKF
    deltaT      = 0
    v_hat_x_k_m = 0.2
    v_hat_y_k_m = 0
    sigma_v_k_m = np.diag([0.4, 0.4])

    # State Initialization for force UKF
    ohm_hat_z_k_m = 0
    F_hat_xf_k_m  = 0 
    F_hat_yf_k_m  = 0
    F_hat_yr_k_m  = 0
    sigma_f_k_m   = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

    # EKF outputs used for plotting
    v_hat_x     = np.zeros_like(stateTime)
    v_hat_y     = np.zeros_like(stateTime)
    sigma_v_k   = np.zeros((stateTime.shape[0], 2, 2))

    # Froce UKF ouputs used for plotting
    ohm_hat_z  = np.zeros_like(stateTime)
    F_hat_xf   = np.zeros_like(stateTime)
    F_hat_yf   = np.zeros_like(stateTime)
    F_hat_yr   = np.zeros_like(stateTime)
    sigma_f_k  = np.zeros((stateTime.shape[0], 6, 6))

    # State Initialization for friction UKF
    mu_hat_f_k_m  = 1
    mu_hat_r_k_m  = 1
    sigma_mu_k_m  = np.diag([10, 10])

    # Friction UKF outputs used for plotting
    mu_r_hat    = np.zeros_like(stateTime)
    mu_f_hat    = np.zeros_like(stateTime)
    sigma_mu_k  = np.zeros((stateTime.shape[0], 2, 2))

    for i, t in enumerate(stateTime):
        # Recursive after initialization
        if i > 0:
            deltaT = stateTime[i] - stateTime[i-1]

            v_hat_x_k_m = v_hat_x[i-1]
            v_hat_y_k_m = v_hat_y[i-1]
            sigma_v_k_m = sigma_v_k[i-1].copy()

            ohm_hat_z_k_m = ohm_hat_z[i-1]
            F_hat_xf_k_m  = F_hat_xf[i-1]
            F_hat_yf_k_m  = F_hat_yf[i-1]
            F_hat_yr_k_m  = F_hat_yr[i-1]
            sigma_f_k_m   = sigma_f_k[i-1].copy()

            mu_hat_f_k_m  = mu_f_hat[i-1]
            mu_hat_r_k_m  = mu_r_hat[i-1]
            sigma_mu_k_m  = sigma_mu_k[i-1].copy()

        # IMU Data
        a_x   = imuData[index_from_time(t, imuTime), VESC_HEADER.index('ax')]
        a_y   = imuData[index_from_time(t, imuTime), VESC_HEADER.index('ay')]
        ohm_z = imuData[index_from_time(t, imuTime), VESC_HEADER.index('wz')]

        # Command input to VESC
        # v_x_m = cmdData[index_from_time(t, cmdTime), ACKR_HEADER.index('V')]
        v_x_m   = pfOdom[index_from_time(t, pfTime), ODOM_HEADER.index('vx')]
        if dataSetNumber == 5:
            # Motor RPM only available for dataset 5
            v_x_m   = -(coreData[index_from_time(t, coreTime), CORE_HEADER.index('RPM')] * TRFC_Filter.tire_circ * 0.4)/(60 * TRFC_Filter.total_ratio)
        

        delta = cmdData[index_from_time(t, cmdTime), ACKR_HEADER.index('delta')]

        # EKF Update Step
        v_hat_x[i], v_hat_y[i], sigma_v_k[i] = TRFC_Filter.estimate_velocities(deltaT, v_hat_x_k_m, v_hat_y_k_m, a_x, a_y, ohm_z, v_x_m, sigma_v_k_m)

        # UKF Update 
        f_hat_k, sigma_f_k[i] = TRFC_Filter.tyreforce_estimates(v_hat_x[i], v_hat_y[i], ohm_hat_z_k_m, F_hat_xf_k_m, F_hat_yf_k_m, F_hat_yr_k_m, sigma_f_k_m, v_hat_x[i], ohm_z, v_hat_y[i], a_x, a_y, delta, deltaT)
        
        # Extract states from state vector
        v_hat_x[i]   = f_hat_k[0]
        v_hat_y[i]   = f_hat_k[1]
        ohm_hat_z[i] = f_hat_k[2]
        F_hat_xf[i]  = -f_hat_k[3]/100
        F_hat_yf[i]  = -f_hat_k[4]/100
        F_hat_yr[i]  = -f_hat_k[5]/100
        
        #UKF Update 
        mu_hat_k, sigma_mu_k[i] = TRFC_Filter.TRFC_estimation(np.array([[mu_hat_f_k_m],[mu_hat_r_k_m]]), 
                                                              np.array([[F_hat_yf[i]],[F_hat_yr[i]]]), sigma_mu_k_m,  
                                                              v_hat_y[i], v_hat_x[i], ohm_hat_z[i], a_x, delta)
                
        # Extract states from state vector
        mu_f_hat[i]   = mu_hat_k[0]
        mu_r_hat[i]   = mu_hat_k[1]               
                                                            
    stateTime = stateTime - stateTime[0]
    
    plt.figure()
    v_x =  pfOdom[:, ODOM_HEADER.index('vx')]
    v_x = padArray(v_x, stateTime.shape[0])
    plt.plot(stateTime, v_x, c='green', label='PF Vx')
    plt.plot(stateTime, v_hat_x, c='red', label='v_hat_x')
    plt.title("Vx Superimposed")
    plt.ylabel("Velocity (m/s)")
    plt.xlabel("Time (s)")
    plt.xlim(stateTime[0], stateTime[-1])
    plt.legend()

    plt.figure()
    v_y = pfOdom[:, ODOM_HEADER.index('vy')]
    v_y = padArray(v_y, stateTime.shape[0])
    plt.plot(stateTime, v_y, c='green', label='PF Vy')
    plt.plot(stateTime, v_hat_y, c='red', label='v_hat_y')
    plt.title("Vy Superimposed")
    plt.ylabel("Velocity (m/s)")
    plt.xlabel("Time (s)")
    plt.xlim(stateTime[0], stateTime[-1])
    plt.legend()
    plt.show()
    
    plt.figure()
    ohm_z =  imuData[:, VESC_HEADER.index('wz')]
    ohm_z = padArray(ohm_z, stateTime.shape[0])
    plt.plot(stateTime, ohm_z, c='green', label='IMU ohm_z')
    plt.plot(stateTime, ohm_hat_z, c='red', label='ohm_hat_z')
    plt.title("Wz Superimposed")
    plt.ylabel("Angular Frequency (rad/s)")
    plt.xlabel("Time (s)")
    plt.xlim(stateTime[0], stateTime[-1])
    plt.legend()
    # plt.show()

    plt.figure()
    a_x = imuData[:, VESC_HEADER.index('ax')]
    a_x = padArray(a_x, stateTime.shape[0])
    plt.plot(stateTime, a_x, c='green', label='IMU ax')
    plt.plot(stateTime, F_hat_xf, c='red', label='F_hat_xf')
    plt.ylabel("Acceleration (m/s/s)")
    plt.xlabel("Time (s)")
    plt.xlim(stateTime[0], stateTime[-1])
    plt.legend()
    plt.title("ax Superimposed")
    # plt.show()

    plt.figure()
    a_y = imuData[:, VESC_HEADER.index('ay')]
    a_y = padArray(a_y, stateTime.shape[0])
    plt.plot(stateTime, a_y, c='green', label='IMU ay')
    plt.plot(stateTime, F_hat_yf, c='red', label='F_hat_yf')
    plt.plot(stateTime, F_hat_yr, c='blue', label='F_hat_yr')
    plt.ylabel("Acceleration (m/s/s)")
    plt.xlabel("Time (s)")
    plt.xlim(stateTime[0], stateTime[-1])
    plt.legend()
    plt.title("ay Superimposed")
    plt.show()

    plt.plot(stateTime, mu_r_hat, label='mu_r_hat')
    plt.plot(stateTime, mu_f_hat, label='mu_f_hat')
    plt.plot(stateTime, 0.65*np.ones_like(stateTime), c='green', label='mu concrete')
    plt.ylabel("Friction Coefficient")
    plt.xlabel("Time (s)")
    plt.xlim(stateTime[0], stateTime[-1])
    plt.legend()
    plt.title("mu Superimposed")
    plt.show()

    #  
    try:
        idx1 = Idx1[dataSetNumber - 1]
        idx2 = Idx2[dataSetNumber - 1]
    except:
        print("WARNING! Custom dataset detected. Make sure friction localization plots are satisfactory. Otherwise add your slices to Idx1 and Idx2")
        idx1 = 50
        idx2 = -1

    x_loc   = pfPose[idx1:idx2, 2]
    y_loc   = pfPose[idx1:idx2, 3]
    c = mu_r_hat[idx1:idx2]
    c = (c - np.min(c))
    c /= np.max(c)
    plt.figure()
    c = 1-c
    plt.scatter(x_loc, y_loc, c=c)
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Localized Scaled Friction")
    plt.colorbar()
    plt.show()