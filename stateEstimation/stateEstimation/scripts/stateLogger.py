#!/usr/bin/env python3
"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""

import numpy as np
from scipy import signal
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
import os
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from vesc_msgs.msg import VescImuStamped, VescStateStamped
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from datetime import datetime
import pandas as pd

POSE_HEADER = ['s','ns','x','y','q.x','q.y','q.z','q.w']
VESC_HEADER = ['s','ns','phi_r','phi_p', 'phi_y', 'ax', 'ay', 'az', 'wx', 'wy', 'wz', 'q.x','q.y','q.z','q.w']
ACKR_HEADER = ['s','ns','V','delta']
ODOM_HEADER = ['s', 'ns', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'x', 'y', 'q.x','q.y','q.z','q.w']
SCAN_HEADER = ['s', 'ns', 'amin', 'amax', 'ai', 'ti', 'st', 'rmin', 'rmax']
CORE_HEADER = ['s', 'ns', 'cM', 'cI', 'DC', 'RPM', 'VI', 'ED', 'ER']

r = []
for i in range(1081):
    r += ['r' + str(i)]
SCAN_HEADER += r

class stateEstimation(Node):
    def __init__(self, session):
        super().__init__('rrt_node')
        self.folderName = session
        self.pose_sub = self.create_subscription(PoseStamped, "/pf/viz/inferred_pose", self.pose_callback, 10)
        self.imu_sub  = self.create_subscription(VescImuStamped, "/sensors/imu", self.imu_callback, 10)
        self.cmd_sub  = self.create_subscription(AckermannDriveStamped, "/ackermann_cmd", self.cmd_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.pf_odom_sub = self.create_subscription(Odometry, "/pf/pose/odom", self.pf_odom_callback, 10)
        self.scansub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.coresub = self.create_subscription(VescStateStamped, "/sensors/core", self.core_callback, 10)

    def core_callback(self, core_msg):
        """
        Callback function for subscribing to VESC core sensor data.
        This funcion saves the current VESC State to a csv

        Args: 
            core_msg (VescStateStamped): incoming message from subscribed topic
        """
        header = core_msg.header
        state  = core_msg.state

        # Timestamp
        timestamp = header.stamp
        seconds   = timestamp.sec
        nanosec   = timestamp.nanosec

        # Motor Stats
        MC     = state.current_motor
        CI     = state.current_input
        DC     = state.duty_cycle
        RPM    = state.speed

        # Energy
        ED     = state.energy_drawn
        ER     = state.energy_regen
        VI     = state.voltage_input

        df = pd.DataFrame([[seconds, nanosec, MC, CI, DC, RPM, VI, ED, ER]], columns=CORE_HEADER)
        filename = 'state_logs/'+self.folderName+'/coreData.csv'
        # if file does not exist write header 
        if not os.path.isfile(filename):
            df.to_csv(filename, header=True)
        else: # else it exists so append without writing the header
            df.to_csv(filename, mode='a', header=False)
        return

    def scan_callback(self, scan_msg):
        """
        Callback function for subscribing to LIDAR's laserscan.
        This funcion saves the current scan of the lidar to a csv

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        """
        # Timestamp
        header = scan_msg.header
        timestamp = header.stamp
        seconds   = timestamp.sec
        nanosec   = timestamp.nanosec
        
        angle_min       = scan_msg.angle_min
        angle_max       = scan_msg.angle_max
        angle_increment = scan_msg.angle_increment
        time_increment  = scan_msg.time_increment
        scan_time       = scan_msg.scan_time
        range_min       = scan_msg.range_min
        range_max       = scan_msg.range_max
        ranges          = scan_msg.ranges
        
        toSave = np.hstack((seconds,
                            nanosec,
                            angle_min,
                            angle_max,
                            angle_increment,
                            time_increment,
                            scan_time,
                            range_min,
                            range_max,
                            ranges)).reshape(1,-1)
                            
        df = pd.DataFrame(toSave, columns=SCAN_HEADER)
        filename = 'state_logs/'+self.folderName+'/scanData.csv'
        # if file does not exist write header 
        if not os.path.isfile(filename):
            df.to_csv(filename, header=True)
        else: # else it exists so append without writing the header
            df.to_csv(filename, mode='a', header=False)
        return

    def pose_callback(self, pose_msg):
        """
        Callback function for subscribing to particle filter's inferred pose.
        This funcion saves the current pose of the car to a csv

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        """
        header = pose_msg.header
        pose   = pose_msg.pose

        # Timestamp
        timestamp = header.stamp
        seconds   = timestamp.sec
        nanosec   = timestamp.nanosec

        # position
        x_pos   = pose.position.x
        y_pos   = pose.position.y

        # Quaternion
        x_q     = pose.orientation.x
        y_q     = pose.orientation.y
        z_q     = pose.orientation.z
        w_q     = pose.orientation.w

        df = pd.DataFrame([[seconds, nanosec, x_pos, y_pos, x_q, y_q, z_q, w_q]], columns=POSE_HEADER)
        filename = 'state_logs/'+self.folderName+'/pfPoseData.csv'
        # if file does not exist write header 
        if not os.path.isfile(filename):
            df.to_csv(filename, header=True)
        else: # else it exists so append without writing the header
            df.to_csv(filename, mode='a', header=False)
        return

    def imu_callback(self, imu_msg):
        """
        Callback function for subscribing to VESC's imu data.
        This funcion saves the current IMU states to a csv file

        Args: 
            imu_msg (VescImuStamped): incoming message from subscribed topic
        """
        header  = imu_msg.header
        imuData = imu_msg.imu

        # Timestamp
        timestamp = header.stamp
        seconds   = timestamp.sec
        nanosec   = timestamp.nanosec

        # euler
        roll  = imuData.ypr.x
        pitch = imuData.ypr.y
        yaw   = imuData.ypr.z

        # Acceleration
        ax = imuData.linear_acceleration.x
        ay = imuData.linear_acceleration.y
        az = imuData.linear_acceleration.z

        # Angular Frequency
        wx = imuData.angular_velocity.x
        wy = imuData.angular_velocity.y
        wz = imuData.angular_velocity.z

        # Quaternion
        x_q     = imuData.orientation.x
        y_q     = imuData.orientation.y
        z_q     = imuData.orientation.z
        w_q     = imuData.orientation.w

        df = pd.DataFrame([[seconds, nanosec, roll, pitch, yaw, ax, ay, az, wx, wy, wz, x_q, y_q, z_q, w_q]], columns=VESC_HEADER)
        filename = 'state_logs/'+self.folderName+'/imuData.csv'
        # if file does not exist write header 
        if not os.path.isfile(filename):
            df.to_csv(filename, header=True)
        else: # else it exists so append without writing the header
            df.to_csv(filename, mode='a', header=False)
        return
    

    def cmd_callback(self, cmd_msg):
        """
        Callback function for subscribing to the current control input to the car.
        This function saves the current ackermann command to a csv file

        Args: 
            cmd_msg (AckermannDriveStamped): incoming message from subscribed topic
        """
        header  = cmd_msg.header
        command = cmd_msg.drive

        # Timestamp
        timestamp = header.stamp
        seconds   = timestamp.sec
        nanosec   = timestamp.nanosec

        # u(t)
        speed          = command.speed
        steering_angle = command.steering_angle

        df = pd.DataFrame([[seconds, nanosec, speed, steering_angle]], columns=ACKR_HEADER)
        filename = 'state_logs/'+self.folderName+'/commandData.csv'
        # if file does not exist write header 
        if not os.path.isfile(filename):
            df.to_csv(filename, header=True)
        else: # else it exists so append without writing the header
            df.to_csv(filename, mode='a', header=False)
        return

    
    def odom_callback(self, odom_msg):
        """
        Callback function for subscribing to particle filter's inferred pose.
        This funcion saves the current pose of the car and obtain the goal
        waypoint from the pure pursuit module.

        Args: 
            odom_msg (Odometry): incoming message from subscribed topic
        """
        header  = odom_msg.header
        pose    = odom_msg.pose.pose
        twist   = odom_msg.twist

        # Timestamp
        timestamp = header.stamp
        seconds   = timestamp.sec
        nanosec   = timestamp.nanosec

        # Linear Velocity
        vx = twist.twist.linear.x
        vy = twist.twist.linear.y
        vz = twist.twist.linear.z

        # Angular Velocity
        wx = twist.twist.linear.x
        wy = twist.twist.linear.y
        wz = twist.twist.linear.z

        # position
        x_pos   = pose.position.x
        y_pos   = pose.position.y

        # Quaternion
        x_q     = pose.orientation.x
        y_q     = pose.orientation.y
        z_q     = pose.orientation.z
        w_q     = pose.orientation.w

        df = pd.DataFrame([[seconds, nanosec, vx, vy, vz, wx, wy, wz, x_pos, y_pos, x_q, y_q, z_q, w_q]], columns=ODOM_HEADER)
        filename = 'state_logs/'+self.folderName+'/odomData.csv'
        # if file does not exist write header 
        if not os.path.isfile(filename):
            df.to_csv(filename, header=True)
        else: # else it exists so append without writing the header
            df.to_csv(filename, mode='a', header=False)
        return

    def pf_odom_callback(self, pf_odom_msg):
        """
        Callback function for subscribing to particle filter's inferred pose.
        This funcion saves the current pose of the car and obtain the goal
        waypoint from the pure pursuit module.

        Args: 
            odom_msg (Odometry): incoming message from subscribed topic
        """
        header  = pf_odom_msg.header
        pose    = pf_odom_msg.pose.pose
        twist   = pf_odom_msg.twist

        # Timestamp
        timestamp = header.stamp
        seconds   = timestamp.sec
        nanosec   = timestamp.nanosec

        # Linear Velocity
        vx = twist.twist.linear.x
        vy = twist.twist.linear.y
        vz = twist.twist.linear.z

        # Angular Velocity
        wx = twist.twist.linear.x
        wy = twist.twist.linear.y
        wz = twist.twist.linear.z

        # position
        x_pos   = pose.position.x
        y_pos   = pose.position.y

        # Quaternion
        x_q     = pose.orientation.x
        y_q     = pose.orientation.y
        z_q     = pose.orientation.z
        w_q     = pose.orientation.w

        df = pd.DataFrame([[seconds, nanosec, vx, vy, vz, wx, wy, wz, x_pos, y_pos, x_q, y_q, z_q, w_q]], columns=ODOM_HEADER)
        filename = 'state_logs/'+self.folderName+'/pfOdomData.csv'
        # if file does not exist write header 
        if not os.path.isfile(filename):
            df.to_csv(filename, header=True)
        else: # else it exists so append without writing the header
            df.to_csv(filename, mode='a', header=False)
        return

def main(args=None):
    # datetime object containing current date and time
    now = datetime.now()
    
    # dd/mm/YY H:M:S
    dt_string = now.strftime("%d-%m-%Y-%H:%M:%S")

    if not os.path.isdir("state_logs"):
        print("Logs folder not yet created. Creating...")
        os.mkdir("state_logs", 0o666)

    os.mkdir("state_logs/" + dt_string, 0o666)

    rclpy.init(args=args)
    print("stateEstimation Initialized")
    stateEstimation_node = stateEstimation(dt_string)
    rclpy.spin(stateEstimation_node)

    stateEstimation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
