#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 15 18:18:24 2017

@author: hyj
"""
import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

np.set_printoptions(suppress = True)
filepath = os.path.abspath('..')+"/bin"

# imu_circle   imu_spline
position = []
quaterntions = []
timestamp = []
tx_index = 5
position = np.loadtxt(filepath + '/imu_pose.txt', usecols = (tx_index, tx_index + 1, tx_index + 2))

# imu_pose   imu_spline
position1 = []
quaterntions1 = []
timestamp1 = []
data = np.loadtxt(filepath + '/imu_int_pose.txt')
# timestamp1 = data[:,0]
# quaterntions1 = data[:,[tx_index + 6, tx_index + 3, tx_index + 4, tx_index + 5]] # qw,qx,qy,qz
position1 = data[:,[tx_index, tx_index + 1, tx_index + 2]]

# cam_pose_opt_o_0   cam_pose_opt_o_0
position2 = []
quaterntions2 = []
timestamp2 = []
data = np.loadtxt(filepath + '/imu_int_pose_noise.txt')
# timestamp2 = data[:,0]
# quaterntions2 = data[:,[tx_index + 6, tx_index + 3, tx_index + 4, tx_index + 5]] # qw,qx,qy,qz
position2 = data[:,[tx_index, tx_index + 1, tx_index + 2]]


##############################################################################################################
filepath_mid = os.path.abspath('..')+"/bin2"
# imu_circle   imu_spline
position_mid = []
quaterntions_mid = []
timestamp_mid = []
tx_index = 5
position_mid = np.loadtxt(filepath_mid + '/imu_pose.txt', usecols = (tx_index, tx_index + 1, tx_index + 2))

# imu_pose   imu_spline
position1_mid = []
quaterntions1_mid = []
timestamp1_mid = []
data_mid = np.loadtxt(filepath_mid + '/imu_int_pose.txt')
# timestamp1 = data[:,0]
# quaterntions1 = data[:,[tx_index + 6, tx_index + 3, tx_index + 4, tx_index + 5]] # qw,qx,qy,qz
position1_mid = data_mid[:,[tx_index, tx_index + 1, tx_index + 2]]

# cam_pose_opt_o_0   cam_pose_opt_o_0
position2_mid = []
quaterntions2_mid = []
timestamp2_mid = []
data_mid = np.loadtxt(filepath_mid + '/imu_int_pose_noise.txt')
# timestamp2 = data[:,0]
# quaterntions2 = data[:,[tx_index + 6, tx_index + 3, tx_index + 4, tx_index + 5]] # qw,qx,qy,qz
position2_mid = data_mid[:,[tx_index, tx_index + 1, tx_index + 2]]

#################################################################################################################

### plot 3d
fig = plt.figure()
ax = fig.gca(projection='3d')

ax.plot(position[:,0], position[:,1], position[:,2], label='gt')
ax.plot(position1[:,0], position1[:,1], position1[:,2], label='imu_int')
# ax.plot(position2[:,0], position2[:,1], position2[:,2], label='noise')
ax.plot([position[0,0]], [position[0,1]], [position[0,2]], 'r.', label='start')
ax.plot(position1_mid[:,0], position1_mid[:,1], position1_mid[:,2], label='imu_mid')

ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
