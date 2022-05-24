#!/usr/bin/env python
from pickletools import float8
from time import sleep
from matplotlib import pyplot as plt
import os
from tables import Float32Col
import rosbag
from rospy import Time
import numpy as np
from scipy import signal
from decimal import Decimal

import rospy


def quat2eul(w, x, y, z):
    """
    Returns the ZYX roll-pitch-yaw angles from a quaternion.
    """
    q = np.array((w, x, y, z))
    #if np.abs(np.linalg.norm(q) - 1) > 1e-6:
    #   raise RuntimeError('Norm of the quaternion must be equal to 1')

    eta = q[0]
    eps = q[1:]

    S = np.array([
        [0, -eps[2], eps[1]],
        [eps[2], 0, -eps[0]],
        [-eps[1], eps[0], 0]
    ])

    R = np.eye(3) + 2 * eta * S + 2 * np.linalg.matrix_power(S, 2)

    if np.abs(R[2, 0]) > 1.0:
        raise RuntimeError('Solution is singular for pitch of +- 90 degrees')

    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = -np.arcsin(R[2, 0])
    yaw = np.arctan2(R[1, 0], R[0, 0])

    return np.array([roll, pitch, yaw])



fileName = 'testbag.bag'
pathName = "{0}/csad_dp_ws/bags/"+fileName

path = os.path.dirname(os.getcwd())
path = pathName.format(path)
bag = rosbag.Bag(path)

title=fileName[0:-4]


#Initialization:
time = []
x = []
y = []
z = []
roll = []
pitch = []
yaw = []
x_dot = []
y_dot = []
z_dot = []
roll_dot = []
pitch_dot = []
yaw_dot = []
biasX = []
biasY = []
biasYaw = []

time_hat = []
x_hat = []
y_hat = []
yaw_hat = []
x_dot_hat = []
y_dot_hat = []
yaw_dot_hat = []
bias_x_hat = []
bias_y_hat = []
bias_yaw_hat = []

waveElevation = []
waveLoadX = []
waveLoadY = []
waveLoadN = []
timeWave = []

imu1_x = []
imu1_y = []
imu1_z = []
imu2_x = []
imu2_y = []
imu2_z = []
imu3_x = []
imu3_y = []
imu3_z = []
imu4_x = []
imu4_y = []
imu4_z = []
timeImu1 = []
timeImu2 = []
timeImu3 = []
timeImu4 = []

tauX = []
tauY = []
tauN = []
timeTau = []

trueTauX = []
trueTauY = []
trueTauN = []

u1 = []
u2 = []
u3 = []
u4 = []
u5 = []
u6 =[]
alpha1 = []
alpha2 = []
alpha3 = []
alpha4 = []
alpha5 = []
alpha6 = []
timeU = []

controlHelper1 = []
controlHelper2 = []
controlHelper3 = []
timeHelper = []


i = 0
initialTime = bag.get_start_time()
topicsAvailable = bag.get_type_and_topic_info()
for topic, msg, t in bag.read_messages(topics={'/qualisys/Body_1/odom',
                                               '/CSAD/state_estimate', 
                                               '/waveElevation',
                                               '/waveLoads',
                                               '/imu1',
                                               '/imu2',
                                               '/imu3',
                                               '/imu4',
                                               '/CSAD/tau',
                                               '/CSAD/u',
                                               '/CSAD/trueThrustLoads',
                                               '/helper'}):
    
    if topic == '/qualisys/Body_1/odom':
        T = Time(t.secs, t.nsecs)
        time.append(T.to_sec()-initialTime)
        x.append(msg.pose.pose.position.x)
        y.append(msg.pose.pose.position.y)
        z.append(msg.pose.pose.position.z)
        euler = quat2eul(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
        roll.append(euler[0])
        pitch.append(euler[1])
        yaw.append(euler[2])
        
        x_dot.append(msg.twist.twist.linear.x)
        y_dot.append(msg.twist.twist.linear.y)
        z_dot.append(msg.twist.twist.linear.z)
        roll_dot.append(msg.twist.twist.angular.x)
        pitch_dot.append(msg.twist.twist.angular.y)
        yaw_dot.append(msg.twist.twist.angular.z)
        
    elif topic == '/CSAD/state_estimate':
        T = Time(msg.secs, msg.nsecs)
        time_hat.append(T.to_sec() - initialTime)
        x_hat.append(msg.eta[0])
        y_hat.append(msg.eta[1])
        yaw_hat.append(msg.eta[2])
        x_dot_hat.append(msg.nu[0])
        y_dot_hat.append(msg.nu[1])
        yaw_dot_hat.append(msg.nu[2])
        bias_x_hat.append(msg.bias[0])
        bias_y_hat.append(msg.bias[1])
        bias_yaw_hat.append(msg.bias[2])
    
    elif topic == '/waveElevation':
        waveElevation.append(msg.elevation)
        Tp = msg.Tp
        Hs = msg.Hs
        title = 'Seastate: Tp=' + str(Tp) + ' s , Hs=' + str(Hs) + ' m'
        
    elif topic =='/waveLoads':
        timeWave.append(msg.data[0])
        waveLoadX.append(msg.data[1])
        waveLoadY.append(msg.data[2])
        waveLoadN.append(msg.data[6])
        
    
    elif topic == '/imu1':
        imu1_x.append(msg.linear_acceleration.x)
        imu1_y.append(msg.linear_acceleration.y)
        imu1_z.append(msg.linear_acceleration.z)
        T = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
        timeImu1.append(T.to_sec()-initialTime)
    elif topic == '/imu2':
        imu2_x.append(msg.linear_acceleration.x)
        imu2_y.append(msg.linear_acceleration.y)
        imu2_z.append(msg.linear_acceleration.z)
        T = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
        timeImu2.append(T.to_sec()-initialTime)
    elif topic == '/imu3':
        imu3_x.append(msg.linear_acceleration.x)
        imu3_y.append(msg.linear_acceleration.y)
        imu3_z.append(msg.linear_acceleration.z)
        T = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
        timeImu3.append(T.to_sec()-initialTime)
    elif topic == '/imu4':
        imu4_x.append(msg.linear_acceleration.x)
        imu4_y.append(msg.linear_acceleration.y)
        imu4_z.append(msg.linear_acceleration.z)
        T = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
        timeImu4.append(T.to_sec()-initialTime)
    
    elif topic == '/CSAD/tau':
        tauX.append(msg.data[0])
        tauY.append(msg.data[1])
        tauN.append(msg.data[2])
        timeTau.append(msg.data[3])
        
    elif topic == '/CSAD/u':
        
        u1.append(msg.data[0])
        u2.append(msg.data[1])
        u3.append(msg.data[2])
        u4.append(msg.data[3])
        u5.append(msg.data[4])
        u6.append(msg.data[5])
        alpha1.append(msg.data[6])
        alpha2.append(msg.data[7])
        alpha3.append(msg.data[8])
        alpha4.append(msg.data[9])
        alpha5.append(msg.data[10])
        alpha6.append(msg.data[11])
        T = Time(int(msg.data[-2]), int(msg.data[-1]))
        timeU.append(T.to_sec()-initialTime)
        
    
        
    elif topic == '/CSAD/trueThrustLoads':
        trueTauX.append(msg.data[0])
        trueTauY.append(msg.data[1])
        trueTauN.append(msg.data[2])
        
    elif topic == '/helper':
        controlHelper1.append(msg.data[0])
        controlHelper2.append(msg.data[1])
        controlHelper3.append(msg.data[2])
        timeHelper.append(msg.data[-1])
        

    
"""Plotting states"""
fig0, ax0 = plt.subplots(3,1)
fig0.suptitle(title)
for ax in ax0:
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(0,0))
ax0[0].plot(time, x, label='x')
ax0[0].grid()
ax0[0].set(ylabel='[m]')
ax0[1].plot(time, y, label='y')
ax0[1].grid()
ax0[1].set(ylabel='[m]')
ax0[2].plot(time, z, label='yaw')
ax0[2].grid()
ax0[2].set(ylabel='[rad]')
ax0[2].set(xlabel='time [s]')
for ax in ax0:
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(0,0))
    ax.legend(loc='upper right')


fig1, ax1 = plt.subplots(3,1)
fig1.suptitle(title)
ax1[0].plot(time, roll, label='roll')
ax1[0].grid()
ax1[0].set(ylabel='[rad]')
ax1[1].plot(time, pitch, label='pitch')
ax1[1].grid()
ax1[1].set(ylabel='[rad]')
ax1[2].plot(time, yaw, label='yaw')
ax1[2].grid()
ax1[2].set(ylabel='[rad]')
ax1[2].set(xlabel='time [s]')
for ax in ax1:
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(0,0))
    ax.legend(loc='upper right')



"""Plotting Observer states"""
fig2, ax2 = plt.subplots(3,2)
fig2.suptitle(title)
ax2[0,0].plot(time, x, label='x')
ax2[0,0].plot(time_hat, x_hat, 'r--', label='x_hat')
ax2[0,0].grid()
ax2[0,0].set(ylabel='[m]')
ax2[1,0].plot(time, y, label='y')
ax2[1,0].plot(time_hat, y_hat, 'r--', label='y_hat')
ax2[1,0].grid()
ax2[1,0].set(ylabel='[m]')
ax2[2,0].plot(time, yaw, label='yaw')
ax2[2,0].plot(time_hat, yaw_hat, 'r--', label='yaw_hat')
ax2[2,0].grid()
ax2[2,0].set(ylabel='[rad]')
ax2[2,0].set(xlabel='time [s]')
for ax in ax2[:,0]:
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(0,0))
    ax.legend(loc='lower right')
ax2[0,1].plot(time, x_dot, label='u')
ax2[0,1].plot(time_hat, x_dot_hat, 'r--', label='u_hat')
ax2[0,1].grid()
ax2[0,1].set(ylabel='[m/s]')
ax2[1,1].plot(time, y_dot, label='v')
ax2[1,1].plot(time_hat, y_dot_hat, 'r--', label='v_hat')
ax2[1,1].grid()
ax2[1,1].set(ylabel='[m/s]')
ax2[2,1].plot(time, yaw_dot, label='r')
ax2[2,1].plot(time_hat, yaw_dot_hat, 'r--', label='r_hat')
ax2[2,1].grid()
ax2[2,1].set(ylabel='[rad/s]')
ax2[2,1].set(xlabel='time [s]')
for ax in ax2[:,1]:
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(0,0))
    ax.legend(loc='upper right')


"""Plotting wave elevation"""
fig3, ax3 = plt.subplots(2,2)
fig3.suptitle(title)
ax3[0,0].plot(timeWave[0:], waveElevation[0:], label="Wave elevation")
ax3[0,0].set(ylabel='[m]')
ax3[0,0].grid()
ax3[1,0].plot(timeWave, waveLoadX, label="Wave load X")
ax3[1,0].set(ylabel='[N]')
ax3[1,0].grid()
ax3[1,0].set(xlabel='time [s]')
for ax in ax3[:,0]:
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(0,0))
    ax.legend(loc='upper right')

ax3[0,1].plot(timeWave, waveLoadY, label="Wave load Y")
ax3[0,1].set(ylabel='[N]')
ax3[0,1].grid()
ax3[1,1].plot(timeWave, waveLoadN, label="Wave load N")
ax3[1,1].set(ylabel='[N]')
ax3[1,1].grid()
ax3[1,1].set(xlabel='time [s]')
for ax in ax3[:,1]:
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(0,0))
    ax.legend(loc='upper right')

"""Plotting controller output"""
fig, ax4 = plt.subplots(3,1)
ax4[0].plot(timeTau, tauX, label="tau_thr X")
ax4[0].plot(timeTau, tauY, label="tau_thr Y")
ax4[0].plot(timeTau, tauN, label="tau_thr N")
ax4[0].grid()
# print(trueTauX)
ax4[1].plot(timeU, u1, label="u1")
ax4[1].plot(timeU, u2, label="u2")
ax4[1].plot(timeU, u3, label="u3")
ax4[1].plot(timeU, u4, label="u4")
ax4[1].plot(timeU, u5, label="u5")
ax4[1].plot(timeU, u6, label="u6")
ax4[1].axhline(y=0.5, color='k', linestyle='--')
ax4[1].grid()
# ax4[1].plot(timeU, np.sum((u1, u2, u3, u4, u5, u6), axis=0))
ax4[2].plot(timeU, alpha1, label="alpha1")
ax4[2].plot(timeU, alpha2, label="alpha2")
ax4[2].plot(timeU, alpha3, label="alpha3")
ax4[2].plot(timeU, alpha4, label="alpha4")
ax4[2].plot(timeU, alpha5, label="alpha5")
ax4[2].plot(timeU, alpha6, label="alpha6")
ax4[2].grid()
for ax in ax4:
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(0,0))
    ax.legend(loc='upper right')


[f, sp] = signal.welch(waveElevation, fs=(timeWave[0]-timeWave[-1]/len(timeWave)))
# f = np.fft.rfftfreq(len(waveElevation), d=(timeWave[0]-timeWave[-1]/len(timeWave)))
print(type(sp[0]))
print(type(f[0]))
fig, ax = plt.subplots(2,1)
ax[0].plot(f, np.abs(sp))

[f, sp] = signal.welch(waveLoadX, fs=(timeWave[0]-timeWave[-1]/len(timeWave)))
ax[1].plot(f, np.abs(sp))

fig5, ax5 = plt.subplots(3,1)
ax5[0].plot(timeImu1, imu1_x)
ax5[0].plot(timeImu2, imu2_x)
ax5[0].plot(timeImu3, imu3_x)
ax5[0].plot(timeImu4, imu4_x)
ax5[0].grid()
ax5[0].plot(np.add(timeTau, timeImu1[0]-timeTau[0]), tauX)

ax5[1].plot(timeImu1, imu1_y)
ax5[1].plot(timeImu2, imu2_y)
ax5[1].plot(timeImu3, imu3_y)
ax5[1].plot(timeImu4, imu4_y)
ax5[1].grid()
ax5[1].plot(np.add(timeTau,timeImu1[0]), tauY)

ax5[2].plot(timeImu1, imu1_z)
ax5[2].plot(timeImu2, imu2_z)
ax5[2].plot(timeImu3, imu3_z)
ax5[2].plot(timeImu4, imu4_z)
ax5[2].grid()

fig6, ax6 = plt.subplots(3,1)
ax6[0].plot(timeHelper, controlHelper1)
ax6[1].plot(timeHelper, controlHelper2)
ax6[2].plot(timeHelper, controlHelper3)






plt.show()
    
    

    
bag.close()
