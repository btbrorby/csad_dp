#!/usr/bin/env python
from click import style
from matplotlib import animation
from matplotlib.animation import FuncAnimation
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from messages.msg import observer_message
import matplotlib.pyplot as plt
import numpy as np

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


class Visualize():
    def __init__(self):
        #ETA
        self.fig, self.ax = plt.subplots(4,1)
        self.x = [0.0]
        self.y = [0.0]
        self.psi = [0.0]
        self.xFinal = [0.0]
        self.yFinal = [0.0]
        self.psiFinal = [0.0]
        self.line, = self.ax[0].plot(self.x, self.y)
        self.x_hat = [0.0]
        self.y_hat = [0.0]
        self.psi_hat = [0.0]
        self.x_hatFinal = [0.0]
        self.y_hatFinal = [0.0]
        self.psi_hatFinal = [0.0]
        
        #NU
        self.figNu, self.axNu = plt.subplots(4,1)
        self.Nux = [0.0]
        self.Nuy = [0.0]
        self.Nur = [0.0]
        self.NuxFinal = [0.0]
        self.NuyFinal = [0.0]
        self.NurFinal = [0.0]
        self.Nux_hat = [0.0]
        self.Nuy_hat = [0.0]
        self.Nur_hat = [0.0]
        self.Nux_hatFinal = [0.0]
        self.Nuy_hatFinal = [0.0]
        self.Nur_hatFinal = [0.0]
        
        
        self.timeInitial = rospy.Time.now()
        self.time = 0.0
        self.timeVec = [0.0]
        self.timeVecFinal = [0.0]
        self.time_hat = 0.0
        self.timeVec_hat = [0.0]
        self.timeVec_hatFinal = [0.0]
        
    def plot_init(self):
        self.ax[0].set_xlim(-50, 50)
        self.ax[0].set_ylim(-50, 50)
        
        self.axNu[0].set_xlim(-50, 50)
        self.axNu[0].set_ylim(-50, 50)
        
        # self.fig.ion()
        # self.fig.show()
        plt.ion()
        plt.show()
        
    def odomCallback(self, msg=Odometry()):
        
        
        #ETA
        yaw = quat2eul(msg.pose.pose.orientation.w,
                       msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z)[2]
        
        self.x.append(msg.pose.pose.position.x)
        self.y.append(msg.pose.pose.position.y)
        self.psi.append(yaw)
        self.xFinal.append(msg.pose.pose.position.x)
        self.yFinal.append(msg.pose.pose.position.y)
        self.psiFinal.append(yaw)
        self.time = msg.header.stamp.to_sec() - self.timeInitial.to_sec()
        self.timeVec.append(self.time)
        self.timeVecFinal.append(self.time)
        
        #NU
        self.Nux.append(msg.twist.twist.linear.x)
        self.Nuy.append(msg.pose.pose.position.y)
        self.Nur.append(msg.twist.twist.angular.z)
        self.NuxFinal.append(msg.twist.twist.linear.x)
        self.NuyFinal.append(msg.twist.twist.linear.y)
        self.NurFinal.append(msg.twist.twist.angular.z)
        
        if len(self.x) > 100:
            self.x.pop(0)
            self.y.pop(0)
            self.psi.pop(0)
            self.Nux.pop(0)
            self.Nuy.pop(0)
            self.Nur.pop(0)
            self.timeVec.pop(0)
            
        
        
    def observerCallback(self, msg=observer_message()):
        #ETA
        self.x_hat.append(msg.eta[0])
        self.y_hat.append(msg.eta[1])
        self.psi_hat.append(msg.eta[2])
        self.x_hatFinal.append(msg.eta[0])
        self.y_hatFinal.append(msg.eta[1])
        self.psi_hatFinal.append(msg.eta[2])
        self.timeVec_hat.append(self.time)
        self.timeVec_hatFinal.append(self.time)
        
        #NU
        self.Nux_hat.append(msg.nu[0])
        self.Nuy_hat.append(msg.nu[1])
        self.Nur_hat.append(msg.nu[2])
        self.Nux_hatFinal.append(msg.nu[0])
        self.Nuy_hatFinal.append(msg.nu[1])
        self.Nur_hatFinal.append(msg.nu[2])
        
        
        if len(self.x_hat) > 100:
            self.x_hat.pop(0)
            self.y_hat.pop(0)
            self.psi_hat.pop(0)
            self.Nux_hat.pop(0)
            self.Nuy_hat.pop(0)
            self.Nur_hat.pop(0)
            self.timeVec_hat.pop(0)
            
        
        
        
    
        
    def updateXYplot(self):
        
        if len(self.x) >= 100:
            self.ax[0].cla()
            self.ax[0].plot(self.x, self.y, color="blue")
            self.ax[0].plot(self.x_hat, self.y_hat, color="red")
            self.ax[1].cla()
            self.ax[1].plot(self.timeVec, self.x, color="blue")
            self.ax[1].plot(self.timeVec, self.x_hat, color="red")
            self.ax[2].cla()
            self.ax[2].plot(self.timeVec, self.y, color="blue")
            self.ax[2].plot(self.timeVec, self.y_hat, color="red")
            self.ax[3].cla()
            self.ax[3].plot(self.timeVec, self.psi, color="blue")
            self.ax[3].plot(self.timeVec, self.psi_hat, color="red")
            plt.draw()
            plt.pause(0.0001)
            
            self.axNu[0].cla()
            self.axNu[0].plot(self.Nux, self.Nuy, color="blue")
            self.axNu[0].plot(self.Nux_hat, self.Nuy_hat, color="red")
            self.axNu[1].cla()
            self.axNu[1].plot(self.timeVec, self.Nux, color="blue")
            self.axNu[1].plot(self.timeVec, self.Nux_hat, color="red")
            self.axNu[2].cla()
            self.axNu[2].plot(self.timeVec, self.Nuy, color="blue")
            self.axNu[2].plot(self.timeVec, self.Nuy_hat, color="red")
            self.axNu[3].cla()
            self.axNu[3].plot(self.timeVec, self.Nur, color="blue")
            self.axNu[3].plot(self.timeVec, self.Nur_hat, color="red")
            plt.draw()
            plt.pause(0.0001)
        # else:
        #     self.ax[0].scatter(self.x[-1], self.y[-1], color="blue")
        #     self.ax[0].scatter(self.x_hat[-1], self.y_hat[-1], color="red")
        #     self.ax[1].scatter(self.timeVec[-1], self.x[-1], color="blue")
        #     self.ax[1].scatter(self.timeVec[-1], self.x_hat[-1], color="red")
        #     self.ax[2].scatter(self.timeVec[-1], self.y[-1], color="blue")
        #     self.ax[2].scatter(self.timeVec[-1], self.y_hat[-1], color="red")
        #     plt.draw()
        #     plt.pause(0.0001)
        
    
    
        
        
        
        
        
    

if __name__ == '__main__':
    
    node = rospy.init_node('Visualize_node')
    
    vis = Visualize()
    odomSub = rospy.Subscriber("/qualisys/Body_1/odom", Odometry, vis.odomCallback)
    observerSub = rospy.Subscriber("/CSAD/state_estimate", observer_message, vis.observerCallback)
    vis.plot_init()
    while not rospy.is_shutdown():
        vis.updateXYplot()
        # vis.updateXTplot()
        # plt.draw()
        
    fig, ax = plt.subplots(4,1)
    ax[0].plot(vis.xFinal, vis.yFinal)
    ax[0].plot(vis.x_hatFinal, vis.y_hatFinal, color="red")
    ax[1].plot(vis.timeVecFinal, vis.xFinal)
    ax[1].plot(vis.timeVec_hatFinal, vis.x_hatFinal, color="red")
    ax[2].plot(vis.timeVecFinal, vis.yFinal)
    ax[2].plot(vis.timeVec_hatFinal, vis.y_hatFinal, color="red")
    ax[3].plot(vis.timeVecFinal, vis.psiFinal)
    ax[3].plot(vis.timeVec_hatFinal, vis.psi_hatFinal, color="red")
    
    fig, ax = plt.subplots(4,1)
    ax[0].plot(vis.NuxFinal, vis.NuyFinal)
    ax[0].plot(vis.Nux_hatFinal, vis.Nuy_hatFinal, color="red")
    ax[1].plot(vis.timeVecFinal, vis.NuxFinal)
    ax[1].plot(vis.timeVec_hatFinal, vis.Nux_hatFinal, color="red")
    ax[2].plot(vis.timeVecFinal, vis.NuyFinal)
    ax[2].plot(vis.timeVec_hatFinal, vis.Nuy_hatFinal, color="red")
    ax[3].plot(vis.timeVecFinal, vis.psiFinal)
    ax[3].plot(vis.timeVec_hatFinal, vis.Nur_hatFinal, color="red")
    plt.show()
    plt.pause(30)
    