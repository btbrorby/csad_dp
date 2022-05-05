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

class Visualize():
    def __init__(self):
        
        self.fig, self.ax = plt.subplots(3,1)
        self.x = [0.0]
        self.y = [0.0]
        self.xFinal = [0.0]
        self.yFinal = [0.0]
        self.line, = self.ax[0].plot(self.x, self.y)
        self.x_hat = [0.0]
        self.y_hat = [0.0]
        self.x_hatFinal = [0.0]
        self.y_hatFinal = [0.0]
        self.timeInitial = rospy.Time.now()
        self.time = 0.0
        self.timeVec = [0.0]
        self.timeVecFinal = [0.0]
        self.time_hat = 0.0
        self.timeVec_hat = [0.0]
        self.timeVecFinal_hat = [0.0]
        
    def plot_init(self):
        self.ax[0].set_xlim(-50, 50)
        self.ax[0].set_ylim(-50, 50)
        
        # self.fig.ion()
        # self.fig.show()
        plt.ion()
        plt.show()
        
    def odomCallback(self, msg=Odometry()):
        self.x.append(msg.pose.pose.position.x)
        self.y.append(msg.pose.pose.position.y)
        self.xFinal.append(msg.pose.pose.position.x)
        self.yFinal.append(msg.pose.pose.position.y)
        self.time = msg.header.stamp.to_sec() - self.timeInitial.to_sec()
        self.timeVec.append(self.time)
        self.timeVecFinal.append(self.time)
        if len(self.x) > 100:
            self.x.pop(0)
            self.y.pop(0)
            self.timeVec.pop(0)
            
        
        
    def observerCallback(self, msg=observer_message()):
        self.x_hat.append(msg.eta[0])
        self.y_hat.append(msg.eta[1])
        self.x_hatFinal.append(msg.eta[0])
        self.y_hatFinal.append(msg.eta[1])
        self.timeVec_hat.append(msg.time)
        self.timeVecFinal_hat.append(msg.time)
        if len(self.x_hat) > 100:
            self.x_hat.pop(0)
            self.y_hat.pop(0)
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
        
    fig, ax = plt.subplots(3,1)
    ax[0].plot(vis.xFinal, vis.yFinal)
    ax[1].plot(vis.timeVecFinal, vis.xFinal)
    ax[1].plot(vis.timeVecFinal_hat, vis.x_hatFinal, color="red")
    ax[2].plot(vis.timeVecFinal, vis.yFinal)
    ax[2].plot(vis.timeVecFinal_hat, vis.y_hatFinal, color="red")
    plt.show()
    plt.pause(20)
    