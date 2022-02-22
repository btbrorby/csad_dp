#!/usr/bin/env python
from operator import imul
from platform import node
import numpy as np
import matplotlib.pyplot as plt
from numpy import size
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3




class IMU():
    """
    acc_tilde = acc + bias + whiteNoise
    """
    def __init__(self, dt, id):
        self.topic = '/imu' + str(id)
        self.dt = dt
        self.nu = np.zeros(6)
        self.nu_previous = np.zeros(6)
        
        self.nu_d = np.zeros(6)
        self.nu_d_previous = np.zeros(6)
        
        self.bias = np.zeros(6)
        self.bias_d = np.zeros(6)
        self.noiseBias = np.zeros(6)
        self.noiseMeasurement = np.zeros(6)
        
        self.mean = 0
        self.std = 0.1
        
        #For plotting:
        self.biasVec = np.array()
        
        
        
        #ROS related:
        self.msg_odometry = Odometry()
        
        self.msg_imu = Imu()
        self.pub_imu = rospy.Publisher(self.topic, Imu, queue_size=1)
    
    def updateOdometry(self, data):
        self.msg_odometry = data
    
    def getOfom(self):
        self.nu = self.msg_odometry.pose
        
        
    
    
    def plotIMU(self):
        """"Plot measurment"""
        
        plt.plot(self.biasVec) #Modify
        plt.pause(0.01)
        
        
        
    def getBias(self, dt):
        """Bias term"""
        self.noiseBias = np.random.normal(self.mean, self.std, 1)
        self.bias_d = self.noiseBias
        self.bias = self.bias + self.bias_d*dt
        return self.bias
        
        
    def getAccMeasured(self, dt):
        """Outputs the simulated acc measurement"""
        self.noiseMeasurement = np.random.normal(self.mean, self.std, 1)
        self.getBias(dt)
        return self.nu_d + self.bias + self.noiseMeasurement
    
    
        


"""For testing"""
imu = IMU(0.0, 0.01, 1)
i = 0
fig = plt.figure()
while i < 1000:
    imu.biasVec = np.append(imu.biasVec, imu.getAccMeasured(0.01))
    if np.size(imu.biasVec) > 100:
        imu.biasVec = np.delete(imu.biasVec, 0)
        plt.clf()
    i += 1
    print(imu.biasVec)
    imu.plotIMU()

plt.show()



imu = IMU(0.01, 1)


def sensorNodeInit():
    global node
    node = rospy.init_node("Sensor_node")
    rospy.Subscriber('/qualisys/Body_1/odom', Odometry, imu.updateOdometry)
    


def loop():
    
    return 0