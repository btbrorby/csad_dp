#!/usr/bin/env python
from cmath import pi
from turtle import width
import numpy as np
import matplotlib.pyplot as plt
import math_tools
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import yaml
import os
import time as tic

path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/simulator/src/sensorParams.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)


class GNSS():
    """
    This class simulates the GNSS signals, or the odometry given from qualisys.
    """
    def __init__(self, dt=0.02):
        self.dt = dt
        self.time = 0.0
        
        self.eta = np.zeros([6,1])
        self.nu = np.zeros([6,1])
        self.eta_dot = np.zeros([6,1])
        self.nu_dot = np.zeros([6,1])
        
        self.signal = True #only used for simulating drop out and frosen points (not in use pr. 23.02.)
        
        self.pub_gnss = rospy.Publisher('/qualisys/Body_1/odom', Odometry, queue_size=1)
        self.msg_gnss = Odometry()
        
        
    def publish(self):
        measurementGnss = self.getGnssMeasured()
        self.msg_gnss.pose.pose.position.x = measurementGnss[0]
        self.msg_gnss.pose.pose.position.y = measurementGnss[1]
        self.msg_gnss.pose.pose.position.z = measurementGnss[2]
        
        [x, y, z, w] = math_tools.euler2quat(measurementGnss[3], measurementGnss[4], measurementGnss[5])
        self.msg_gnss.pose.pose.orientation.w = w
        self.msg_gnss.pose.pose.orientation.x = x
        self.msg_gnss.pose.pose.orientation.y = y
        self.msg_gnss.pose.pose.orientation.z = z
        
        self.msg_gnss.twist.twist.linear.x = measurementGnss[6]
        self.msg_gnss.twist.twist.linear.x = measurementGnss[7]
        self.msg_gnss.twist.twist.linear.x = measurementGnss[8]
        
        self.msg_gnss.twist.twist.angular.x = measurementGnss[9]
        self.msg_gnss.twist.twist.angular.x = measurementGnss[10]
        self.msg_gnss.twist.twist.angular.x = measurementGnss[11]
        
        self.msg_gnss.header.stamp = rospy.Time.now()
        self.msg_gnss.header.frame_id = "simulated_qualisys"
        
        self.pub_gnss.publish(self.msg_gnss)
        
    def setOdometry(self, eta, nu, eta_dot, nu_dot):
        self.eta = eta
        self.nu = nu
        self.eta_dot = eta_dot
        self.nu_dot = nu_dot
        
    def generateDropOut(self):
        """This is only an example of how to give a random dropout. Currently not in use in simulator."""
        dropOut = False
        randomDropOut = np.random.normal(0, 1)
        if (randomDropOut > 1.5):
            dropOut = True
        return dropOut
    
    def getGnssMeasured(self):
        """Simulates the odometry signal by adding uncertanties, dropouts etc. Unfinnished"""
        self.time += self.dt
        output = np.concatenate(self.eta, self.nu)
        return output
            
        

class IMU():
    """
    This class simulates imu measurements located on a vessel. The location have to be specified in sensorParams.yaml
    acc_tilde = acc + bias + whiteNoise
    """
    def __init__(self, id, dt=0.02):
        self.time = 0.0
        
        self.id = id
        self.topic = '/imu' + str(id)
        self.dt = dt
        self.nu = np.zeros([6,1])
        self.nu_previous = np.zeros([6,1])
        
        self.eta = np.zeros([6,1])
        self.eta_previous = np.zeros([6,1])
        
        self.signal = True #only used for simulating drop out and frosen points (not in use pr. 23.02.)
        
        self.nu_dot = np.zeros([6,1])
        self.eta_dot = np.zeros([6,1])
        
        self.bias = np.zeros([6,1])
        self.bias_dot = np.zeros([6,1])
        self.noiseBias = np.zeros([6,1])
        self.noiseMeasurement = np.zeros([6,1])
        
        self.mean = params["imu_properties"]["mean"]
        self.std = params["imu_properties"]["standardDeviation"]
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        self.g = np.array([0.0, 0.0, 9.81]) 
        self.g = np.resize(self.g, (3,1))
        #For plotting:
        self.biasVec = np.array([])
        
        #ROS related:
        self.msg_imu = Imu()
        self.pub_imu = rospy.Publisher(self.topic, Imu, queue_size=1)
        
    
    def publish(self):
        
        measurementAcc = self.getAccMeasured()
        self.msg_imu.linear_acceleration.x = measurementAcc[0]
        self.msg_imu.linear_acceleration.y = measurementAcc[1]
        self.msg_imu.linear_acceleration.z = measurementAcc[2]
        
        measurementGyro = self.getGyroMeasured()
        self.msg_imu.angular_velocity.x = measurementGyro[0]
        self.msg_imu.angular_velocity.y = measurementGyro[1]
        self.msg_imu.angular_velocity.z = measurementGyro[2]
        
        self.msg_imu.linear_acceleration_covariance[0] = self.std**2
        self.msg_imu.linear_acceleration_covariance[4] = self.std**2
        self.msg_imu.linear_acceleration_covariance[8] = self.std**2
        
        self.msg_imu.angular_velocity_covariance[0] = self.std**2
        self.msg_imu.angular_velocity_covariance[4] = self.std**2
        self.msg_imu.angular_velocity_covariance[8] = self.std**2
        
        self.msg_imu.orientation_covariance[0] = -1
        
        self.msg_imu.header.stamp = rospy.Time.now()
        self.msg_imu.header.frame_id = "simulated_imu"
        
        self.pub_imu.publish(self.msg_imu)
        
    

    
    def setOdometry(self, eta, nu, eta_dot, nu_dot):
        self.eta = eta
        self.nu = nu
        self.eta_dot = eta_dot
        self.nu_dot = nu_dot
    
        
    def getImuLocation(self):
        
        imuName = 'imu'+str(self.id)
        locations = params['imu_locations'][imuName]
        self.x = locations['x']
        self.y = locations['y']
        self.z = locations['z']
        loc = np.array([self.x, self.y, self.z])
        loc = np.resize(loc, (3,1))
        return loc
    
    def bodyToSensorFrame(self):
        """transforming odometry motions from body fram to sensor frame
        """
        # angularVelocity_sensor = angularVelocity_body
        
        
    
    
    def plotIMU(self):
        """"Plot measurment"""
        
        plt.plot(self.biasVec) #Modify
        plt.pause(0.01)
        
        
        
    def __getBias(self):
        """Bias term"""
        self.noiseBias = np.random.normal(self.mean, self.std, 6)
        self.noiseBias = np.resize(self.noiseBias, (6,1))
        self.bias_dot = self.noiseBias
        self.bias += self.bias_dot*self.dt
        return self.bias
    
    def __getMeasurementNoise(self):
        w = np.random.normal(self.mean, self.std, 6)
        w = np.resize(w, (6,1))
        return w
    
    def getS(self, vec):
        p = vec[0]
        q = vec[1]
        r = vec[2]
        S = np.array([[0.0, -r, q],
                      [r, 0.0, -p],
                      [-q, p, 0.0]])
        return S
        
    def getAccMeasured(self):
        """Outputs the simulated measurement from one imu in sensor frame"""
        l = self.getImuLocation()
    
        S_nu = self.getS(self.nu[3:6])
        g_dot = -np.matmul(S_nu, self.g)
        g_dot = g_dot.astype(float)
        
        self.g += g_dot*self.dt
        
        b = self.__getBias()
        w = self.__getMeasurementNoise()
        
        a_l = self.nu_dot[0:3] + np.matmul(S_nu, l) + np.matmul(np.matmul(S_nu, S_nu), l)
        a_l = a_l.astype(float)
        
        a_m = a_l + np.matmul(S_nu, self.nu[0:3]) + self.g + b[0:3] + w[0:3]
        a_m = a_m.astype(float)
        
        self.time += self.dt
        """Remember to turn the output measurements so that it is equvivalent to the physical setup!!!"""
        return a_m
    
    def getGyroMeasured(self):
        
        return [0.0, 0.0, 0.0]