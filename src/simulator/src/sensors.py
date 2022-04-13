#!/usr/bin/env python
from operator import imul
from platform import node
import numpy as np
import matplotlib.pyplot as plt
import math_tools
from numpy import size
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import yaml
import os

path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/simulator/src/sensorParams.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)


class GNSS():
    """
    This class simulates the GNSS signals, or the odometry given from qualisys.
    """
    def __init__(self, dt=0.01):
        self.dt = dt
        self.eta = 0.0
        
        self.signal = True #only used for simulating drop out and frosen points (not in use pr. 23.02.)
        
        self.pub_gnss = rospy.Publisher('/qualisys/Body_1/odom', Odometry, queue_size=1)
        self.msg_odom = Odometry()
        self.msg_gnss = Odometry()
        
        
    def updateOdom(self, msg):
        self.msg_odom = msg
        self.getOdom()
    
    def getOdom(self):
        # Velocities:
        self.nu[0] = self.msg_odometry.twist.twist.linear.x
        self.nu[1] = self.msg_odometry.twist.twist.linear.y
        self.nu[2] = self.msg_odometry.twist.twist.linear.z
        self.nu[3] = self.msg_odometry.twist.twist.angular.x
        self.nu[4] = self.msg_odometry.twist.twist.angular.y
        self.nu[5] = self.msg_odometry.twist.twist.angular.z
        # Poses:
        self.eta[0] = self.msg_odometry.pose.pose.position.x
        self.eta[1] = self.msg_odometry.pose.pose.position.y
        self.eta[2] = self.msg_odometry.pose.pose.position.z
        
        quat_x = self.msg_odometry.pose.pose.orientation.x
        quat_y = self.msg_odometry.pose.pose.orientation.y
        quat_z = self.msg_odometry.pose.pose.orientation.z
        quat_w = self.msg_odometry.pose.pose.orientation.w
        self.eta[3:6] = math_tools.quat2eul(quat_w, quat_x, quat_y, quat_z)
        return self.eta, self.nu
        
        
    def generateDropOut(self):
        """This is only an example of how to give a random dropout"""
        dropOut = False
        randomDropOut = np.random.normal(0, 1)
        if (randomDropOut > 1.5):
            dropOut = True
        return dropOut
    
    def getGnssMeasured(self):
        return self.msg_odom
            
        
        

class IMU():
    """
    This class simulates imu measurements located on a vessel. The location have to be specified in sensorParams.yaml
    acc_tilde = acc + bias + whiteNoise
    """
    def __init__(self, id, dt=0.01):
        self.time = 0.0
        
        self.id = id
        self.topic = '/imu' + str(id)
        self.dt = dt
        self.nu = np.zeros(6)
        self.nu_previous = np.zeros(6)
        
        self.eta = np.zeros(6)
        self.eta_previous = np.zeros(6)
        
        self.signal = True #only used for simulating drop out and frosen points (not in use pr. 23.02.)
        
        self.nu_dot = np.zeros(6)
        self.eta_dot = np.zeros(6)
        
        self.bias = np.zeros(6)
        self.bias_dot = np.zeros(6)
        self.noiseBias = np.zeros(6)
        self.noiseMeasurement = np.zeros(6)
        
        self.mean = params["imu_properties"]["mean"]
        self.std = params["imu_properties"]["standardDeviation"]
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        self.g = [0.0, 0.0, 9.81] #expressed in body frame
        
        
        #For plotting:
        self.biasVec = np.array([])
        
        
        
        #ROS related:
        self.msg_odometry = Odometry()
        
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
        
        self.msg_imu.header.stamp.secs = self.time
        
        self.pub_imu.publish(self.msg_imu)
        
    
    def updateOdometry(self, msg):
        #Probably not nessecarry
        self.msg_odometry = msg
        self.getOdom()
    
    def setOdometry(self, eta, nu, eta_dot, nu_dot):
        self.eta = eta
        self.nu = nu
        self.eta_dot = eta_dot
        self.nu_dot = nu_dot
    
    def __getOdom(self):
        # #Velocities:
        # self.nu[0] = self.msg_odometry.twist.twist.linear.x
        # self.nu[1] = self.msg_odometry.twist.twist.linear.y
        # self.nu[2] = self.msg_odometry.twist.twist.linear.z
        # self.nu[3] = self.msg_odometry.twist.twist.angular.x
        # self.nu[4] = self.msg_odometry.twist.twist.angular.y
        # self.nu[5] = self.msg_odometry.twist.twist.angular.z
        # #Poses:
        # self.eta[0] = self.msg_odometry.pose.pose.position.x
        # self.eta[1] = self.msg_odometry.pose.pose.position.y
        # self.eta[2] = self.msg_odometry.pose.pose.position.z
        
        # quat_x = self.msg_odometry.pose.pose.orientation.x
        # quat_y = self.msg_odometry.pose.pose.orientation.y
        # quat_z = self.msg_odometry.pose.pose.orientation.z
        # quat_w = self.msg_odometry.pose.pose.orientation.w
        # self.eta[3:6] = math_tools.quat2eul(quat_w, quat_x, quat_y, quat_z)
        return self.eta, self.nu
        
    def getImuLocation(self):
        path = os.path.dirname(os.getcwd())
        with open(r"{0}/csad_dp_ws/src/simulator/src/sensorParams.yaml".format(path)) as file:
            params = yaml.load(file, Loader=yaml.Loader)
        
        imuName = 'imu'+str(self.id)
        locations = params['imu_locations'][imuName]
        self.x = locations['x']
        self.y = locations['y']
        self.z = locations['z']
        loc = np.array([self.x, self.y, self.z])
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
        self.bias_dot = self.noiseBias
        self.bias = self.bias + self.bias_dot*self.dt
        return self.bias
    
    def __getMeasurementNoise(self):
        w = np.random.normal(self.mean, self.std, 6)
        return w
    
    
        
    def getAccMeasured(self):
        """Outputs the simulated measurement from one imu in sensor frame"""
        [eta, nu] =self.__getOdom()
        l = self.getImuLocation()
        
        g_dot = np.cross(-nu[3:6], self.g)
        self.g += g_dot*self.dt
        b = self.__getBias()
        w = self.__getMeasurementNoise()
        
        a_l = self.nu_dot[0:3] + np.cross(self.nu_dot[3:6], l) + np.cross(nu[3:6], np.cross(nu[3:6], l))
        a_m = a_l + np.cross(nu[3:6], nu[0:3]) + self.g + b[0:3] + w[0:3]
        
        self.time += self.dt
        """Remember to turn the output measurements so that it is equvivalent to the physical setup!!!"""
        return a_m
    
    def getGyroMeasured(self):
        return [0.0, 0.0, 0.0]

        


"""For testing"""
# dt = 0.01
# gnss = GNSS(dt)
# imu = IMU(4)


# i = 0
# fig = plt.figure()
# while i < 1000:
#     imu.biasVec = np.append(imu.biasVec, imu.getAccMeasured(0.01))
#     if np.size(imu.biasVec) > 100:
#         imu.biasVec = np.delete(imu.biasVec, 0)
#         plt.clf()
#     i += 1
#     print(imu.biasVec)
#     imu.plotIMU()

# # plt.show()

# print('/imu' + str(4))






def sensorNodeInit():
    global node
    node = rospy.init_node("Sensor_node")
    rospy.Subscriber('/CSAD/simulator', Odometry, imu.updateOdometry)
    rospy.Subscriber('/CSAD/simulator', Odometry, gnss.updateGnss)


def loop():
    
    return 0