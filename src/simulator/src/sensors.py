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



class GNSS():
    """
    This class simulates the GNSS signals, or the odometry given from qualisys.
    """
    def __init__(self, dt):
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
            
        
        

class IMU():
    """
    This class simulates imu measurements located on a vessel. The location have to be specified in sensorParams.yaml
    acc_tilde = acc + bias + whiteNoise
    """
    def __init__(self, dt, id):
        self.id = id
        self.topic = '/imu' + str(id)
        self.dt = dt
        self.nu = np.zeros(6)
        self.nu_previous = np.zeros(6)
        
        self.eta = np.zeros(6)
        self.eta_previous = np.zeros(6)
        
        self.signal = True #only used for simulating drop out and frosen points (not in use pr. 23.02.)
        
        self.nu_d = np.zeros(6)
        self.nu_d_previous = np.zeros(6)
        
        self.bias = np.zeros(6)
        self.bias_d = np.zeros(6)
        self.noiseBias = np.zeros(6)
        self.noiseMeasurement = np.zeros(6)
        
        self.mean = 0
        self.std = 0.1
        
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
    
    def updateOdometry(self, msg):
        self.msg_odometry = msg
        self.getOdom()
    
    def getOdom(self):
        #Velocities:
        self.nu[0] = self.msg_odometry.twist.twist.linear.x
        self.nu[1] = self.msg_odometry.twist.twist.linear.y
        self.nu[2] = self.msg_odometry.twist.twist.linear.z
        self.nu[3] = self.msg_odometry.twist.twist.angular.x
        self.nu[4] = self.msg_odometry.twist.twist.angular.y
        self.nu[5] = self.msg_odometry.twist.twist.angular.z
        #Poses:
        self.eta[0] = self.msg_odometry.pose.pose.position.x
        self.eta[1] = self.msg_odometry.pose.pose.position.y
        self.eta[2] = self.msg_odometry.pose.pose.position.z
        
        quat_x = self.msg_odometry.pose.pose.orientation.x
        quat_y = self.msg_odometry.pose.pose.orientation.y
        quat_z = self.msg_odometry.pose.pose.orientation.z
        quat_w = self.msg_odometry.pose.pose.orientation.w
        self.eta[3:6] = math_tools.quat2eul(quat_w, quat_x, quat_y, quat_z)
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
        
        return self.x, self.y, self.z
    
    def bodyToSensorFrame(self):
        """transforming odometry motions from body fram to sensor frame
        """
        # angularVelocity_sensor = angularVelocity_body
        
        
    
    
    def plotIMU(self):
        """"Plot measurment"""
        
        plt.plot(self.biasVec) #Modify
        plt.pause(0.01)
        
        
        
    def getBias(self):
        """Bias term"""
        self.noiseBias = np.random.normal(self.mean, self.std, 1)
        self.bias_d = self.noiseBias
        self.bias = self.bias + self.bias_d*self.dt
        return self.bias
    
    def getMeasurementNoise(self):
        w = np.random.normal(self.mean, self.std, 1)
        return w
        
        
    def getAccMeasured(self):
        """Outputs the simulated measurement from one imu in sensor frame"""
        [eta, nu] =self.getOdom()
        l = self.getImuLocation()
        
        g_dot = np.matmul(-nu[3:6], self.g)
        self.g += g_dot
        b = self.getBias()
        w = self.getMeasurementNoise()
        
        # al = #unfinnished...
        am = al + np.matmul(nu[3:6], nu[0:3]) + self.g + b + w
        
        
        S = math_tools.skewSymmetricMatrix(l)
        H = np.array([[0, -l[0], -l[0], l[1], l[2], 0],
                      [-l[1], 0, -l[1], l[0], 0, l[2]],
                      [-l[2], -l[2], 0, 0, l[0], l[1]]])
        W = np.array([np.eye(3), np.linalg.inv(S), H])
        
        al = np.matmul(W) #unfinnished....
        
        
        self.noiseMeasurement = np.random.normal(self.mean, self.std, 1)
        self.getBias()
        
        """Remember to turn the output measurements so that it is equvivalent to the physical setup!!!"""
        return self.nu_d + self.bias + self.noiseMeasurement
    
    
        


"""For testing"""
dt = 0.01
gnss = GNSS(dt)
imu = IMU(dt, 4)

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

# plt.show()

print(imu.getImuLocation())






def sensorNodeInit():
    global node
    node = rospy.init_node("Sensor_node")
    rospy.Subscriber('/CSAD/simulator', Odometry, imu.updateOdometry)
    rospy.Subscriber('/CSAD/simulator', Odometry, gnss.updateGnss)


def loop():
    
    return 0