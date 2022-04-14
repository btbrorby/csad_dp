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

path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/simulator/src/sensorParams.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)


class GNSS():
    """
    This class simulates the GNSS signals, or the odometry given from qualisys.
    """
    def __init__(self, dt=0.01):
        self.dt = dt
        self.time = 0.0
        
        self.eta = np.zeros(6)
        self.nu = np.zeros(6)
        self.eta_dot = np.zeros(6)
        self.nu_dot = np.zeros(6)
        
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
        
        self.msg_gnss.header.stamp.secs = self.time
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
        output = np.append(self.eta, self.nu)
        return output
            
        
        

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
        
        self.msg_imu.header.stamp.secs = self.time
        self.msg_imu.header.frame_id = "simulated_imu"
        
        self.pub_imu.publish(self.msg_imu)
        
    

    
    def setOdometry(self, eta, nu, eta_dot, nu_dot):
        self.eta = eta
        self.nu = nu
        self.eta_dot = eta_dot
        self.nu_dot = nu_dot
    
        
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
        l = self.getImuLocation()
        
        g_dot = np.cross(-self.nu[3:6], self.g)
        self.g += g_dot*self.dt
        b = self.__getBias()
        w = self.__getMeasurementNoise()
        
        a_l = self.nu_dot[0:3] + np.cross(self.nu_dot[3:6], l) + np.cross(self.nu[3:6], np.cross(self.nu[3:6], l))
        a_m = a_l + np.cross(self.nu[3:6], self.nu[0:3]) + self.g + b[0:3] + w[0:3]
                
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






# def sensorNodeInit():
#     global node
#     node = rospy.init_node("Sensor_node")
#     rospy.Subscriber('/CSAD/simulator', Odometry, imu.updateOdometry)
#     rospy.Subscriber('/CSAD/simulator', Odometry, gnss.updateGnss)
