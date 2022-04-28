#!/usr/bin/env python
from numpy import angle
import generateModelData_CSAD as data
from waveLoads import Wave
import math_tools

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from ThrusterDynamics import ThrusterDynamics
import rospy
import numpy as np
from matplotlib import pyplot
from mpl_toolkits import mplot3d
import time
import os
import yaml

path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/simulator/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)
    
global timeStep
timeStep = 1.0/params["runfrequency"]

class CSAD:
    
    
    def __init__(self, eta0, dt=timeStep):
        self.dt = dt
        self.time = 0
        self.RAOForce = data.RAO_FORCE
        #Initialzing states
        self.eta = eta0
        self.nu = np.zeros(6)
        self.bias = np.zeros(6)
        self.eta_dot = np.zeros(6)
        self.nu_dot = np.zeros(6)
        self.bias_dot = np.zeros(6)
        
        self.T_b = 0.0*np.ones(6)   # Tuning bias 2~3 times larger than the wave period(?) Make function for this!
        self.biasMean = 0.0     # Defining white noise
        self.biasStd = 0.0      # Defining white noise
        
        #Initionalize thrust dynamics
        self.u = np.zeros(12)
        self.thrustDynamics = ThrusterDynamics(self.u)
        
        #ROS communication related:
        self.odometry_msg = Odometry()
        self.odometry_msg.header.frame_id = "odometry"
        
        #Simulates the published qualisys node generated by the simulation model:
        self.pub_odometry = rospy.Publisher('/CSAD/simulator', Odometry, queue_size=1)
        self.pub_odometry2 = rospy.Publisher('/qualisys/Body_1/odom', Odometry, queue_size=1)
        
        #Subscribes on the u vector from thrust allocation: 
        self.sub_u = rospy.Subscriber('/CSAD/u', Float32MultiArray, queue_size=1)
        
        # For plotting:
        self.timeVec = []
        self.xVec = []
        self.yVec = []
        self.zVec = []
        self.phiVec = []
        self.thetaVec = []
        self.psiVec = []
        self.thrustLoadVec = []
        
    def publish(self):
        quaternion = math_tools.euler2quat(self.eta[3], self.eta[4], self.eta[5])
        self.odometry_msg.pose.pose.position.x = self.eta[0]
        self.odometry_msg.pose.pose.position.y = self.eta[1]
        self.odometry_msg.pose.pose.position.z = self.eta[2]
        self.odometry_msg.pose.pose.orientation.x = quaternion[0]
        self.odometry_msg.pose.pose.orientation.y = quaternion[1]
        self.odometry_msg.pose.pose.orientation.z = quaternion[2]
        self.odometry_msg.pose.pose.orientation.w = quaternion[3]
        
        self.odometry_msg.twist.twist.linear.x = self.nu[0]
        self.odometry_msg.twist.twist.linear.y = self.nu[1]
        self.odometry_msg.twist.twist.linear.z = self.nu[2]
        self.odometry_msg.twist.twist.angular.x = self.nu[3]
        self.odometry_msg.twist.twist.angular.y = self.nu[4]
        self.odometry_msg.twist.twist.angular.z = self.nu[5]
        
        self.odometry_msg.header.stamp.secs = self.time
        
        self.pub_odometry.publish(self.odometry_msg)
        self.pub_odometry2.publish(self.odometry_msg)
    
     
    def saveData(self):
        """
        Creating vectors for plotting.
        """
        self.timeVec.append(self.time)
        self.xVec.append(self.eta[0])
        self.yVec.append(self.eta[1])
        self.zVec.append(self.eta[2])
        self.phiVec.append(self.eta[3])
        self.thetaVec.append(self.eta[4])
        self.psiVec.append(self.eta[5])
        self.thrustLoadVec.append(self.thrustDynamics.loads)
        
    
    def nav_msg(self):
        quaternion = math_tools.euler2quat(self.eta[3], self.eta[4], self.eta[5])
        self.odometry_msg.pose.pose.position.x = self.eta[0]
        self.odometry_msg.pose.pose.position.y = self.eta[1]
        self.odometry_msg.pose.pose.position.z = self.eta[2]
        self.odometry_msg.pose.pose.orientation.x = quaternion[0]
        self.odometry_msg.pose.pose.orientation.y = quaternion[1]
        self.odometry_msg.pose.pose.orientation.z = quaternion[2]
        self.odometry_msg.pose.pose.orientation.w = quaternion[3]
        
        self.odometry_msg.twist.twist.linear.x = self.nu[0]
        self.odometry_msg.twist.twist.linear.y = self.nu[1]
        self.odometry_msg.twist.twist.linear.z = self.nu[2]
        self.odometry_msg.twist.twist.angular.x = self.nu[3]
        self.odometry_msg.twist.twist.angular.y = self.nu[4]
        self.odometry_msg.twist.twist.angular.z = self.nu[5]
        
        
    # def getC(self):
        
    def updateStates(self, tauEnv, tauThr, waveFreq):
        """
        Update states for the vessel, with parameters based on different frequencies achieved from experiments (Bjornoe).
        Note that Waves.setHeading() have to be called after calling this function.
        """
        index = np.argmin(np.abs(data.frequencies - waveFreq)) #Should probably be a interpolation instead...
        A = data.A[:,:,index]   #Added mass
        B = data.B[:,:,index]   #Potential + viscous damping
        C = data.C[:,:,index]   #Restoring forces
        
        M = A + data.MRB
        Minv = np.linalg.inv(M)
        J = math_tools.transformationMatrix(self.eta)
        Jinv = np.linalg.inv(J)
        
        self.eta_dot = np.matmul(J, self.nu)
        
        noise = np.random.normal(self.biasMean, self.biasStd, 6)
        self.bias_dot = np.matmul(-self.T_b, self.bias) + noise
        
        self.nu_dot = np.matmul(Minv, (np.matmul(-B, self.nu) + np.matmul(-C, self.eta) + np.matmul(Jinv, self.bias) + tauEnv + tauThr))
        
        # Euler integration:
        self.eta += self.eta_dot*self.dt
        self.eta[5] = math_tools.ssa(self.eta[5])
        self.nu += self.nu_dot*self.dt
        self.bias += self.bias_dot*self.dt
        
        self.saveData()
        self.time += self.dt
        


# eta0 = np.zeros(6)
# vessel = CSAD(eta0, dt=0.01)
# seastate = Wave(0.04*2, 1.0, angle=np.radians(0), dt=0.01, regular=True)
# thrLoad = np.zeros(6)
# loadx = []
# loady = []

# while vessel.time < 20:
#     start = time.time()
#     waveLoads = seastate.getWaveLoads()
#     vessel.updateStates(waveLoads, thrLoad, seastate.frequency)
#     seastate.updateHeading(vessel.eta[5])
#     end = time.time()

#     loadx.append(waveLoads[0])
#     loady.append(waveLoads[1])
    
    
    

# fig = pyplot.figure()
# pyplot.title('Load function of time')
# pyplot.plot(vessel.timeVec,loadx, label='loadx')
# pyplot.plot(vessel.timeVec,loady,'--', label='loady')
# pyplot.legend()
# fig1 = pyplot.figure()
# pyplot.title('Pos function of time')
# pyplot.plot(vessel.timeVec,vessel.xVec, label='x')
# pyplot.plot(vessel.timeVec,vessel.yVec, label='y')
# pyplot.plot(vessel.timeVec,vessel.psiVec, label='psi')
# #pyplot.plot(vessel.timeVec,vessel.thetaVec, label='pitch')
# #pyplot.plot(vessel.timeVec,vessel.psiVec, '--', label='heading')
# pyplot.legend()
# pyplot.grid()

# # fig2 = pyplot.figure()
# # ax = pyplot.axes()

# # ax.quiver(vessel.xVec, vessel.yVec, np.cos(vessel.psiVec), np.sin(vessel.psiVec), scale=5)
# # pyplot.title('XY plot')
# # #pyplot.plot(vessel.xVec, vessel.yVec, 'o', label='vessel1', markersize=2)
# # #pyplot.legend()
# # pyplot.grid()
# pyplot.show()


# # Predefined objects:
# eta0 = np.zeros(6)
# vessel = CSAD(eta0)

# Hs = 0.01
# Tp = 0.01
# waveAngle = 0
# seastate = Wave(Hs, Tp, waveAngle, regular=True)
# seastate.updateHeading(vessel.eta[5]) #Set heading must be called every time updateStates() are called!

# # def loop():
# #     # Update loads:
# #     tauWave = seastate.getWaveLoads() #!
# #     tauThrust = vessel.thrustDynamics.getThrustLoads()
# #     #Update vessel dynamics:
# #     vessel.updateStates(tauWave, tauThrust, seastate.frequency)
# #     seastate.updateHeading(vessel.eta[5])  #Set heading must be called every time updateStates() are called!
    
    
#     # return 0
