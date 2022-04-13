#!/usr/bin/env python
import numpy as np
import rospy
import os 
import yaml
from sensors import IMU, GNSS 
from CSAD_DP_6DOF import CSAD 
from waveLoads import Wave
from ThrusterDynamics import ThrusterDynamics
from std_msgs.msg import Float64MultiArray
# from nav_msgs.msg import Odometry

# from sensor_msgs.msg import Imu

path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/simulator/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)

#Sensors:
imu1 = IMU(1)
imu2 = IMU(2)
imu3 = IMU(3)
imu4 = IMU(4)
gnss = GNSS()

#Vessel:
eta0 = np.zeros(6)
vessel = CSAD(eta0)
u0 = np.zeros(12)
thrusters = ThrusterDynamics(u0)


#Seastate:
Hs = 0.4
Tp = 1.0
seastate = Wave(Hs, Tp, angle=0, regular = True)
seastate.updateHeading(vessel.eta[5])

def initSimulatorNode():
    global node
    node = rospy.init_node("Simulator_node")
    rospy.Subscriber("/CSAD/u", Float64MultiArray, thrusters.updateU)

def nodeEnd():
    node.destroy_node()
    
if __name__ == '__main__':

    initSimulatorNode()
    r = rospy.Rate(params["runfrequency"]) # Usually set to 100 Hz
    
    while not rospy.is_shutdown():
        
        tau_wave = seastate.getWaveLoads()
        tau_thr = thrusters.getThrustLoads() #needs to be finnished
        waveFrequency = seastate.frequency
        
        vessel.updateStates(tau_wave, tau_thr, waveFrequency)
        seastate.updateHeading(vessel.eta[5])
        
        imu1.setOdometry(vessel.eta, vessel.nu, vessel.eta_dot, vessel.nu_dot)
        imu2.setOdometry(vessel.eta, vessel.nu, vessel.eta_dot, vessel.nu_dot)
        imu3.setOdometry(vessel.eta, vessel.nu, vessel.eta_dot, vessel.nu_dot)
        imu4.setOdometry(vessel.eta, vessel.nu, vessel.eta_dot, vessel.nu_dot)
        gnss.setOdometry(vessel.eta, vessel.nu, vessel.eta_dot, vessel.nu_dot)
        
        measurement_wave = seastate.getWaveElevation(30) #needs to be made for use in control system
        
        
        
        vessel.publish()
        imu1.publish()
        imu2.publish()
        imu3.publish()
        imu4.publish()
        gnss.publish()
        # seastate.publish()
        
        r.sleep()
    
    nodeEnd()