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
import time as tic

path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/simulator/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)
rate = params["runfrequency"]

#Sensors:
imu1 = IMU(1, dt=1.0/rate)
imu2 = IMU(2, dt=1.0/rate)
imu3 = IMU(3, dt=1.0/rate)
imu4 = IMU(4, dt=1.0/rate)
gnss = GNSS(dt=1.0/rate)

#Vessel:
eta0 = np.zeros([6, 1])
vessel = CSAD(eta0, dt=1.0/rate)
u0 = np.zeros([12, 1])
thrusters = ThrusterDynamics(u0, dt=1.0/rate)


#Seastate:
Hs = 0.06
Tp = 1.15
seastate = Wave(Hs, Tp, stateDescription='rough', angle=0.0*np.pi/180.0, regular = False, dt=1.0/rate)
seastate.updateHeading(vessel.eta[5])
    
if __name__ == '__main__':

    global node
    node = rospy.init_node("Simulator_node")
    rospy.Subscriber("/CSAD/u", Float64MultiArray, thrusters.updateU)
    r = rospy.Rate(params["runfrequency"]) # Usually set to 100 Hz
    t0 = tic.time()
    while not rospy.is_shutdown():
        t1 = tic.time()
        tau_wave = seastate.getWaveLoads()
        tau_thr = thrusters.getThrustLoads()
        waveFrequency = seastate.frequency
        
        vessel.updateStates(tau_wave, tau_thr, waveFrequency)
        seastate.updateHeading(vessel.eta[5])
        
        imu1.setOdometry(vessel.eta, vessel.nu, vessel.eta_dot, vessel.nu_dot)
        imu2.setOdometry(vessel.eta, vessel.nu, vessel.eta_dot, vessel.nu_dot)
        imu3.setOdometry(vessel.eta, vessel.nu, vessel.eta_dot, vessel.nu_dot)
        imu4.setOdometry(vessel.eta, vessel.nu, vessel.eta_dot, vessel.nu_dot)
        gnss.setOdometry(vessel.eta, vessel.nu, vessel.eta_dot, vessel.nu_dot)
        
        vessel.publish()
        thrusters.publish(tau_thr)
        seastate.publish(tauWave=tau_wave)
        imu1.publish()
        imu2.publish()
        imu3.publish()
        imu4.publish()
        # gnss.publish()
        
        r.sleep()
        # rospy.spin()
        print("SIMULATION time:", tic.time()-t0)
        # print("Time step:", tic.time()-t1)

    
    node.destroy_node()
    
    