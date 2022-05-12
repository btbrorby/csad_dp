#!/usr/bin/env python
import numpy as np
import math
from lib import qualisys, tau, observer, gains
from math_tools import Rzyx, rad2pipi
import matplotlib.pyplot as plt
import yaml
import os
import rospy

class Observer:
    def __init__(self, dt=0.01, peakFrequency=1.0):
        self.xsi_hat = np.zeros([6,1])
        self.eta_hat = np.zeros([3,1])
        self.nu_hat = np.zeros([3,1])
        self.bias_hat = np.zeros([3,1])
        self.eta = np.zeros([3,1])
        self.tau = np.zeros([3,1])
        
        self.y_tilde = np.zeros([3,1])
        
        # self.L_1 = 0.0*np.diag([1.0, 1.0, 1.0])
        # self.L_2 = np.diag([5.2, 3.3, 5.7])
        # self.L_3 = 10.0*np.diag([675.0, 675.0, 130.0])
        # self.L_4 = np.diag([1.0, 1.0, 1.0])
        
        self.Tb = (1000.0/90.0)*np.diag([1.0,1.0,1.0])
        
        omega_c = peakFrequency + 0.01
        L_11 = -2.0*(1.0-0.1)*(omega_c/peakFrequency)*np.diag([1.0, 1.0, 1.0])
        L_12 = -(peakFrequency**2.0/omega_c)*L_11
        self.L_1 = np.concatenate((L_11, L_12), 0)
        self.L_2 = omega_c*np.diag([0.01, 0.1, 0.1])
        self.L_3 = 800.0*np.diag([0.6, 0.9, 0.18])
        self.L_4 = 0.01*self.L_3
        self.L_4[2,2] *= 0.01
        
        dampingRatio = 0.1
        Omega = peakFrequency*np.eye(3)
        Lamda = dampingRatio*np.eye(3)
        Aw1 = np.concatenate((np.zeros([3 ,3]), np.eye(3)), axis=1)
        Aw2 = np.concatenate((-np.power(Omega, 2.0), -2.0*np.matmul(Lamda, Omega)), 1)
        self.A_w = np.concatenate((Aw1, Aw2), 0)
        
        # self.A_w = np.eye(3)
        self.C_w = np.concatenate((np.zeros([3,3]), np.eye(3)), 1)
        
        self.time = 0.0
        
    def updateStates(self, eta, tau):
        self.eta = eta
        self.tau = tau
        
    def updateGains(self, L_1, L_2, L_3, L_4):
        self.L_1 = L_1
        self.L_2 = L_2
        self.L_3 = L_3
        self.L_4 = L_4
        
    def observer_linear(self):
        """
        Observer
        """
        
        
        # Tuning parameters:
        # L_1 = np.diag([10.0, 10.0, 10.0])
        # L_2 = np.diag([50.0, 50.0, 50.0])
        # L_3 = np.diag([1.0, 1.0, 1.0])
        
        MRB = np.array([[127.92, 0.0, 0.0],
                        [0.0, 127.92, 0.0],
                        [0.0, 0.0, 61.967]])
        MA = np.array([[3.262, 0.0, 0.0],
                       [0.0, 28.89, 0.525],
                       [0.0, 0.525, 13.98]])
        M = MRB + MA
        M_inv = np.linalg.inv(M)
        D = np.array([[80.66, 0.0, 0.0],
                      [0.0, 632.09, 34.67],
                      [0.0, 34.67, 228.07]])    #taken from data.B(omega=2pi)
        # D = np.array([[2.332, 0.0, 0.0],
        #               [0.0, 4.673, 0.0],
        #               [0.0, 0.0, 0.01675]])
        
        self.eta[2] = rad2pipi(self.eta[2]) #Is this right?
        R = Rzyx(self.eta[2])
        R_inv = np.transpose(R)
        
        
        y_hat = self.eta_hat + np.matmul(self.C_w, self.xsi_hat)
        self.y_tilde = self.eta - y_hat
        self.y_tilde[2] = rad2pipi(self.y_tilde[2])
        
        # Observer dynamics:
        xsi_hat_dot = np.matmul(self.A_w, self.xsi_hat) + np.matmul(self.L_1, self.y_tilde)
        xsi_hat_dot = xsi_hat_dot.astype(float)
        
        eta_hat_dot = np.matmul(R, self.nu_hat) + np.matmul(self.L_2, self.y_tilde)
        eta_hat_dot = eta_hat_dot.astype(float)
        
        nu_hat_dot = np.matmul(M_inv, (-np.matmul(D, self.nu_hat) + np.matmul(R_inv, self.bias_hat) + np.matmul(R_inv, np.matmul(self.L_3, self.y_tilde)) + self.tau))
        nu_hat_dot = nu_hat_dot.astype(float)
        
        bias_hat_dot = np.matmul(self.L_4, self.y_tilde)
        bias_hat_dot = bias_hat_dot.astype(float)
        
        # Euler integration:
        self.xsi_hat += xsi_hat_dot*dt
        self.eta_hat += eta_hat_dot*dt
        self.eta_hat[2] = rad2pipi(self.eta_hat[2])
        self.bias_hat += bias_hat_dot*dt
        self.nu_hat += nu_hat_dot*dt
        
        self.time += dt
        
        return self.eta_hat, self.nu_hat, self.bias_hat


path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/observer/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)
dt = 1.0/params["runfrequency"]

peakFrequency = 1.1

linearObserver = Observer(dt, peakFrequency)

def loop():
    """
    All calls to functions and methods should be handled inside here. loop() is called by the main-function in obs_node.py
    """
    eta = qualisys.getQualisysOdometry()
    
    # L1 = gains.L1
    # L2 = gains.L2
    # L3 = gains.L3
    # L4 = gains.L4
    # if ((not np.array_equal(L1, linearObserver.L_1)) or (not np.array_equal(L2, linearObserver.L_2)) or (not np.array_equal(L3, linearObserver.L_3)) or (not np.array_equal(L4, linearObserver.L_4))):
    #     linearObserver.updateGains(L1, L2, L3, L4)
    
    controlOutput = tau.getTau()
    linearObserver.updateStates(eta, controlOutput)
    [eta_hat, nu_hat, bias_hat] = linearObserver.observer_linear()
    observer.publish(eta_hat, nu_hat, bias_hat)
    
    
    
    
    # qualisys.publish()
    # rospy.spin()
    

# o = Observer(0.01)
# eta = np.array([1.0, 2.0, 3.0])
# tau = np.array([4.0, 5.0, 6.0])
# o.updateStates(eta, tau)
# [a, b, c] = o.observer_linear()
# print(a, b, c)