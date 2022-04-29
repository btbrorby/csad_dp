#!/usr/bin/env python
import numpy as np
import math
from lib import qualisys, tau, observer, gains
from math_tools import Rzyx, rad2pipi
import yaml
import os

### Write your code here ###
class Observer:
    def __init__(self, dt):
        self.xsi_hat = self.eta_hat = self.nu_hat = self.bias_hat = np.zeros([3,1])
        self.eta = self.tau = np.zeros([3,1])
        
        self.L_1 = np.zeros([3,3])
        self.L_2 = np.zeros([3,3])
        self.L_3 = np.zeros([3,3])
        self.L_4 = np.zeros([3,3])
        
        self.A_w = np.zeros([3,3])
        self.C_w = np.zeros([3,3])
        
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
        
        MRB = np.array([127.92, 127.92, 61.967])
        MA = np.array([[3.262, 0.0, 0.0],
                    [0.0, 28.89, 0.525],
                    [0.0, 0.525, 13.98]])
        M = MRB + MA
        M_inv = np.linalg.inv(M)
        
        D = np.array([[2.332, 0.0, 0.0],
                    [0.0, 4.673, 0.0],
                    [0.0, 0.0, 0.01675]])
        
        self.eta[2] = rad2pipi(self.eta[2]) #Is this right?
        R = Rzyx(self.eta[2])
        R_inv = np.transpose(R)
        
        y_tilde = self.eta_hat + np.matmul(self.C_w, self.xsi_hat)
        eta_tilde = self.eta - self.eta_hat
        
        # Observer dynamics:
        xsi_hat_dot = np.matmul(self.A_w, self.xsi_hat) + np.matmul(self.L_1, y_tilde)
        eta_hat_dot = np.matmul(R, self.nu_hat) + np.matmul(self.L_2, eta_tilde)
        nu_hat_dot = np.matmul(M_inv, (-np.matmul(D, self.nu_hat) + np.matmul(R_inv, self.bias_hat) + np.matmul(R_inv, np.matmul(self.L_3, eta_tilde)) + self.tau))
        bias_hat_dot = np.matmul(self.L_4, eta_tilde)

        # Euler integration:
        self.xsi_hat += xsi_hat_dot*dt
        self.eta_hat += eta_hat_dot*dt
        self.nu_hat += nu_hat_dot*dt
        self.bias_hat += bias_hat_dot*dt
        
        return self.eta_hat, self.nu_hat, self.bias_hat


path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/controller/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)
dt = 1/params["runfrequency"]

linearObserver = Observer(dt)
def loop():
    """
    All calls to functions and methods should be handled inside here. loop() is called by the main-function in obs_node.py
    """
    eta = qualisys.getQualisysOdometry()
    
    L1 = gains.L1
    L2 = gains.L2
    L3 = gains.L3
    L4 = gains.L4
    if ((not np.array_equal(L1, linearObserver.L_1)) or (not np.array_equal(L2, linearObserver.L_2)) or (not np.array_equal(L3, linearObserver.L_3)) or (not np.array_equal(L4, linearObserver.L_4))):
        linearObserver.updateGains(L1, L2, L3, L4)
    
    controlOutput = tau.getTau()
    linearObserver.updateStates(eta, controlOutput)
    eta_hat, nu_hat, bias_hat = linearObserver.observer_linear()
    
    observer.publish(eta_hat, nu_hat, bias_hat)
    # qualisys.publish()
    
    return 0

loop()