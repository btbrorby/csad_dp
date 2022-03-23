#!/usr/bin/env python3
import this
import rospy
import numpy as np
import math
from lib import qualisys, tau, observer, gains
from math_tools import Rzyx, rad2pipi

### Write your code here ###
def observer_linear(eta_hat, nu_hat, bias_hat, eta, tau, gains):
    """
    Observer
    """
    L_1 = gains.L1
    L_2 = gains.L2
    L_3 = gains.L3
    L_4 = gains.L4
    
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
    
    eta[2] = rad2pipi(eta[2]) #Is this right?
    R = Rzyx(eta[2])
    R_inv = np.transpose(R)
    dt = 0.01 #Should be depending on yaml file!!!
    
    eta_tilde = eta - eta_hat
    
    # Observer dynamics:
    #xsi_hat_dot = A_w @ zeta_hat + L_1 @ y_tilde
    eta_hat_dot = R @ nu_hat + L_2 @ eta_tilde
    nu_hat_dot = M_inv @ (-D @ nu_hat + R_inv @ bias_hat + R_inv @ L_3 @eta_tilde + tau)
    bias_hat_dot = L_4 @ eta_tilde

    # Euler integration:
    #xsi_hat = zeta_hat + zeta_dot*dt
    eta_hat = eta_hat + eta_hat_dot * dt
    nu_hat = nu_hat + nu_hat_dot * dt
    bias_hat = bias_hat + bias_hat_dot * dt
    
    return eta_hat, nu_hat, bias_hat


### End of student code ###

def loop():
    """
    All calls to functions and methods should be handled inside here. loop() is called by the main-function in obs_node.py
    """
    
    eta = qualisys.getQualisysOdometry()
    eta_hat_previous, nu_hat_previous, bias_hat_previous = observer.get_observer_data()
    eta_hat, nu_hat, bias_hat = observer_linear(eta_hat_previous, nu_hat_previous, bias_hat_previous, eta, tau.getTau(), gains)
    
    observer.publish(eta_hat, nu_hat, bias_hat)
    qualisys.publish()
    
    return 0