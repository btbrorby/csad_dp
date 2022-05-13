#!/usr/bin/env python
import rospy
import numpy as np
import math
from lib import tau, u_data
from math_tools import Rzyx, rad2pipi
import os
import yaml

path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/thrust_allocation/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)

def ssa(angle):
    angle = ((angle + np.math.pi) % (2*np.math.pi)) - np.math.pi
    return angle

def thrustAllocationExtended(tau_d):
    
        
    lx = params['thruster_coefficients']['lx']
    ly = params['thruster_coefficients']['ly']
    K_vec = params['thruster_coefficients']['K']
    
    K_e = np.diag([K_vec[0], K_vec[0], K_vec[1], K_vec[1], K_vec[2], K_vec[2], K_vec[3], K_vec[3], K_vec[4], K_vec[4], K_vec[5], K_vec[5]])
    T_e = np.array([[1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0],
                    [0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0],
                    [-ly[0], lx[0], -ly[1], lx[1], -ly[2], lx[2], -ly[3], lx[3], -ly[4], lx[4], -ly[5], lx[5]]])
    K_e_inv = np.linalg.inv(K_e)
    T_e_transpose = np.transpose(T_e)
    T_e_pseudoInv = np.matmul(T_e_transpose, np.linalg.inv(np.matmul(T_e, T_e_transpose)))
    
    u_e = np.matmul(np.matmul(K_e_inv, T_e_pseudoInv), tau_d)
    
    u_1 = np.math.sqrt(u_e[0]**2 + u_e[1]**2)
    u_2 = np.math.sqrt(u_e[2]**2 + u_e[3]**2)
    u_3 = np.math.sqrt(u_e[4]**2 + u_e[5]**2)
    u_4 = np.math.sqrt(u_e[6]**2 + u_e[7]**2)
    u_5 = np.math.sqrt(u_e[8]**2 + u_e[9]**2)
    u_6 = np.math.sqrt(u_e[10]**2 + u_e[11]**2)
    
    alpha_1 = ssa(np.math.atan2(u_e[1], u_e[0]))
    alpha_2 = ssa(np.math.atan2(u_e[3], u_e[2]))
    alpha_3 = ssa(np.math.atan2(u_e[5], u_e[4]))
    alpha_4 = ssa(np.math.atan2(u_e[7], u_e[6]))
    alpha_5 = ssa(np.math.atan2(u_e[9], u_e[8]))
    alpha_6 = ssa(np.math.atan2(u_e[11], u_e[10]))
    
    u = np.array([u_1, u_2, u_3, u_4, u_5, u_6, alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_6])
    u = np.resize(u, (len(u), 1))
    
    return u


### End of student code ###

def loop():
    """
    Calling thrust allocation algorithm
    """
    tau_d = tau.getTau()
    u = thrustAllocationExtended(tau_d)
    u_data.publish(u)
