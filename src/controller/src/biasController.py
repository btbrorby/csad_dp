#!/usr/bin/env python
import numpy as np
from lib import odometry, observer, reference, ps4, u_data, gains, tau
from math_tools import Rzyx, rad2pipi
from matplotlib import pyplot as plt
import yaml
import os
import time as tic
import rospy

class BiasController:
    def __init__(self,  dt=0.02, eta_d=5.0*np.ones([3,1]), eta_d_dot=np.zeros([3,1]), eta_d_dotdot=np.zeros([3,1])):
        self.eta_d = eta_d
        self.eta_d[2] = rad2pipi(self.eta_d[2])
        self.eta_d_dot = eta_d_dot
        self.eta_d_dotdot = eta_d_dotdot
        self.dt = dt
        self.Kp = np.diag([30.0, 0.0, 0.0])
        self.Kd = np.diag([1.0, 0.0, 0.0])
        
    def updateGains(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd
        
    def getControlOutput(self, eta_hat, nu_hat, bias_hat):
        """
        Bias estimate controller, uses the bias estimate provided from the observer directly
        """
        R = Rzyx(eta_hat[2])
        R_transpose = np.transpose(R)
        
        nu_d = np.matmul(R_transpose, self.eta_d_dot)
        eta_tilde = eta_hat - self.eta_d
        nu_tilde = nu_hat - nu_d
        
        controlOutput = -np.matmul(self.Kp, np.matmul(R_transpose, eta_tilde)) - np.matmul(self.Kd, nu_tilde) - 0.0*np.matmul(R_transpose, bias_hat)
        controlOutput = np.resize(controlOutput, (3,1))
        return controlOutput
        
        
     
path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/controller/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)
dt = 1.0/params["runfrequency"]

controller = BiasController(dt)
def loop():
    eta_hat = observer.eta_hat
    nu_hat = observer.nu_hat
    bias_hat = observer.bias_hat
    eta_hat = np.resize(eta_hat, (3,1))
    nu_hat = np.resize(nu_hat, (3,1))
    bias_hat = np.resize(bias_hat, (3,1))
    
    controlOutput = controller.getControlOutput(eta_hat, nu_hat, bias_hat)
    tau.publish(controlOutput, tau.time)
    
    tau.time += dt
   