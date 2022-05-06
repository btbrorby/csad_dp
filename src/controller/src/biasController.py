#!/usr/bin/env python
import numpy as np
from lib import odometry, observer, reference, ps4, u_data, gains, tau
from math_tools import Rzyx, rad2pipi
from matplotlib import pyplot as plt
import yaml
import os

import rospy

class BiasController:
    def __init__(self, dt):
        self.eta_bias = self.eta_hat = self.nu_hat = np.zeros([3,1])
        self.eta_ref = self.nu_ref = np.zeros([3,1])
        self.dt = dt
        self.Kp = np.diag([1.0, 1.0, 1.0])
        self.Kd = 0.0*np.diag([1.0, 1.0, 1.0])
        self.Kb = 0.0*np.diag([1.0, 1.0, 1.0])
        
    def updateStates(self, eta_hat, nu_hat, bias_hat, eta_ref, nu_ref):
        self.eta_hat = eta_hat
        self.nu_hat = nu_hat
        self.bias_hat = bias_hat
        self.eta_ref = eta_ref
        self.nu_ref = nu_ref
        
    def updateGains(self, Kp, Kd, Kb):
        self.Kp = Kp
        self.Kb = Kb
        self.Kd = Kd
        
    def getControlOutput(self):
        """
        Bias estimate controller, uses the bias estimate provided from the observer directly
        """
        # self.eta_hat[2] = rad2pipi(self.eta_hat[2]) #Is this right?
        R = Rzyx(self.eta_hat[2])
        R_transpose = np.transpose(R)
        
        eta_tilde = self.eta_hat - self.eta_ref
        nu_tilde = self.nu_hat - self.nu_ref
        
        controlOutput = -np.matmul(self.Kp, np.matmul(R_transpose, eta_tilde)) - np.matmul(self.Kd, nu_tilde) - np.matmul(self.Kb, self.bias_hat)
        
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
    # eta_ref = reference.eta_d
    # nu_ref = reference.eta_ds #Is this right?
    eta_ref = np.zeros([3,1])
    nu_ref = np.zeros([3,1])
    
    # Kp = gains.Kp
    # Kd = gains.Kd
    # Kb = gains.Kb
    # if ((not np.array_equal(Kp, controller.Kp)) or (not np.array_equal(Kd, controller.Kd)) or (not np.array_equal(Kb, controller.Kb))):
    #     controller.updateGains(Kp, Kb, Kd)
        
    controller.updateStates(eta_hat, nu_hat, bias_hat, eta_ref, nu_ref)
    controlOutput = controller.getControlOutput()
    tau.publish(controlOutput)
    
    tau.time += dt
    # rospy.spin()
   