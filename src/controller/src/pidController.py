#!/usr/bin/env python
import rospy
import numpy as np
import math
from lib import observer, reference, ps4, u_data, gains, tau
from math_tools import Rzyx, rad2pipi
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import yaml
import os




def saturate(u):
    if u < -0.5:
        u = -0.5
    if u > 0.5:
        u = 0.5
    return u

def sixaxis2thruster(lStickX, lStickY, rStickX, rStickY, xbutton):
    """
    sixaxis2thruster controlls enables the user to control the from actuators with the left ps4 stick, 
    and the back thrusters with the right thrusters
    """
    u = np.zeros(12)
    

    ### Front actuators ###

    u[0:3] = saturate(math.sqrt(lStickY ** 2 + lStickX ** 2)) # Input
    for i in range(3):
        if u[1] > 0.005:
            u[6:9] = math.atan2(lStickY, -lStickX) # Angle



    ### Back actuators ###
    u[3:6] = saturate(math.sqrt(rStickY**2 + rStickX**2))
    for i in range(3):
        if u[4] > 0.005:
            u[9:12] = math.atan2(rStickY, -rStickX)
    
    if xbutton == 1:
        u[0:6] = 0

    return u


class PID:
    def __init__(self, dt=0.02, eta_d=np.zeros([3,1]), eta_d_dot=np.zeros([3,1]), eta_d_dotdot=np.zeros([3,1])):
        self.eta_d = eta_d
        self.eta_d_dot = eta_d_dot
        self.eta_d_dotdot = eta_d_dotdot
        self.z = np.zeros([3,1])
        self.dt = dt
        self.Kp = 2.0*np.pi*2.0*np.pi*np.diag([1.8982, 197.0, 0.0])
        self.Ki = np.diag([25.0, 0.0, 0.0])
        self.Kd = np.diag([204.8, 0.0, 0.0])
        
        
    def getControllerOutput(self, eta_hat, nu_hat):
        """
        PID controller
        """
        R = Rzyx(eta_hat[2])
        R_inv = np.transpose(R)
        
        eta_hat_dot = np.matmul(R, nu_hat)
        eta_hat_dot = np.resize(eta_hat_dot, (3,1))
        
        z_dot = eta_hat - self.eta_d
        self.z += z_dot*self.dt
        self.z[2] = rad2pipi(self.z[2])
        
        nu_d = np.matmul(R_inv, self.eta_d_dot)
                
        eta_tilde = eta_hat - self.eta_d
        nu_tilde = nu_hat - nu_d
        
        controlOutput = - np.matmul(self.Kp, np.matmul(R_inv, eta_tilde)) - np.matmul(self.Kd, nu_tilde) - np.matmul(self.Ki, np.matmul(R_inv, self.z))
        controlOutput = np.resize(controlOutput, (3,1))
        return controlOutput


path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/controller/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)

dt = 1.0/params["runfrequency"]

controller = PID(dt)
    
def loop():

    eta_hat = observer.eta_hat
    nu_hat = observer.nu_hat
    bias_hat = observer.bias_hat
    eta_hat = np.resize(eta_hat, (3,1))
    nu_hat = np.resize(nu_hat, (3,1))
    bias_hat = np.resize(bias_hat, (3,1))
    
    controllerOutput = controller.getControllerOutput(eta_hat, nu_hat)
    
    tau.publish(controllerOutput, tau.time)
    tau.time += dt
    
    # if CIRCLE is pressed, mode shift between manual control and dp control
    # if X is pressed, thrusters is set to zero
    # if (ps4.circle == 1 and mode == True) or (ps4.circle == 0 and mode == False):
    #     u = sixaxis2thruster(ps4.lStickX, ps4.lStickY, ps4.rStickX, ps4.rStickY, ps4.x)
    #     u_data.publish(u)
    #     mode = False
    # elif (ps4.circle == 1 and mode == False) or (ps4.circle == 0 and mode == True):
    #     tau_d, z = pid(eta_hat, nu_hat, bias_hat, z, eta_ref, nu_ref, Kp, Ki, Kd)
    #     #tau_d = biasDP(eta_hat, nu_hat, bias_hat, eta_ref, nu_ref, Kp, Kb, Kd)
    #     tau.updateIntegralAction(z)
    #     tau.publish(tau_d)
    #     mode = True
    
    """
    All calls to functions and methods should be handled inside here. loop() is called by the main-function in ctrl_node.py
    """
  