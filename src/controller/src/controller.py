#!/usr/bin/env python3
from unittest import case
#from lib import Qualisys
import rospy
import numpy as np
import math
from lib import odometry, observer, reference, ps4, u_data, gains, tau
#from src.lib import tau
from math_tools import Rzyx, rad2pipi
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray




### Write your code here ###


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



def pid(eta_hat, nu_hat, bias_hat, z, eta_ref, nu_ref, Kp, Ki, Kd):
    """
    PID controller
    """
    
    #Check if circle has been pressed, then turn on/off integral action
    
    # Kp = np.diag([0.0, 0.0, 0.0])
    # Ki = np.diag([0.0, 0.0, 0.0])
    # Kd = np.diag([0.0, 0.0, 0.0])
    
    #eta_hat[2] = rad2pipi(eta_hat[2]) #Is this right?
    R = Rzyx(eta_hat[2])
    R_inv = np.transpose(R)
    dt = 0.01 #Should be depending on yaml file!!!
    
    eta_hat_dot = R @ nu_hat
    z = z + dt*eta_hat_dot
    
    eta_tilde = eta_hat - eta_ref
    nu_tilde = nu_hat - nu_ref
    
    tau_d = Kp @ R_inv @ eta_tilde + Kd @ nu_tilde + Ki @ R_inv @ z
    
    tau_d = 1
    
    return tau_d, z


def biasDP(eta_hat, nu_hat, bias_hat, eta_ref, nu_ref, Kp, Kb, Kd):
    """
    Bias estimate controller
    """
    eta_hat[2] = rad2pipi(eta_hat[2]) #Is this right?
    R = Rzyx(eta_hat[2])
    R_inv = np.transpose(R)
    dt = 0.01 #Should be depending on yaml file!!!
    
    eta_tilde = eta_hat - eta_ref
    nu_tilde = nu_hat - nu_ref
    
    tau_d = R_inv @ Kp @ eta_tilde + Kd @ nu_tilde + Kb @ bias_hat
    
    return tau_d

### End of custom code
    
def loop():


    eta_hat = observer.eta_hat
    nu_hat = observer.nu_hat
    bias_hat = observer.bias_hat
    
    eta_ref = reference.eta_d
    nu_ref = reference.eta_ds #Is this right?
    eta_ref = np.array([0.0, 0.0, 0.0])
    nu_ref = np.array([0.0, 0.0, 0.0])
    
    
    Kp = gains.Kp
    Ki = gains.Ki
    Kd = gains.Kd
    Kb = gains.Kb
    
    z = tau.getIntegralAction()
    
    mode = tau.mode
    
    # if CIRCLE is pressed, mode shift between manual control and dp control
    # if X is pressed, thrusters is set to zero
    if (ps4.circle == 1 and mode == True) or (ps4.circle == 0 and mode == False):
        u = sixaxis2thruster(ps4.lStickX, ps4.lStickY, ps4.rStickX, ps4.rStickY, ps4.x)
        u_data.publish(u)
        mode = False
    elif (ps4.circle == 1 and mode == False) or (ps4.circle == 0 and mode == True):
        tau_d, z = pid(eta_hat, nu_hat, bias_hat, z, eta_ref, nu_ref, Kp, Ki, Kd)
        #tau_d = biasDP(eta_hat, nu_hat, bias_hat, eta_ref, nu_ref, Kp, Kb, Kd)
        tau.updateIntegralAction(z)
        tau.publish(tau_d)
        mode = True
    
    """
    All calls to functions and methods should be handled inside here. loop() is called by the main-function in ctrl_node.py
    """
  