import rospy
import math
import numpy as np
from lib import ps4, u_data
from std_msgs.msg import Float64MultiArray


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

def loop():
    u = sixaxis2thruster(ps4.lStickX, ps4.lStickY, ps4.rStickX, ps4.rStickY, ps4.x)
    u_data.publish(u)
