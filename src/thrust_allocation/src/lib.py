from math import tau
from math_tools import quat2eul, Rzyx
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
import numpy as np


class UVector():
    def __init__(self):
        self.Udata = np.zeros([12, 1])
        self.pub = rospy.Publisher('/CSAD/u', Float64MultiArray, queue_size=1)
        self.u_message = Float64MultiArray()

    def getU(self):
        return self.Udata 
    
    def publish(self, u):
        self.u_message.data = u
        self.pub.publish(self.u_message)
        


class Tau():
    def __init__(self):
        self.tau = np.array([0, 0, 0])

    def updateTau(self, msg):
        self.tau = msg.data
    
    def getTau(self):
        return self.tau



u_data = UVector()
tau  = Tau()

# Initialize observer node
def thrusterNodeInit():
    global node
    node = rospy.init_node('Allocation_node')
    rospy.Subscriber("/CSAD/tau", Float64MultiArray, Tau.updateTau)
    
   
# Destroy observer node
def nodeEnd():
    node.destroy_node()
