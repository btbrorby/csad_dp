#!/usr/bin/env python
from pydoc import importfile
from math_tools import quat2eul, Rzyx
import rospy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from messages.msg import observer_message, regulator_message
import numpy as np
import dynamic_reconfigure.client

class Qualisys():
    """
    Retrieves qualisys measurements by listening to the /qualisys/CSEI/odom topic
    """
    def __init__(self):
        self.odom = Odometry()
        self.eta = np.zeros([3,1])
        
        #For visualization
        self.true_pos_msg = Vector3()
        self.true_pos_pub = rospy.Publisher('/CSAD/true_position', Vector3, queue_size=1)

    def updateQualisysOdometry(self, data=Odometry()):
        self.odom = data

    def getQualisysOdometry(self):
        w = self.odom.pose.pose.orientation.w
        x = self.odom.pose.pose.orientation.x
        y = self.odom.pose.pose.orientation.y
        z = self.odom.pose.pose.orientation.z

        self.eta[0] = self.odom.pose.pose.position.x
        self.eta[1] = self.odom.pose.pose.position.y
        self.eta[2] = quat2eul(w, x, y, z)[2]
        
        return self.eta
    
    def publish(self):
        self.true_pos_msg.x = self.eta[0]
        self.true_pos_msg.y = self.eta[1]
        self.true_pos_msg.z = self.eta[2]
        self.true_pos_pub.publish(self.true_pos_msg)



class Tau():
    def __init__(self):
        self.tau = np.zeros([3,1])

    def callback(self, msg=Float64MultiArray()):
        self.tau[0] = msg.data[0]
        self.tau[1] = msg.data[1]
        self.tau[2] = msg.data[2]
    
    def getTau(self):
        return self.tau

class Observer_Converser():
    def __init__(self):
        self.observer_msg = observer_message()
        self.pub = rospy.Publisher('/CSAD/state_estimate', observer_message, queue_size=10)
        self.eta_hat = np.zeros([3,1])
        self.nu_hat = np.zeros([3,1])
        self.bias_hat = np.zeros([3,1])
        
        #For visualization
        self.xyz_msg = Vector3()
        self.xyzPub = rospy.Publisher('/CSAD/xy_estimate', Vector3, queue_size=10)
        


    def callback(self, msg):
        self.eta_hat = msg.eta
        self.nu_hat = msg.nu
        self.bias_hat = msg.bias

    def publish(self, eta_hat, nu_hat, bias_hat, time):
        self.observer_msg.eta = eta_hat
        self.observer_msg.nu = nu_hat
        self.observer_msg.bias = bias_hat
        self.observer_msg.time = time
        self.pub.publish(self.observer_msg)
        
        self.xyz_msg.x = eta_hat[0]
        self.xyz_msg.y = eta_hat[1]
        self.xyz_msg.z = eta_hat[2]
        self.xyzPub.publish(self.xyz_msg)

    def get_observer_data(self):
        return self.eta_hat, self.nu_hat, self.bias_hat


class Observer_Gains():
    def __init__(self):
        self.L1 = np.zeros([3,3])
        self.L2 = np.zeros([3,3])
        self.L3 = np.zeros([3,3])
        self.L4 = np.zeros([3,3])
        

    def get_observer_gains(self):
        return self.L1, self.L2, self.L3, self.L4

    def callback(self, config):
        self.L1 = self.string2array(config.L1)
        self.L2 = self.string2array(config.L2)
        self.L3 = self.string2array(config.L3)
        self.L4 = self.string2array(config.L4)       
       

    def string2array(self, string):
        return np.array(list(map(float, string.split(',')))) # Suuuuuuuuper scuffed

qualisys = Qualisys()
observer = Observer_Converser()
gains = Observer_Gains()
tau  = Tau()

# Initialize observer node
def observerNodeInit():
    global node
    node = rospy.init_node('Observer_node')
    rospy.Subscriber("/qualisys/Body_1/odom", Odometry, callback=qualisys.updateQualisysOdometry) #check rostopic list for actual name on odom message!
    rospy.Subscriber("/CSAD/tau", Float64MultiArray, callback=tau.callback)
    gain_client = dynamic_reconfigure.client.Client('gain_server', timeout=30, config_callback = gains.callback)
    
    
   
# Destroy observer node
def nodeEnd():
    node.destroy_node()
