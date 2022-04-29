#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import numpy as np
import dynamic_reconfigure.client
from std_msgs.msg import Float64MultiArray
from messages.msg import observer_message, reference_message
class Controller():
    """
    The controller listens to the /joy topic and maps all input signals from the DS4 to a variable that can be called
    """
    def __init__(self):
        self.x = self.square = self.circle = self.triangle = self.rightArrow = self.leftArrow = self.upArrow = self.DownArrow = self.L1 = self.R1 = self.L2 = self.R2 = self.L3 = self.R3 = self.share = self.options = self.PS = self.pad = 0
        self.lStickX = self.lStickY = self.rStickX = self.rStickY = self.L2A = self.R2A = 0.0

    def updateState(self, data):
        self.x = data.buttons[3]
        self.square = data.buttons[0]
        self.circle = data.buttons[2]
        self.triangle = data.buttons[1]
        self.rightArrow = data.buttons[16]
        self.leftArrow = data.buttons[14]
        self.upArrow = data.buttons[15]
        self.DownArrow = data.buttons[17]
        self.L1 = data.buttons[4]
        self.R1 = data.buttons[6]
        self.L2 = data.buttons[5]
        self.R2 = data.buttons[7]
        self.L3 = data.buttons[12]
        self.R3 = data.buttons[13]
        self.options = data.buttons[9]
        self.share = data.buttons[8]
        self.PS = data.buttons[10]
        self.pad = data.buttons[11]

        self.lStickX = -data.axes[0]
        self.lStickY = data.axes[1]
        self.rStickX = -data.axes[2]
        self.rStickY = data.axes[3]
        self.L2A = data.axes[4]
        self.R2A = data.axes[5]
    
class UVector():
    """
    The UVector initializing and publishing the computed actuator commands
    Only used for manual controller
    """
    def __init__(self):
        self.Udata = np.zeros([12, 1])
        # self.pub = rospy.Publisher('/CSAD/u', Float64MultiArray, queue_size = 1)
        self.message = Float64MultiArray()

    def publish(self, data):
        self.message.data = data
        self.pub.publish(self.message)
        
    def callback(self, udata):
        self.Udata = udata
    

class Observer_Converser():
    def __init__(self):
        self.observer_msg = observer_message()
        self.sub = rospy.Subscriber('/CSAD/state_estimate', observer_message, queue_size=1)
        self.eta_hat = np.zeros([3, 1])
        self.nu_hat = np.zeros([3, 1])
        self.bias_hat = np.zeros([3, 1])

    def callback(self, msg):
        self.eta_hat = msg.eta
        self.nu_hat = msg.nu
        self.bias_hat = msg.bias

    def get_observer_data(self):
        return self.eta_hat, self.nu_hat, self.bias_hat

class Reference_Converser():
    """
    The reference converser listens and publishes to the CSEI/reference topic
    """
    # Initialize the guidance parameters in [0; 0; 0]
    def __init__(self):
        self.ref_msg = reference_message()
        self.sub = rospy.Subscriber('/CSAD/reference', reference_message, queue_size=1)
        self.eta_d = np.zeros([3, 1])
        self.eta_ds = np.zeros([3, 1])
        self.eta_ds2 = np.zeros([3, 1])

    # Callback function is called when the topic is updated
    def callback(self, msg):
        self.eta_d = np.array(msg.eta_d)
        self.eta_ds = np.array(msg.eta_ds)
        self.eta_ds2 = np.array(msg.eta_ds2)

    # Publishes new gains to the reference topic. These should be numpy arrays with n=3
    # def publish(self, eta_d, eta_ds, eta_ds2):
    #     self.ref_msg.eta_d = eta_d
    #     self.ref_msg.eta_ds = eta_ds
    #     self.ref_msg.eta_ds2 = eta_ds2
    #     self.pub.publish(self.ref_msg)

    # Retrieve the references from the object 
    def get_ref(self):
        return self.eta_d, self.eta_ds, self.eta_ds2
    
class Controller_Gains():
    """
    Controller gains retrieves the parameters from the dynamic_reconfigure server.
    """
    # Initialize all gains to zero
    def __init__(self):
        self.Kp = np.zeros([3, 3])
        self.Kd = np.zeros([3, 3])
        self.Ki = np.zeros([3, 3])
        self.Kb = np.zeros([3, 3])
        self.mu = 0
        self.Uref = 0

    # Retrieves the gaines 
    def get_data(self):
        return self.Kp, self.Kd, self.Ki, self.Kb, self.mu, self.Uref

    # Updates gains everytime the parameters are tuned
    def callback(self, config):
        self.Kp = self.string2array(config.Kp)
        self.Kd = self.string2array(config.Kd)
        self.Ki = self.string2array(config.Ki)
        self.Kb = self.string2array(config.Kb)
        self.mu = config.mu
        self.Uref = config.U_ref

    # dynamic_reconfigure does not handle arrays, so gains like L1 or KP are strings on the form "x11,x12,x13"
    # the server to limit the number of variables. This function converts 
    # the string into a numpy array when they are retrieved. Very scuffed :^)

    def string2array(self, string): 
        return np.array(list(map(float, string.split(',')))) 


class Tau():
    def __init__(self):
        self.tau_msg = Float64MultiArray()
        self.pub = rospy.Publisher("/CSAD/tau", Float64MultiArray, queue_size=1)
        self.tau = np.zeros([3, 1])
        self.z = np.zeros([3, 1])
        self.mode = False
        self.time = 0.0

    def updateTau(self, msg):
        self.tau = msg.data
        
    def updateIntegralAction(self, z):
        self.z = z
        
    def getIntegralAction(self):
        return self.z
    
    def publish(self, tau):
        self.tau = tau
        self.tau_msg.data = tau
        # self.tau_msg.data[0] = tau[0]
        # self.tau_msg.data[1] = tau[1]
        # self.tau_msg.data[2] = tau[2]
        self.pub.publish(self.tau_msg)
        
    
    def getTau(self):
        return self.tau
    


# Build the objects to be imported
ps4 = Controller()
u_data = UVector() #used for manual controller
odometry = Odometry()
observer = Observer_Converser()
gains = Controller_Gains()
reference = Reference_Converser()
tau = Tau()


# Initialize controller node
def controllNodeInit():
    global node
    node = rospy.init_node('Controller_node')
    rospy.Subscriber("/joy", Joy, ps4.updateState)
    rospy.Subscriber("/CSAD/state_estimate", observer_message, observer.callback)
    # rospy.Subscriber("/CSAD/reference", reference_message, reference.callback)
    gain_client = dynamic_reconfigure.client.Client('gain_server', timeout=30, config_callback = gains.callback)
    

# Destroy node when prompted
def nodeEnd():
    global node
    node.destroy_node()

