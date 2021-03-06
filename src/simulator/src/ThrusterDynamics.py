#!/usr/bin/env python
from time import sleep
import numpy as np
import rospkg
import math_tools
import generateModelData_CSAD as data
import math
import rospy

from std_msgs.msg import Float64MultiArray


#This class should maybe be in separate python file?
class ThrusterDynamics:
    def __init__(self, u=np.zeros([12,1]), dt=0.02):
        #Initialize thruster dynamics
        self.loads = np.zeros([6,1])
        self.u = u[0:6]
        self.alpha = u[6:12]
        self.n = np.zeros([6,1])
        self.dt = dt
        
        self.alpha_previous = np.zeros([6,1])
        self.n_previous = np.zeros([6,1])
        
        #Thruster konfiguration
        self.Lx = data.Lx
        self.Ly = data.Ly
        self.K = data.K
        
        #Limitations and saturations
        self.alpha_dot_max = data.alpha_dot_max
        self.n_dot_max = data.n_dot_max
        
        self.msg_u = Float64MultiArray()
        
        self.pub = rospy.Publisher('/CSAD/trueThrustLoads', Float64MultiArray, queue_size=1)
        
        
    def actualThrustLoads(self, u, alpha):
        """
        Calcluates the actual load generated by the azimuth thrusters in body frame.
        The thrust dynamics are saturated and the rates are limited.
        Calculates the load from each actuator by F_i=rho*D^4*K_T*abs(n)*n
        Returns:
            self.loads: Actual thrust loads (body frame) generated by the resulting loads from all actuators.
        """
        Lx = np.resize(self.Lx, (len(self.Lx), 1))
        Ly = np.resize(self.Ly, (len(self.Ly), 1))
        #Calculates the propeller revolution based on desired actuator inputs:
        n = self.propellerRevolution(u)
        n = np.resize(n, (len(n), 1))

        #Limits the signal rates based on specified limitations:
        alpha_actual = self.rateLimiter(alpha, self.alpha_previous, self.alpha_dot_max*self.dt)
        n_actual = self.rateLimiter(n, self.n_previous, self.n_dot_max*self.dt)
        #Calculates the actual actuator loads for each actuator:
        # actuatorLoads1 = data.rho*(data.propellerDiameter**4)*(np.matmul(data.K, np.abs(n)*n))
        actuatorLoads = np.matmul(data.K, n_actual*np.abs(n_actual))
        
        #Saturates the actuator loads based on specified limitations:
        actuatorLoads = self.saturate(actuatorLoads, -data.thrust_max, data.thrust_max)
        
        #Summerize contributions from each eactuator and converts them to specific loads in body frame:
        loads = np.zeros([6,1])
        loads[0] = np.sum(actuatorLoads*np.cos(alpha))
        loads[1] = np.sum(actuatorLoads*np.sin(alpha))
        loads[5] = np.sum(Lx*actuatorLoads*np.sin(alpha)) - np.sum(Ly*actuatorLoads*np.cos(alpha))
        
        #Stores previous signals. Used for limit the signal rates:
        self.alpha_previous = alpha_actual 
        self.n_previous = n 
        return loads, n_actual, alpha_actual
    
    
    def propellerRevolution(self, u):
        """Calculates the propeller revolution, depending on the control input u."""
        n = math_tools.sign(u)*math_tools.sqrt(np.abs(u))
        return n
        
    def saturate(self, signal, min, max):
        signal = np.resize(signal, (len(signal),1))
        count = 0
        for s in signal:
            if signal[count] < min:
                signal[count] = min
            elif signal[count] > max:
                signal[count] = max
        signal = np.resize(signal, (len(signal),1))
        return signal
    
    def rateLimiter(self, signal, signalPrevious, limit):
        """
        Limits the rates of the signal.

        Args:
            signal (_type_): The signal that needs to be limited
            signalPrevious (_type_): A copy of the previous signal
            limit (_type_): The specified rate limit
        """
        signal = np.resize(signal, (len(signal),1))
        signalPrevious = np.resize(signalPrevious, (len(signalPrevious),1))
        count = 0
        for s in signal:
            plussMinus = math_tools.sign(signal[count]-signalPrevious[count]) #negative if signal is decreasing, positive if increasing
            if (plussMinus*(signal[count]-signalPrevious[count])/self.dt > limit):
                signal[count] = signalPrevious[count] + plussMinus*limit*self.dt
            count += 1
        signal = np.resize(signal, (len(signal), 1))
        return signal
    
    # Implement a low pass filter for smoothing thrust dynamics (angle shift and setpoint shift).
    # n_dot <= 5 [1/s^2] propeller acceleration 5 revolutions pr s^2
    # alpha_dot <= 2 [1/s] propeller rotation speed
    # Saturate min and max values of thrust (max thrust is 1.5[N])
        
    def getThrustLoads(self):
        [self.loads, n_actual, alpha_actual] = self.actualThrustLoads(self.u, self.alpha)
        return self.loads
    
    def updateU(self, controlInput = Float64MultiArray()):
        self.u = controlInput.data[0:6]
        self.u = np.resize(self.u, (6, 1))
        self.alpha = controlInput.data[6:12]
        self.alpha = np.resize(self.alpha, (6, 1))
        
    def publish(self, loads):
        msg=Float64MultiArray()
        msg.data = loads
        self.pub.publish(msg)
        

    