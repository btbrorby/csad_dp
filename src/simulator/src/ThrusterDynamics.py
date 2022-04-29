#!/usr/bin/env python
import numpy as np
import math_tools
import generateModelData_CSAD as data
import math

from std_msgs.msg import Float64MultiArray

import yaml
import os

path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/simulator/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)
    
global timeStep
timeStep = 1.0/params["runfrequency"]
      
#This class should maybe be in separate python file?
class ThrusterDynamics:
    def __init__(self, u=np.zeros([12,1]), dt=timeStep):
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
        
        
    def actualThrustLoads(self, u, alpha):
        """
        Calcluates the actual load generated by the azimuth thrusters in body frame.
        The thrust dynamics are saturated and the rates are limited.
        Calculates the load from each actuator by F_i=rho*D^4*K_T*abs(n)*n
        Returns:
            self.loads: Actual thrust loads (body frame) generated by the resulting loads from all actuators.
        """
        
        #Calculates the propeller revolution based on desired actuator inputs:
        n = self.propellerRevolution(u)
        
        #Limits the signal rates based on specified limitations:
        alpha_actual = self.rateLimiter(alpha, self.alpha_previous, self.alpha_dot_max)
        n_actual = self.rateLimiter(n, self.n_previous, self.n_dot_max)
        #Calculates the actual actuator loads for each actuator:
        actuatorLoads = data.rho*(data.propellerDiameter**4)*(data.K * np.abs(n) * n)
        
        #Saturates the actuator loads based on specified limitations:
        actuatorLoads = self.saturate(actuatorLoads, -data.thrust_max, data.thrust_max)
        
        #Summerize contributions from each eactuator and converts them to specific loads in body frame:
        loads = np.zeros(6)
        for i in range(np.size(actuatorLoads)):
            loads[0] += actuatorLoads[i]*math.cos(alpha[i])
            loads[1] += actuatorLoads[i]*math.sin(alpha[i])
            loads[5] += data.Lx[i]*actuatorLoads[i]*math.sin(alpha[i]) - data.Ly[i]*actuatorLoads[i]*math.cos(alpha[i])
            
        #Stores previous signals. Used for limit the signal rates:
        self.alpha_previous = alpha_actual 
        self.n_previous = n 
        
        return loads, n_actual, alpha_actual
    
    
    def propellerRevolution(self, u):
        """Calculates the propeller revolution, depending on the control input u."""
        n = math_tools.sign(u)*math_tools.sqrt(np.abs(u))
        return n
        
    def saturate(self, signal, min, max):
        count = 0
        for s in signal:
            if signal[count] < min:
                signal[count] = min
            elif signal[count] > max:
                signal[count] = max
        
        return signal
    
    def rateLimiter(self, signal, signalPrevious, limit):
        """
        Limits the rates of the signal.

        Args:
            signal (_type_): The signal that needs to be limited
            signalPrevious (_type_): A copy of the previous signal
            limit (_type_): The specified rate limit
        """
        count = 0
        for s in signal:
            plussMinus = math_tools.sign(signal[count]-signalPrevious[count]) #negative if signal is decreasing, positive if increasing
            if (plussMinus*(signal[count]-signalPrevious[count])/self.dt > limit):
                signal[count] = signalPrevious[count] + plussMinus*limit*self.dt
            count += 1
        return signal
    
    # Implement a low pass filter for smoothing thrust dynamics (angle shift and setpoint shift).
    # n_dot <= 5 [1/s^2] propeller acceleration 5 revolutions pr s^2
    # alpha_dot <= 2 [1/s] propeller rotation speed
    # Saturate min and max values of thrust (max thrust is 1.5[N])
        
    def getThrustLoads(self):
        [self.loads, n_actual, alpha_actual] = self.actualThrustLoads(self.u, self.alpha)
        return self.loads
    
    def updateU(self, controlInput):
        self.u = controlInput.data[0:6]
        self.alpha = controlInput.data[6:12]

    
