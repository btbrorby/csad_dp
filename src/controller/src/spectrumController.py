#!/usr/bin/env python
from unicodedata import name

from scipy.fftpack import fft
import rospy
import numpy as np
import decimal
from scipy import signal
from lib import observer, reference, ps4, u_data, gains, tau, waveMeasurement
from math_tools import Rzyx, rad2pipi
from matplotlib import pyplot as plt
import yaml
import os
import generateModelData_CSAD as data
class SpectrumController:
    def __init__(self, dt=0.02, eta_d=np.zeros([3,1]), eta_d_dot=np.zeros([3,1]), eta_d_dotdot=np.zeros([3,1])):
        self.dt = dt
        self.time = 0.0
        
        self.shiftRegister = np.array([])
        self.S_hat = np.array([])
        self.S_hat_Index = []
        self.QTF_index = []
        
        self.waveMeasurements = []
        
        self.eta_d = eta_d
        self.eta_d_dot = eta_d_dot
        self.eta_d_dotdot = eta_d_dotdot
        self.z = np.zeros([3,1])
        
        self.isUpdated = True
        
        #Tuning parameters:
        self.N = 10 #not in use
        self.spectrumStep = 300
        self.gamma = 0.5
        self.Kp = 2.0*np.pi*2.0*np.pi*np.diag([1.8982, 197.0, 0.0])
        self.Ki = np.diag([25.0, 0.0, 0.0])
        self.Kd = np.diag([204.8, 0.0, 0.0])
        
        self.dw = np.abs(data.driftForceFreq[0] - data.driftForceFreq[-1]/len(data.driftForceFreq))
        
        
    def registerMeasurement(self, measurement):
        self.waveMeasurements.append(measurement)

    def updateSpectrum(self):
        savedMeasurements = np.array(self.waveMeasurements[:])
        
        f, currentSpectrum = signal.welch(savedMeasurements, (1.0/self.dt)/2.0)
        f *= 2.0*np.pi
        print(np.shape(f), np.shape(currentSpectrum))
        #Finding the relevant indecies that we have data for:
        freqIndex = []
        for i in data.driftForceFreq:
            if len(freqIndex) == 0:
                freqIndex.append(np.argmin(np.abs(f - i)))
            elif freqIndex[-1] != np.argmin(np.abs(f - i)):
                freqIndex.append(np.argmin(np.abs(f - i)))
        
        self.S_hat_Index = freqIndex[:]    
        
        self.shiftRegister = np.resize(self.shiftRegister, (np.shape(self.shiftRegister)[0]+1, len(currentSpectrum)))
        self.shiftRegister[-1] = currentSpectrum
        
        self.estimateSpectrum()
        self.isUpdated = True
        
        return f, currentSpectrum
        
    def estimateSpectrum(self):
        num = np.zeros([len(self.shiftRegister[0])])
        denum = 0.0
        j = 0.0
        for i in self.shiftRegister:
            num += i*(np.power(self.gamma, j))
            denum += self.gamma**j
            j += 1.0
            
        self.S_hat = np.resize(self.S_hat, (len(self.S_hat), 1))
        self.S_hat = num/denum
            
    def getControllerOutput(self, eta_hat, nu_hat):
        headingIndex = np.argmin(np.abs(data.headingsData - eta_hat[2]))
        
        T = data.driftForceAmpX[:, headingIndex]
        
        # tau_wave = 2.0*self.S_hat[self.S_hat_Index]*driftForceAmpX[]
        
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

fig = plt.figure()

f = []
spec = []
waves = []
controller = SpectrumController(dt)
def loop():
    
    eta_hat = observer.eta_hat
    nu_hat = observer.nu_hat
    eta_hat = np.resize(eta_hat, (3,1))
    nu_hat = np.resize(nu_hat, (3,1))
    
    measurement = waveMeasurement.getMeasurement()
    waves.append(measurement)
    controller.registerMeasurement(measurement)
    
    mod = round(np.math.fmod(controller.time, controller.spectrumStep), 1)
    if (mod == 0.0) and (controller.isUpdated == False):
        [f, spec] = controller.updateSpectrum()
        
        plt.plot(f, np.abs(spec))
        
        plt.show()
        

    if mod == 1.0:
        controller.isUpdated = False
    
    controllerOutput = controller.getControllerOutput(eta_hat, nu_hat)
    tau.publish(controllerOutput, controller.time)
    controller.time += dt
    print(controller.time)



