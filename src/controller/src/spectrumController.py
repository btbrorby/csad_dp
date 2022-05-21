#!/usr/bin/env python
import rospy
import numpy as np
from lib import observer, reference, ps4, u_data, gains, tau, waveMeasurement
from math_tools import Rzyx, rad2pipi
from matplotlib import pyplot as plt
import yaml
import os


class SpectrumController:
    def __init__(self, dt=0.02):
        self.dt = dt
        self.time = 0.0
        
        self.shiftRegister = np.array([])
        self.S_hat = np.array([])
        
        self.waveMeasurements = []
        
        #Tuning parameters:
        self.N = 10
        self.spectrumStep = 60
        self.Kp = np.diag([1.0, 1.0, 1.0])
        self.Kd = np.diag([1.0, 1.0, 1.0])
        self.Ki = np.diag([1.0, 1.0, 1.0])
        
        
    def regMeasurement(self, measurement):
        self.waveMeasurements.append(measurement)
        
    def updateSpectrum(self):
        savedMeasurements = self.waveMeasurements.copy()
        
        currentSpectrum = np.fft.fft(savedMeasurements, self.N)
        
        self.shiftRegister = np.resize(self.shiftRegister, (np.shape(self.shiftRegister)[0]+1, len(currentSpectrum)))
        
        
        
        
        
path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/controller/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)
dt = 1.0/params["runfrequency"]

controller = SpectrumController(dt, N = 10)
def loop():
    
    eta_hat = observer.eta_hat
    nu_hat = observer.nu_hat
    bias_hat = observer.bias_hat
    
    measurement = waveMeasurement.getMeasurement()
    controller.regMeasurement(measurement)
    
    if controller.time % controller.spectrumStep == 0:
        controller.updateSpectrum()
    
    