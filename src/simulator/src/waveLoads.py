#!/usr/bin/env python

from cmath import phase, pi
from turtle import heading, shape
import numpy as np
import math

from numpy import angle
import math_tools
from scipy import optimize as opt, size
import generateModelData_CSAD as data
import matplotlib.pyplot as plt

from std_msgs.msg import Float64MultiArray

import rospy

import yaml
import os

path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/simulator/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)
    
global timeStep
timeStep = 1.0/params["runfrequency"]



class Wave():
    def __init__(self, Hs, Tp, waterDepth=np.infty, angle=0, deltaFrequency=0.1, regular=True, dt=timeStep):
        
        self.dt = dt
        
        self.Hs = Hs
        self.Tp = Tp
        self.amplitude = Hs/2.0
        self.angleWaveNED = angle
        self.angleWaveBody= self.angleWaveNED
        self.frequency = 2.0*math.pi/Tp
        self.waterDepth = waterDepth
        #Parameters for irregular states
        self.alpha = 0.0081
        self.gamma = 3.3
        self.sigma_a = 0.07
        self.sigma_b = 0.09
        self.dw = deltaFrequency
        # self.dT = 0.2 #unused
        
        self.w1 = 2.0*pi*1.3/self.Tp
        self.a = 0.11*(self.Hs**2)*self.w1
        self.b = 0.44*(self.w1**4)
        self.m0 = self.a/(4.0*self.b)
        self.Hm0 = 4.0*math_tools.sqrt(self.m0)
        self.spectrumFrequencies = np.arange(3.6*math_tools.sqrt(self.Hm0), 5.0*math_tools.sqrt(self.Hm0), self.dw)
        # self.spectrumFrequencies = np.arange(data.frequencies[0], self.frequency, self.dw) #Set upper and lower values!
        
        #Initial guess for wave number:
        self.k = self.getWaveNumber(initial=True)
    
        self.regular = regular #True = regular, False = irregular
        
        self.time = 0
        
        [self.elementAmplitude, self.elementFrequencies, self.elementPhase] = self.generateIrregular()
        
        self.msg_waveMeasurement = Float64MultiArray()
        self.pub_waveMeasurement = rospy.Publisher('/waveElevation', Float64MultiArray, queue_size=1)
        
    def publish(self):
        self.msg_waveMeasurement[0] = self.getWaveElevation()
        self.msg_waveMeasurement[1] = self.frequency
        
        self.pub_waveMeasurement.publish(self.msg_waveMeasurement)
        
    def updateHeading(self, heading):
        """
        This function must be called every time the vessel state are updated!

        Args:
            heading (float): vessel's heading
        """
        self.angleWaveBody = self.angleWaveNED - heading
        
        
        
    
       
    def getFirstOrderLoad(self, waveFreq, angleWaveBody, waveAmplitude, wavePhase=0.0):
        headingIndex = np.argmin(np.abs(data.headingsData - angleWaveBody))
        if self.regular == False:
            count = 0
            firstOrderLoad = np.zeros(6)
            for w in waveFreq:
                
                frequencyIndex = np.argmin(np.abs(data.frequencies - w))
                
                forceAmp = np.array([data.RAO_FORCE['amp'][0][frequencyIndex, headingIndex,0],
                                    data.RAO_FORCE['amp'][1][frequencyIndex, headingIndex,0],
                                    data.RAO_FORCE['amp'][2][frequencyIndex, headingIndex,0],
                                    data.RAO_FORCE['amp'][3][frequencyIndex, headingIndex,0],
                                    data.RAO_FORCE['amp'][4][frequencyIndex, headingIndex,0],
                                    data.RAO_FORCE['amp'][5][frequencyIndex, headingIndex,0]])
                forcePhase = np.array([data.RAO_FORCE['phase'][0][frequencyIndex, headingIndex,0],
                                    data.RAO_FORCE['phase'][1][frequencyIndex, headingIndex,0],
                                    data.RAO_FORCE['phase'][2][frequencyIndex, headingIndex,0],
                                    data.RAO_FORCE['phase'][3][frequencyIndex, headingIndex,0],
                                    data.RAO_FORCE['phase'][4][frequencyIndex, headingIndex,0],
                                    data.RAO_FORCE['phase'][5][frequencyIndex, headingIndex,0]])
                firstOrderLoad = np.add(firstOrderLoad, np.abs(forceAmp)*waveAmplitude[count]*np.cos(w*self.time + forcePhase + wavePhase[count]))
                
                count += 1
                
        else:
            frequencyIndex = np.argmin(np.abs(data.frequencies - waveFreq))
            forceAmp = np.array([data.RAO_FORCE['amp'][0][frequencyIndex, headingIndex,0],
                                data.RAO_FORCE['amp'][1][frequencyIndex, headingIndex,0],
                                data.RAO_FORCE['amp'][2][frequencyIndex, headingIndex,0],
                                data.RAO_FORCE['amp'][3][frequencyIndex, headingIndex,0],
                                data.RAO_FORCE['amp'][4][frequencyIndex, headingIndex,0],
                                data.RAO_FORCE['amp'][5][frequencyIndex, headingIndex,0]])
            forcePhase = np.array([data.RAO_FORCE['phase'][0][frequencyIndex, headingIndex,0],
                                data.RAO_FORCE['phase'][1][frequencyIndex, headingIndex,0],
                                data.RAO_FORCE['phase'][2][frequencyIndex, headingIndex,0],
                                data.RAO_FORCE['phase'][3][frequencyIndex, headingIndex,0],
                                data.RAO_FORCE['phase'][4][frequencyIndex, headingIndex,0],
                                data.RAO_FORCE['phase'][5][frequencyIndex, headingIndex,0]])
            
            firstOrderLoad = np.abs(forceAmp)*waveAmplitude*np.cos(waveFreq*self.time + forcePhase + wavePhase)
        return firstOrderLoad
    
    def getDriftLoads(self, waveFreq, angleWaveBody, waveAmplitude):
        """
        This function takes wavefrequency and angle as input and find corresponding wave drift coefficient [N/A^2] in a lookup table.
        Returns wave drift load in 3DOF (output is 6DOF). These are based on Bjoernoe's data from CSAD.
        """
        headingIndex = np.argmin(np.abs(data.headingsData - angleWaveBody))
        if self.regular == False:
            count = 0
            driftLoads = np.zeros(3)
            for w in waveFreq:
                frequencyIndex = np.argmin(np.abs(data.driftForceFreq - w))
            
                driftCoefficient = np.array([data.driftForceAmpX[frequencyIndex, headingIndex],
                                    data.driftForceAmpY[frequencyIndex, headingIndex],
                                    data.driftForceAmpPsi[frequencyIndex, headingIndex]])
                driftLoads = np.add(driftLoads, np.abs(driftCoefficient)*np.power(waveAmplitude[count],2))
                count += 1
        else:
            frequencyIndex = np.argmin(np.abs(data.driftForceFreq - waveFreq))
            driftCoefficient = np.array([data.driftForceAmpX[frequencyIndex, headingIndex],
                                data.driftForceAmpY[frequencyIndex, headingIndex],
                                data.driftForceAmpPsi[frequencyIndex, headingIndex]])
            driftLoads = np.abs(driftCoefficient)*(waveAmplitude**2)
        
        driftLoads = math_tools.three2sixDof(driftLoads)
        return driftLoads
    
    def getSlowlyVaryingLoads(self, waveFreq, angleWaveBody, waveAmplitude, phaseLag=0.0):
        """
        Provides slowly varying loads based on Newman's approximation.
        
        Args:
            waveFreq (float | array): if seastate is irregular, this is an array of the wave element's frequencies
            angleWaveBody (float): _description_
            waveAmplitude (float | array): if seastate is irregular, this is an array of the wave element's amplitudes
            phaseLag (array, optional): if seastate is irregular, this is an array of the wave element's phaselags. Defaults to 0.

        Returns:
            array: 6DOF array of slowly varying loads
        """
        headingIndex = np.argmin(np.abs(data.headingsData - angleWaveBody))
        if self.regular == False:
            xElement = 0.0
            yElement = 0.0
            psiElement = 0.0
            count = 0
            for w in waveFreq:
                frequencyIndex = np.argmin(np.abs(data.driftForceFreq - w))
                
                driftCoefficient = np.array([data.driftForceAmpX[frequencyIndex, headingIndex],
                                    data.driftForceAmpY[frequencyIndex, headingIndex],
                                    data.driftForceAmpPsi[frequencyIndex, headingIndex]])
                xElement = np.add(xElement, waveAmplitude[count]*np.power(np.abs(driftCoefficient[0]), 0.5)*np.cos(w*self.time + phaseLag[count]))
                yElement = np.add(yElement, waveAmplitude[count]*np.power(np.abs(driftCoefficient[1]), 0.5)*np.cos(w*self.time + phaseLag[count])) #What happens when driftCoefficients are less than zero?????
                psiElement = np.add(psiElement, waveAmplitude[count]*np.power(np.abs(driftCoefficient[2]), 0.5)*np.cos(w*self.time + phaseLag[count]))
                count += 1    
            
            slowlyVaryingLoads = np.array([2.0*np.power(xElement, 2.0), 2.0*np.power(yElement, 2.0), 2.0*np.power(psiElement, 2.0)])
                
            
        else:
                
            frequencyIndex = np.argmin(np.abs(data.driftForceFreq - waveFreq))
            
            driftCoefficient = np.array([data.driftForceAmpX[frequencyIndex, headingIndex],
                                data.driftForceAmpY[frequencyIndex, headingIndex],
                                data.driftForceAmpPsi[frequencyIndex, headingIndex]])
            xElement = np.sum(waveAmplitude*np.power(np.abs(driftCoefficient[0]), 0.5)*np.cos(waveFreq*self.time + phaseLag))
            yElement = np.sum(waveAmplitude*np.power(np.abs(driftCoefficient[1]), 0.5)*np.cos(waveFreq*self.time + phaseLag)) #What happens when driftCoefficients are less than zero?????
            psiElement = np.sum(waveAmplitude*np.power(np.abs(driftCoefficient[2]), 0.5)*np.cos(waveFreq*self.time + phaseLag))
            
            slowlyVaryingLoads = np.array([2*np.power(xElement, 2), 2*np.power(yElement, 2), 2*np.power(psiElement, 2)])
        
        slowlyVaryingLoads = math_tools.three2sixDof(slowlyVaryingLoads)
        return slowlyVaryingLoads
        
    def getWaveLoads(self):
        """Outputs sum of wave loads in body frame

        Returns:
            array: sum of driftLoads, froudeKryolv and slowly varying wave loads
        """
        
        #This function must rely on wheter there is a regular or irregular state!
        #Regular waves:

        if (self.regular == False):
            # [elementAmplitude, elementFrequencies, elementPhase] = self.generateIrregular()
            firstOrderLoads = self.getFirstOrderLoad(self.elementFrequencies, self.angleWaveBody, self.elementAmplitude, self.elementPhase)    #Does it reflect first order loads?
            driftLoads = self.getDriftLoads(self.elementFrequencies, self.angleWaveBody, self.elementAmplitude)   #Gives too high values...
            slowlyVaryingLoads = self.getSlowlyVaryingLoads(self.elementFrequencies, self.angleWaveBody, self.elementAmplitude, phaseLag=self.elementPhase) #To high values and not slowly varying...
            
        else:   #If the regular==True:
            firstOrderLoads = self.getFirstOrderLoad(self.frequency, self.angleWaveBody, self.amplitude)
            driftLoads = self.getDriftLoads(self.frequency, self.angleWaveBody, self.amplitude)
            slowlyVaryingLoads = self.getSlowlyVaryingLoads(self.frequency, self.angleWaveBody, self.amplitude)
        
        self.time += self.dt
        
        sumLoads = np.add(driftLoads, slowlyVaryingLoads) 
        sumLoads = np.add(sumLoads, firstOrderLoads)
        
        return sumLoads
    
    
        
    def getWaveNumber(self, initial=False):
        """
        Estimates the wave number based on the dispertion relation.
        """
        if (initial==False):
            #Checks if deepwater assumptions are valid:
            if ((self.waterDepth*(self.frequency**2))/(data.g*2*pi) >= 0.5):
                deepWater = False
            else:
                deepWater = True
        
        else: #If initial == True
            deepWater = True 
       
        
        if (deepWater==False):
            f = data.g*self.k*np.math.tanh(self.k*self.waterDepth) - self.frequency**2
            while (np.abs(f) >= 0.1):
                self.k += 0.0001
                f = data.g*self.k*np.math.tanh(self.k*self.waterDepth) - self.frequency**2
                
            return self.k
        else:
            return (self.frequency**2)/data.g
    
    
    # def generateWave(self, x=0):
        
    #     if (self.regular==True): #Regular
    #         amplitude = self.Hs/2
        
            
    #         k = self.getWaveNumber()

    #         waveElevation = amplitude*math.cos(self.frequency*self.time - k*x)

    #         self.time += self.dt
            
    #     else: #Irregular
    #         waveElevation = 0
            
    #     return waveElevation
        
        
    
    def defineSpectrum(self, frequency):
        """JONSWAP"""
        spectrum = self.alpha*((data.g**2)/frequency**5)*np.exp(-(5/4)*(self.frequency/frequency)**4)
        gamma = self.gamma*np.ones(np.shape(spectrum))
        if np.size(frequency) > 1:
            # sigma = []
            for w in frequency:
                if w <= self.frequency:
                    # sigma.append(0.07)
                    sigma = 0.07
                else: 
                    # sigma.append(0.09)
                    sigma = 0.09
        else:
            if frequency <= self.frequency:
                sigma = 0.07
            else: 
                sigma = 0.09
        
        factor = gamma**(np.exp(-0.5*(np.divide((frequency-self.frequency),(np.multiply(sigma,self.frequency))))**2))
        spectrum *= factor
        
        
        """Gaussian swell spectrum"""
        # sigma = 0.1
        # spectrum = ((self.Hs/4)**2)/(2*np.pi*sigma*math_tools.sqrt(2*pi))*np.exp(-((frequency-self.frequency)**2)/(2*sigma**2))
        
        return spectrum
    

    def generateIrregular(self):
        # k = self.getWaveNumber()
        elementAmplitude = []
        elementFrequencies = []
        elementPhase = []
        for w in self.spectrumFrequencies:
            
            spectrum = self.defineSpectrum(w)
            elementAmplitude.append(math_tools.sqrt(2.0*spectrum*self.dw))  #contains all wave amplitudes
            elementFrequencies.append(w)
            elementPhase.append(2*pi*np.random.rand())
            
        return elementAmplitude, elementFrequencies, elementPhase
        
        
    def getWaveElevation(self, stopTime=30):
        """Returns time array and a wave elevation array. Pr 14.04.2022, not made for useage in control system."""
        time = np.arange(0.0, stopTime, self.dt)
        elevation = []
        
        if (self.regular == True):
            for t in time:
                elevation.append(self.amplitude*np.cos(self.frequency*t)) #should include ther term -k*x based on where the elevation are measured.
        else:
            for t in time:
                e = 0
                counter = 0
                for w in self.elementFrequencies:
                    e += self.elementAmplitude[counter]*np.cos(self.elementFrequencies[counter]*t + self.elementPhase[counter])
                    counter += 1
                elevation.append(e)
                    
        return time, elevation
        

seastate = Wave(3.1, 5.4, regular=False, dt=0.001)
[time, elev] = seastate.getWaveElevation(20)
f = np.fft.fft(elev, len(elev))
freq = np.fft.fftfreq(len(elev), seastate.dt)
# print(freq)
# inx = fhat > 40
# fhat *= inx
# restore = np.fft.ifft(fhat)
# fhat = np.fft.fft(restore)
# print(seastate.elementFrequencies)

fig,axs = plt.subplots(2,1)
plt.sca(axs[0])
plt.plot(time, elev)

plt.sca(axs[1])
plt.plot(2*pi/time, f)

plt.show()
# t = 0
# time = []
# a = []
# fig = plt.figure()
# while(t < 20):
    
#     [amp, freq, phases] = seastate.generateIrregular()
#     print(amp)
#     a.append(amp)
#     time.append(t)
#     # plt.scatter(t, a)
    
    
#     t += 1
# print(shape(time), shape(a))
# plt.show()
# teststate = Wave(0.1, 2.4, regular=False)
# freq = teststate.spectrumFrequencies
# spec = teststate.defineSpectrum(freq)
# # fig = plt.figure()
# # plt.plot(freq, spec)
# t = 0
# load = []
# time = []
# while t < 10:
#     # print(seastate.getWaveLoads())
#     # seastate.getWaveLoads()
#     seastate.getWaveLoads()
#     time.append(t)
#     t+=0.01
    
# # fig = plt.figure()
# # plt.plot(time, load)
# # plt.show()

    
    
