#!/usr/bin/env python

from cmath import phase, pi
from random import setstate
from turtle import heading, shape
import wave
import numpy as np
import math

from numpy import angle
import math_tools
from scipy import optimize as opt, size
import generateModelData_CSAD as data
import matplotlib.pyplot as plt

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from messages.msg import wave_message

import rospy

import yaml
import os

path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/simulator/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)
    
global timeStep
timeStep = 1.0/params["runfrequency"]

with open(r"{0}/csad_dp_ws/src/simulator/include/seastateDefinitions.yaml".format(path)) as file:
    states = yaml.load(file, Loader=yaml.Loader)

class Wave():
    def __init__(self, Hs, Tp, stateDescription='calm', waterDepth=np.infty, angle=0.0, deltaFrequency=0.1, regular=True, dt=timeStep):
        """
        StateDescription defines realistic sea states where the valid area for Hs and Tp is defined.
        Can choose between 'calm', 'smooth', 'slight', 'moderate', 'rough', 'very_rough', 'high', 'very_high' or 'phenomenal'.
        If nothing is stated, then Hs and Tp will be overwritten to closest value within default 'calm'.
        If the Hs or Tp is outside the valid area, they will be overwritten to closest value within valid area.
        """
        Hs_min = states['seastates'][stateDescription]['Hs_min']
        Hs_max = states['seastates'][stateDescription]['Hs_max']
        Tp_min = states['seastates'][stateDescription]['Tp_min']
        Tp_max = states['seastates'][stateDescription]['Tp_max']
        
        if stateDescription == 'calm':
            self.Hs = Hs_max
            self.Tp = Tp_max
        if Hs_min > Hs: self.Hs = Hs_min
        elif Hs_max < Hs: self.Hs = Hs_max    
        else: self.Hs = Hs
        if Tp_min > Tp: self.Tp = Tp_min
        elif Tp_max < Tp: self.Tp = Tp_max
        else: self.Tp = Tp
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
        self.a = 0.11*(self.Hs**2.0)*self.w1
        self.b = 0.44*(self.w1**4.0)
        self.m0 = self.a/(4.0*self.b)
        self.Hm0 = 4.0*math_tools.sqrt(self.m0)
        # self.spectrumFrequencies = np.arange(3.6*math_tools.sqrt(self.Hm0), 5.0*math_tools.sqrt(self.Hm0), self.dw)
        self.spectrumFrequencies = np.arange(2.0*np.pi/Tp_max, 2.0*np.pi/Tp_min, self.dw)
        # self.spectrumFrequencies = np.arange(data.frequencies[0], self.frequency, self.dw) #Set upper and lower values!
        
        #Initial guess for wave number:
        self.k = self.getWaveNumber(initial=True)
        self.measurementDistance = 0.0
        self.regular = regular #True = regular, False = irregular
        
        self.time = 0.0
        
        [self.elementAmplitude, self.elementFrequencies, self.elementPhase] = self.generateIrregular()
        [self.xMinus, self.xPluss, self.yMinus, self.yPluss, self.psiMinus, self.psiPluss] = self.separatePlusMinusCoefficients(self.elementFrequencies)
        
        
        self.pub_waveMeasurement = rospy.Publisher('/waveElevation', wave_message, queue_size=1)
        
    def separatePlusMinusCoefficients(self, frequencies):
        xMinus = np.array([])
        xPluss = np.array([])
        yMinus = np.array([])
        yPluss = np.array([])
        psiMinus = np.array([])
        psiPluss = np.array([])
        headingIndex = np.argmin(np.abs(data.headingsData - self.angleWaveBody))
        countX = 0
        countY = 0
        countPsi = 0
        for w in frequencies:
            frequencyIndex = np.argmin(np.abs(data.frequencies - w))
            
            driftCoefficient = np.array([data.driftForceAmpX[frequencyIndex, headingIndex],
                                        data.driftForceAmpY[frequencyIndex, headingIndex],
                                        data.driftForceAmpPsi[frequencyIndex, headingIndex]])
            if driftCoefficient[0] < 0.0:
                xMinus = np.resize(xMinus, (len(xMinus)+1, 2))
                xMinus[-1,0] = countX
                xMinus[-1,1] = driftCoefficient[0]
                countX += 1
            else:
                xPluss = np.resize(xMinus, (len(xMinus)+1, 2))
                xPluss[-1,0] = countX
                xPluss[-1,1] = driftCoefficient[0]
                countX += 1
            if driftCoefficient[1] < 0.0:
                yMinus = np.resize(yMinus, (len(yMinus)+1, 2))
                yMinus[-1,0] = countY
                yMinus[-1,1] = driftCoefficient[1]
                countY += 1
            else:
                yPluss = np.resize(yPluss, (len(yPluss)+1, 2))
                yPluss[-1,0] = countY
                yPluss[-1,1] = driftCoefficient[1]
                countY += 1
            if driftCoefficient[2] < 0.0:
                psiMinus = np.resize(psiMinus, (len(psiMinus)+1, 2))
                psiMinus[-1,0] = countPsi
                psiMinus[-1,1] = driftCoefficient[2]
                countPsi += 1
            else:
                psiPluss = np.resize(psiPluss, (len(psiPluss)+1, 2))
                psiPluss[-1,0] = countPsi
                psiPluss[-1,1] = driftCoefficient[2]
                countPsi += 1
        return xMinus, xPluss, yMinus, yPluss, psiMinus, psiPluss
            
                
            
        
    def publish(self):
        msg_waveMeasurement = wave_message()
        msg_waveMeasurement.elevation = self.getWaveElevation(self.time)
        msg_waveMeasurement.distance = self.measurementDistance
        msg_waveMeasurement.Tp = self.Tp
        msg_waveMeasurement.Hs = self.Hs
        
        self.pub_waveMeasurement.publish(msg_waveMeasurement)
        
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
        firstOrderLoad = np.array(firstOrderLoad)
        firstOrderLoad = np.resize(firstOrderLoad, (6,1))
        
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
                driftLoads = np.add(driftLoads, np.abs(driftCoefficient)*np.power(waveAmplitude[count],2.0))
                count += 1
        else:
            frequencyIndex = np.argmin(np.abs(data.driftForceFreq - waveFreq))
            driftCoefficient = np.array([data.driftForceAmpX[frequencyIndex, headingIndex],
                                         data.driftForceAmpY[frequencyIndex, headingIndex],
                                         data.driftForceAmpPsi[frequencyIndex, headingIndex]])
            driftLoads = np.abs(driftCoefficient)*(waveAmplitude**2.0)
        
        driftLoads = math_tools.three2sixDof(driftLoads)
        return driftLoads
    
    def getSlowlyVaryingLoads(self, waveFreq, angleWaveBody, waveAmplitude, phaseLag=np.array([0.0])):
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
        # if self.regular == False:
        #     xElement = 0.0
        #     yElement = 0.0
        #     psiElement = 0.0
        #     
        #     if self.xPluss.any():
        #         i = self.xPluss[:,0].astype(int)
        #         xElement = np.power(np.sum(waveAmplitude[i]*np.sqrt(2.0*self.xPluss[:,1])*np.cos(waveFreq[i]*self.time + phaseLag[i])), 2.0)
        #     if self.xMinus.any():
        #         i = self.xMinus[:,0].astype(int)
        #         xElement -= np.power(np.sum(waveAmplitude[i]*np.sqrt(-2.0*self.xMinus[:,1])*np.cos(waveFreq[i]*self.time + phaseLag[i])), 2.0)
        #     if self.yPluss.any():
        #         i = self.yPluss[:,0].astype(int)
        #         yElement = np.power(np.sum(waveAmplitude[i]*np.sqrt(2.0*self.yPluss[:,1])*np.cos(waveFreq[i]*self.time + phaseLag[i])), 2.0)
        #     if self.yMinus.any():
        #         i = self.yMinus[:,0].astype(int)
        #         yElement -= np.power(np.sum(waveAmplitude[i]*np.sqrt(-2.0*self.yMinus[:,1])*np.cos(waveFreq[i]*self.time + phaseLag[i])), 2.0)
        #     if self.psiPluss.any():
        #         i = self.psiPluss[:,0].astype(int)
        #         psiElement = np.power(np.sum(waveAmplitude[i]*np.sqrt(2.0*self.psiPluss[:,1])*np.cos(waveFreq[i]*self.time + phaseLag[i])), 2.0)
        #     if self.psiMinus.any():
        #         i = self.psiMinus[:,0].astype(int)
        #         psiElement -= np.power(np.sum(waveAmplitude[i]*np.sqrt(-2.0*self.psiMinus[:,1])*np.cos(waveFreq[i]*self.time + phaseLag[i])), 2.0)
        #     slowlyVaryingLoads = np.array([xElement, yElement, psiElement])
            
            
        xElement = 0.0
        yElement = 0.0
        psiElement = 0.0
        count = 0    
        if self.regular == False:
            for w in waveFreq:
                frequencyIndex = np.argmin(np.abs(data.driftForceFreq - w))
                
                driftCoefficient = np.array([data.driftForceAmpX[frequencyIndex, headingIndex],
                                                data.driftForceAmpY[frequencyIndex, headingIndex],
                                                data.driftForceAmpPsi[frequencyIndex, headingIndex]])
                xElement += waveAmplitude[count]*np.power(np.abs(driftCoefficient[0]), 0.5)*np.cos(w*self.time + phaseLag[count])
                yElement += waveAmplitude[count]*np.power(np.abs(driftCoefficient[1]), 0.5)*np.cos(w*self.time + phaseLag[count])
                psiElement += waveAmplitude[count]*np.power(np.abs(driftCoefficient[2]), 0.5)*np.cos(w*self.time + phaseLag[count])
                count += 1    
            
            slowlyVaryingLoads = np.array([2.0*np.power(xElement, 2.0), 2.0*np.power(yElement, 2.0), 2.0*np.power(psiElement, 2.0)])
                
            
        else:
            frequencyIndex = np.argmin(np.abs(data.driftForceFreq - waveFreq))
            
            driftCoefficient = np.array([data.driftForceAmpX[frequencyIndex, headingIndex],
                                         data.driftForceAmpY[frequencyIndex, headingIndex],
                                         data.driftForceAmpPsi[frequencyIndex, headingIndex]])
            
            
            xElement = waveAmplitude*(np.power(np.abs(driftCoefficient[0]), 0.5))*np.cos(waveFreq*self.time + phaseLag[0])
            yElement = waveAmplitude*(np.power(np.abs(driftCoefficient[1]), 0.5))*np.cos(waveFreq*self.time + phaseLag[0])
            psiElement = waveAmplitude*(np.power(np.abs(driftCoefficient[2]), 0.5))*np.cos(waveFreq*self.time + phaseLag[0])
            slowlyVaryingLoads = np.array([2.0*np.power(xElement, 2.0), 2.0*np.power(yElement, 2.0), 2.0*np.power(psiElement, 2.0)])
            
        slowlyVaryingLoads = math_tools.three2sixDof(slowlyVaryingLoads)
        slowlyVaryingLoads = np.resize(slowlyVaryingLoads, (6,1))
        
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
        
        # sumLoads = np.add(driftLoads, slowlyVaryingLoads) 
        # sumLoads = np.add(sumLoads, firstOrderLoads)
        
        return firstOrderLoads + driftLoads #driftLoads #firstOrderLoads #slowlyVaryingLoads #sumLoads
    
    
        
    def getWaveNumber(self, initial=False):
        """
        Estimates the wave number based on the dispertion relation. 
        Make sure that the wave depth is set correctly before calling.
        """
        if (initial==False):
            #Checks if deepwater assumptions are valid:
            if ((self.waterDepth*(self.frequency**2.0))/(data.g*2.0*np.pi) >= 0.5):
                deepWater = False
            else:
                deepWater = True
        
        else: #If initial == True
            deepWater = True 
       
        
        if (deepWater==False):
            f = data.g*self.k*np.math.tanh(self.k*self.waterDepth) - self.frequency**2.0
            while (np.abs(f) >= 0.1):
                self.k += 0.0001
                f = data.g*self.k*np.math.tanh(self.k*self.waterDepth) - self.frequency**2.0
                
            return self.k
        else:
            return (self.frequency**2.0)/data.g
    
    
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
        
        spectrum = self.alpha*((data.g**2.0)/frequency**5.0)*np.exp(-(5.0/4.0)*(self.frequency/frequency)**4.0)
        gamma = self.gamma*np.ones(np.shape(spectrum))
        
        if frequency <= self.frequency:
            sigma = 0.07
            
        else: 
            sigma = 0.09
        
        factor = gamma**(np.exp(-0.5*(np.divide((frequency-self.frequency),(np.multiply(sigma,self.frequency))))**2.0))
        spectrum *= factor
        
        """Gaussian swell spectrum"""
        # sigma = 0.1
        # spectrum = ((self.Hs/4)**2)/(2*np.pi*sigma*math_tools.sqrt(2*pi))*np.exp(-((frequency-self.frequency)**2)/(2*sigma**2))
        
        return spectrum
    

    def generateIrregular(self):
        """This functions should only be called in the constructor when the sea state is defined."""
        
        elementFrequencies = self.spectrumFrequencies
        elementAmplitude = np.zeros(len(elementFrequencies))
        elementPhase = np.zeros(len(elementFrequencies))
        spectrums = np.zeros(len(elementFrequencies))
        count = 0
        for w in self.spectrumFrequencies:
            spectrum = self.defineSpectrum(w)
            spectrums[count] = spectrum
            elementAmplitude[count] = math_tools.sqrt(2.0*spectrum*self.dw) #contains all wave amplitudes
            elementPhase[count] = 2.0*pi*np.random.rand()
            count +=1
        
        # elementAmplitude = np.array(elementAmplitude).astype(float)
        # elementFrequencies = np.array(elementFrequencies).astype(float)
        # elementPhase = np.array(elementPhase).astype(float)
        # elementFrequencies = elementFrequencies.astype(float)
        fig = plt.figure()
        plt.plot(elementFrequencies/(2.0*np.pi), spectrums)
            
        return elementAmplitude, elementFrequencies, elementPhase
        
    def setMeasurementDistance(self, x=0.0):
        self.measurementDistance = x
    
    def getWaveElevation(self, time):
        """Returns time array and a wave elevation array. Pr 14.04.2022, not made for useage in control system."""
        x = self.measurementDistance
        
        if (self.regular == True):
            k = np.power(self.frequency, 2.0)/data.g
            elevation = self.amplitude*np.cos(self.frequency*time - k*x)
        else:
            amplitudes = self.elementAmplitude
            frequencies = self.elementFrequencies
            phases = 2.0*np.pi*np.random.normal(len(self.elementPhase))
            frequencies += np.random.normal(loc=0.0,scale=self.dw/2.0, size=len(frequencies))
            k = np.power(frequencies, 2.0)/data.g
            elevation = np.sum(amplitudes*np.cos(frequencies*time - k*x + phases))
        return elevation
        



# seastate = Wave(0.06, 1.15, stateDescription='very_rough', regular=False)
# seastate.updateHeading(0.0)
# t = 0.0
# time = []
# tauX = []
# tauY = []
# tauZ = []
# elev = []
# while t < 50:
#     loads = seastate.getWaveLoads()
#     seastate.getWaveLoads()
#     time.append(t)
#     tauX.append(loads[0])
#     tauY.append(loads[1])
#     tauZ.append(loads[5])
#     elev.append(seastate.getWaveElevation(t))
#     t += seastate.dt
    
# m = np.max(tauX)
# ind = np.where(tauX==m)

# Tstr = 'Mean drift: Tp = ' + str(2*pi/seastate.frequency) + ', Hs = ' + str(seastate.Hs)

# fig,axs = plt.subplots(4,1)
# plt.sca(axs[0])
# plt.title(Tstr)
# plt.plot(time, tauX)
# for i in ind[0]:
#     plt.scatter(time[i], tauX[i], color="red")
# plt.grid()
# plt.legend(['force x'])
# plt.sca(axs[1])
# plt.plot(time, tauY)
# plt.grid()
# plt.legend(['force y'])
# plt.sca(axs[2])
# plt.plot(time, tauZ)
# plt.grid()
# plt.legend(['force z'])
# # # plt.sca(axs[1])
# # # plt.plot(2*pi/time, f)

# plt.sca(axs[3])
# plt.plot(time, elev)
# plt.grid()
# plt.legend(['wave elevation'])


# plt.show()



    
    
