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
with open(r"{0}/csad_dp_ws/src/simulator/include/seastateDefinitions.yaml".format(path)) as file:
    states = yaml.load(file, Loader=yaml.Loader)

class Wave():
    def __init__(self, Hs, Tp, stateDescription='calm', waterDepth=np.infty, angle=0.0, N = 15, regular=True, dt=0.02):
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
        
        self.Hs = Hs
        self.Tp = Tp
        # if stateDescription == 'calm':
        #     self.Hs = Hs_max
        #     self.Tp = Tp_max
        # if Hs_min > Hs:
        #     self.Hs = Hs_min
        # elif Hs_max < Hs:
        #     self.Hs = Hs_max   
        # else: 
        #     self.Hs = Hs
        # if Tp_min > Tp:
        #     self.Tp = Tp_min
        # elif Tp_max < Tp:
        #     self.Tp = Tp_max
        # else:
        #     self.Tp = Tp
        
        self.dt = dt
        
        self.amplitude = self.Hs/2.0
        self.angleWaveNED = angle
        self.angleWaveBody= self.angleWaveNED
        self.frequency = 2.0*np.pi/self.Tp
        self.waterDepth = waterDepth
        #Parameters for irregular states
        self.alpha = 0.0081
        self.gamma = 3.3
        self.sigma_a = 0.07
        self.sigma_b = 0.09
        # self.dT = 0.2 #unused
        
        self.w1 = 2.0*pi*1.3/self.Tp
        self.a = 0.11*(self.Hs**2.0)*self.w1
        self.b = 0.44*(self.w1**4.0)
        self.m0 = self.a/(4.0*self.b)
        self.Hm0 = 4.0*math_tools.sqrt(self.m0)
        
        self.N = N
        spectrum_min = 0.5*2.0*np.pi/self.Tp
        spectrum_max = 2.0*2.0*np.pi/self.Tp
        self.dw = (spectrum_max - spectrum_min)/self.N
        
        self.spectrumFrequencies = np.arange(spectrum_min, spectrum_max, self.dw)
        
        
        #Initial guess for wave number:
        self.k = self.getWaveNumber(initial=True)
        self.measurementDistance = 0.0
        self.regular = regular #True = regular, False = irregular
        
        self.time = 0.0
        
        [self.elementAmplitude, self.elementFrequencies, self.elementPhase] = self.generateIrregular()
        # [self.xMinus, self.xPluss, self.yMinus, self.yPluss, self.psiMinus, self.psiPluss] = self.separatePlusMinusCoefficients(self.elementFrequencies)
        self.omega_c = self.frequency*1.1
        self.slowlyVaryingLoads_previous = np.zeros([6,1])
        self.slowlyVaryingLoads_previous_filtered = np.zeros([6,1])
        
        self.pub_waveLoads = rospy.Publisher('/waveLoads', Float64MultiArray, queue_size=1) #Not used inside control system, only for plotting.
        self.pub_waveMeasurement = rospy.Publisher('/waveElevation', wave_message, queue_size=1)
            
        
    def publish(self, tauWave=np.zeros([6,1])):
        msg_waveMeasurement = wave_message()
        msg_waveMeasurement.elevation = self.getWaveElevation(self.time)
        msg_waveMeasurement.distance = self.measurementDistance
        msg_waveMeasurement.Tp = self.Tp
        msg_waveMeasurement.Hs = self.Hs
        self.pub_waveMeasurement.publish(msg_waveMeasurement)
        
        msg_waveLoads = Float64MultiArray()
        tauWithTime = np.insert(tauWave, 0, self.time)
        msg_waveLoads.data = tauWithTime
        
        self.pub_waveLoads.publish(msg_waveLoads)
    
        
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
    
    def getSlowlyVaryingLoads(self, waveFreq, angleWaveBody, waveAmplitude, wavePhase=np.array([0.0])):
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
            
        xElement = 0.0
        yElement = 0.0
        psiElement = 0.0
        realValue = np.zeros([3,1])
        if self.regular == False:
            for k in range(int(0.2*self.N), int(0.7*self.N), 1):
                frequencyIndex_k = np.argmin(np.abs(data.driftForceFreq - waveFreq[k]))
                Q_k = np.array([data.driftForceAmpX[frequencyIndex_k, headingIndex], 
                                    data.driftForceAmpY[frequencyIndex_k, headingIndex],
                                    data.driftForceAmpPsi[frequencyIndex_k, headingIndex]])
                for i in range(k):
                    frequencyIndex_i = np.argmin(np.abs(data.driftForceFreq - waveFreq[i]))
                    Q_i = np.array([data.driftForceAmpX[frequencyIndex_i, headingIndex], 
                                    data.driftForceAmpY[frequencyIndex_i, headingIndex],
                                    data.driftForceAmpPsi[frequencyIndex_i, headingIndex]])
                    Q_ki = 0.5*(np.abs(Q_k) + np.abs(Q_i))
                    element = Q_ki*waveAmplitude[k]*waveAmplitude[i]*np.exp(1j*(waveFreq[k]-waveFreq[i])*self.time - (wavePhase[k]-wavePhase[i]))
                    element = np.resize(element, (3,1))
                    
                    realValue += np.real(element)
                    
            slowlyVaryingLoads = 2.0*realValue
           
            
        else: #not in use?
            frequencyIndex = np.argmin(np.abs(data.driftForceFreq - waveFreq))
            
            driftCoefficient = np.array([data.driftForceAmpX[frequencyIndex, headingIndex],
                                         data.driftForceAmpY[frequencyIndex, headingIndex],
                                         data.driftForceAmpPsi[frequencyIndex, headingIndex]])
            
            
            xElement = waveAmplitude*(np.power(np.abs(driftCoefficient[0]), 0.5))*np.cos(waveFreq*self.time + wavePhase[0])
            yElement = waveAmplitude*(np.power(np.abs(driftCoefficient[1]), 0.5))*np.cos(waveFreq*self.time + wavePhase[0])
            psiElement = waveAmplitude*(np.power(np.abs(driftCoefficient[2]), 0.5))*np.cos(waveFreq*self.time + wavePhase[0])
            slowlyVaryingLoads = np.array([2.0*np.power(xElement, 2.0), 2.0*np.power(yElement, 2.0), 2.0*np.power(psiElement, 2.0)])
            
        slowlyVaryingLoads = math_tools.three2sixDof(slowlyVaryingLoads)
        slowlyVaryingLoads = np.resize(slowlyVaryingLoads, (6,1))
        slowlyVaryingLoads_filtered = math_tools.firstOrderLowPass(slowlyVaryingLoads, self.slowlyVaryingLoads_previous, self.slowlyVaryingLoads_previous_filtered, self.omega_c, self.dt)
        self.slowlyVaryingLoads_previous = slowlyVaryingLoads.copy()
        self.slowlyVaryingLoads_previous_filtered = slowlyVaryingLoads_filtered.copy()
        return slowlyVaryingLoads_filtered
        
    def getWaveLoads(self):
        """Outputs sum of wave loads in body frame

        Returns:   
            array: sum of driftLoads, froudeKryolv and slowly varying wave loads
        """
        #This function must rely on wheter there is a regular or irregular state!
        #Regular waves:
        Tf = 0.0
        if (self.regular == False):
            # [elementAmplitude, elementFrequencies, elementPhase] = self.generateIrregular()
            firstOrderLoads = self.getFirstOrderLoad(self.elementFrequencies, self.angleWaveBody, self.elementAmplitude, self.elementPhase)    #Does it reflect first order loads?
            driftLoads = self.getDriftLoads(self.elementFrequencies, self.angleWaveBody, self.elementAmplitude)   #Gives too high values...
            slowlyVaryingLoads = self.getSlowlyVaryingLoads(self.elementFrequencies, self.angleWaveBody, self.elementAmplitude, wavePhase=self.elementPhase) #To high values and not slowly varying...
            filteredSlow = np.power(slowlyVaryingLoads*Tf + 1.0, -1.0)
            
        else:   #If the regular==True:
            firstOrderLoads = self.getFirstOrderLoad(self.frequency, self.angleWaveBody, self.amplitude)
            driftLoads = self.getDriftLoads(self.frequency, self.angleWaveBody, self.amplitude)
            slowlyVaryingLoads = np.zeros([6,1])
        
        self.time += self.dt
        
        return - slowlyVaryingLoads - driftLoads - firstOrderLoads
    
    
        
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
            elementPhase[count] = 2.0*np.pi*np.random.rand()
            count += 1
        
        # fig = plt.figure()
        # plt.plot(elementFrequencies, spectrums)
        # plt.plot(elementFrequencies[int(0.2*self.N):int(0.7*self.N)], spectrums[int(0.2*self.N):int(0.7*self.N)])
        # plt.show()
        # # elementAmplitude = np.array(elementAmplitude).astype(float)
        # elementFrequencies = np.array(elementFrequencies).astype(float)
        # elementPhase = np.array(elementPhase).astype(float)
        # elementFrequencies = elementFrequencies.astype(float)
            
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
            # phases = 2.0*np.pi*np.random.normal(len(self.elementPhase))
            frequencies += np.random.normal(loc=0.0,scale=self.dw/2.0, size=len(frequencies))
            k = np.power(frequencies, 2.0)/data.g
            elevation = np.sum(amplitudes*np.cos(frequencies*time - k*x + self.elementPhase))
        return elevation
        
    
        


# #Uncomment for testing and debugging:
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



    
    
