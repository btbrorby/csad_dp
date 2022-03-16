#!/usr/bin/env python

from cmath import pi
from turtle import heading
import numpy as np
import math

from numpy import angle
import math_tools
from scipy import optimize as opt
import generateModelData_CSAD as data
import matplotlib.pyplot as plt




class Wave():
    def __init__(self, Hs, Tp, waterDepth=np.infty, angle=0, regular=True, dt=0.01):
        
        self.dt = dt
        
        self.Hs = Hs
        self.Tp = Tp
        self.amplitude = Hs/2
        self.angleWaveNED = angle
        self.angleWaveBody= self.angleWaveNED
        self.frequency = 2*math.pi/Tp
        
        
        self.waterDepth = waterDepth
        #Initial guess for wave number:
        self.k = self.getWaveNumber(initial=True)
    
        self.regular = regular #True = regular, False = irregular
        
        self.time = 0
        
    def setHeading(self, heading):
        """
        This function must be called every time the vessel state are updated!

        Args:
            heading (float): vessel's heading
        """
        self.angleWaveBody = self.angleWaveNED - heading
        
        
        
    
       
    def getFirstOrderLoad(self, waveFreq, angleWaveBody, waveAmplitude):
        
        headingIndex = np.argmin(np.abs(data.headingsData - angleWaveBody))
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
        
        firstOrderLoad = forceAmp*waveAmplitude*np.cos(waveFreq*self.time + forcePhase)
        
        # waveElevation=self.generateWave()
        # load = data.S*data.rho*data.g*waveElevation
        # loadX = -load*np.math.cos(self.angleWaveBody)
        # loadY = -load*np.math.sin(self.angleWaveBody)
        # loadPsi = 0
        #return [loadX, loadY, loadPsi]
        return firstOrderLoad
    
    def getDriftLoads(self, waveFreq, angleWaveBody, waveAmplitude):
        """
        This function takes wavefrequency and angle as input and find corresponding wave drift coefficient [N/A^2] in a lookup table.
        Returns wave drift load in 3DOF (output is 6DOF). These are based on Bjoernoe's data from CSAD.
        """
        headingIndex = np.argmin(np.abs(data.headingsData - angleWaveBody))
        frequencyIndex = np.argmin(np.abs(data.driftForceFreq - waveFreq))
        
        driftCoefficient = np.array([data.driftForceAmpX[frequencyIndex, headingIndex],
                            data.driftForceAmpY[frequencyIndex, headingIndex],
                            data.driftForceAmpPsi[frequencyIndex, headingIndex]])
        
        driftLoads = driftCoefficient*(waveAmplitude**2)
        driftLoads = math_tools.three2sixDof(driftLoads)
        return driftLoads
    
    def getSlowlyVaryingLoads(self, waveFreq, angleWaveBody, waveAmplitude):
        slowlyVaryingLoads = np.zeros(6)
        return slowlyVaryingLoads
        
    def getWaveLoads(self):
        """Outputs sum of wave loads in body frame

        Returns:
            array: sum of driftLoads, froudeKryolv and slowly varying wave loads
        """
        
        #This function must rely on wheter there is a regular or irregular state!
        #Regular waves:
        
        # if (self.regular == False):
        #     self.generateIrregular(self.time)
        
        #If the regular==True:
        firstOrderLoads = self.getFirstOrderLoad(self.frequency, self.angleWaveBody, self.amplitude)
        driftLoads = self.getDriftLoads(self.frequency, self.angleWaveBody, self.amplitude)
        slowlyVaryingLoads = self.getSlowlyVaryingLoads(self.frequency, self.angleWaveBody, self.amplitude)
        self.time += self.dt
        return driftLoads + firstOrderLoads # + slowlyVaryingLoads
    
    
        
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
        
        
    
    
    
    def generateIrregular(self, time):
        
        #First generate a spectrum, then make time varying sea state!!!
        phaseAngle = np.random.rand()
        
    
    
    
    
