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
        
        
        
        
    def getDriftLoads(self, waveFreq, angleWaveBody, waveAmplitude):
        """
        This function takes wavefrequency and angle as input and find corresponding wave drift coefficient [N/A^2] in a lookup table.
        Returns wave drift load in 3DOF. These are based on Bjoernoe's data from CSAD.
        """
        headingIndex = np.argmin(np.abs(data.headingsData - angleWaveBody))
        frequencyIndex = np.argmin(np.abs(data.driftForceFreq - waveFreq))
        
        driftCoefficient = np.array([data.driftForceAmpX[frequencyIndex, headingIndex],
                            data.driftForceAmpY[frequencyIndex, headingIndex],
                            data.driftForceAmpPsi[frequencyIndex, headingIndex]])
        
        driftLoads = driftCoefficient*(waveAmplitude**2)
        return driftLoads
    
       
    def getFroudeKrylov(self):
        waveElevation=self.generateWave()
        load = data.S*data.rho*data.g*waveElevation
        loadX = -load*np.math.cos(self.angleWaveBody)
        loadY = -load*np.math.sin(self.angleWaveBody)
        loadPsi = 0
        return [loadX, loadY, loadPsi]
        
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
    
    
    def generateWave(self, x=0):
        
        if (self.regular==True): #Regular
            amplitude = self.Hs/2
        
            
            k = self.getWaveNumber()

            waveElevation = amplitude*math.cos(self.frequency*self.time - k*x)

            self.time += self.dt
            
        else: #Irregular
            waveElevation = 0
            
        return waveElevation
        
        
    
    
    
    def generateIrregular(self, time, x, y):
        #First generate a spectrum, then make time varying sea state!!!
        phaseAngle = np.random.rand()
        
    
    
    def getWaveLoads(self):
        #This function must rely on wheter there is a regular or irregular state!
        driftLoads = self.getDriftLoads(self.frequency, self.angleWaveBody, self.Hs/2)
        froudeKrylovLoads = self.getFroudeKrylov()
        #slowlyVaryingLoads = ...
        
        return driftLoads + froudeKrylovLoads # + slowlyVaryingLoads
    
#For testing:
        
obj = Wave(0.04*2, 1)
t = 0
dt = 0.1
fig = plt.figure()

print(obj.getDriftLoads(obj.frequency, obj.angleWaveBody, 0.04))

time=[]
elev=[]
force = []
count = 0
while (t < 10):
    
    elev.append(obj.generateWave())
    force.append(obj.getFroudeKrylov())
    time.append(t)
    t += dt
    count +=1
    
plt.plot(time, elev)
plt.plot(time, force)


    
#plt.show()