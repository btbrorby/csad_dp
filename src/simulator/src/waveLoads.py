#!/usr/bin/env python

from cmath import pi
import numpy as np
import math
import math_tools
from scipy import optimize as opt
import generateModelData_CSAD as data
import matplotlib.pyplot as plt




class Wave():
    def __init__(self, Hs, Tp, waterDepth=np.infty, angle=0, regular=True, dt=0.01):
        
        self.dt = dt
        
        self.Hs = Hs
        self.Tp = Tp
        self.angle = angle
        self.frequency = 2*math.pi/Tp
        
        self.waterDepth = waterDepth
        #Initial guess for wave number:
        self.k = self.getWaveNumber(deepWater=True)
    
        self.regular = regular #True = regular, False = irregular
        
        self.time = 0
        
    
       
    def checkDeepWaterConditions(self):
        """
        Checks whether deep water conditionas are valid.
        """
        if ((self.waterDepth*(self.frequency**2))/(data.g*2*pi) >= 0.5):
            deepWater = False
        else:
            deepWater = True
        return deepWater
        
    def getWaveNumber(self, deepWater=True):
        """
        Estimates the wave number based on the dispertion relation.
        """
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
        
            deepWaterCondition = self.checkDeepWaterConditions()
            k = self.getWaveNumber(deepWater=deepWaterCondition)

            waveElevation = amplitude*math.cos(self.frequency*self.time - k*x)

            self.time += self.dt
            
        else: #Irregular
            waveElevation = 0
            
        return waveElevation
        
        
    def FroudeKrylov(self):
        waveElevation=self.generateWave()
        return data.S*data.rho*data.g*waveElevation
    
    
    def generateIrregular(self, time, x, y):
        phaseAngle = np.random.rand()
        
        
    def waveLoadsFirst(self):
        return 0
    
    def waveLoadsSecond(self):
        return 0
    
    def publishWaveLoads(self):
        load = self.waveLoadsFirst() + self.waveLoadsSecond()

    
    
#For testing:
        
obj = Wave(0.09/2, 1)
t = 0
dt = 0.1
fig = plt.figure()

time=[]
elev=[]
force = []
count = 0
while (t < 10):
    
    elev.append(obj.generateWave())
    force.append(obj.FroudeKrylov())
    time.append(t)
    t += dt
    count +=1
    
plt.plot(time, elev)
plt.plot(time, force)


    
plt.show()