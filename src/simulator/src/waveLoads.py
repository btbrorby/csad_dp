#!/usr/bin/env python

from cmath import pi
import numpy as np
import math
import math_tools
from generateModelData_CSAD import g
from scipy import optimize as opt




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
        if ((self.waterDepth*(self.frequency**2))/(g*2*pi) >= 0.5):
            deepWater = False
        else:
            deepWater = True
        return deepWater
        
    def getWaveNumber(self, deepWater=True, waterDepth=np.infty):
        """
        Estimates the wave number based on the dispertion relation.
        """
        if (deepWater==False):
            f = g*self.k*np.math.tanh(self.k*waterDepth) - self.frequency**2
            while (np.abs(f) >= 0.1):
                self.k += 0.0001
                f = g*self.k*np.math.tanh(self.k*waterDepth) - self.frequency**2
                
            return self.k
        else:
            return (self.frequency**2)/g
    
    
    def generateRegular(self, x=0):
        
        
        
        amplitude = self.Hs/2
        
        deepWaterCondition = self.checkDeepWaterConditions()
        k = self.getWaveNumber(self.frequency, deepWater=deepWaterCondition)
        
        surfaceHeight = amplitude*math.cos(self.frequency*self.time - k*x)
        
        self.time += self.dt
        return surfaceHeight
        
    def generateIrregular(self, time, x, y):
        phaseAngle = np.random.rand()
        
        
    def waveLoadsFirst(self):
        return 0
    
    def waveLoadsSecond(self):
        return 0
    
    def publishWaveLoads(self):
        load = self.waveLoadsFirst() + self.waveLoadsSecond()

    
        
obj = Wave(0.5, 1)
print(obj.getWaveNumber(shallowWater=True, waterDepth=0.2))