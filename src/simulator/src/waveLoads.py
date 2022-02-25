#!/usr/bin/env python

from glob import glob
from re import X
import numpy as np
import math
import math_tools
from generateModelData_CSAD import g


class Wave():
    def __init__(self, dt, angle, Hs, Tp, regular=True):
        
        self.dt = dt
        
        self.Hs = Hs
        self.Tp = Tp
        self.angle = angle
        self.frequency = 2*math.pi/Tp
        
    
        self.regular = regular #True = regular, False = irregular
        
        
    def getWaveNumber(self, frequency):
        """
        Assuming deep water conditions, giving the wave number as a function of wave frequency
        """
        
        return (frequency**2)/g
    
    
    def generateRegular(self, time, x):
        amplitude = self.Hs/2
        k = self.getWaveNumber(self.frequency)
        surfaceHeight = amplitude*math.sin(self.frequency*time - k*x)
        return surfaceHeight
        
    def generateIrregular(self, time, x, y):
        phaseAngle = np.random.rand()
        

    
        
    