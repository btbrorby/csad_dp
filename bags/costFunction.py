#!/usr/bin/env python
from pickletools import float8
from time import sleep
from matplotlib import pyplot as plt
import os
from tables import Float32Col
import rosbag
from rospy import Time
import numpy as np
from scipy import signal
from decimal import Decimal
from scipy.integrate import cumtrapz

import rospy




def costFunctionEta(time, etaX=[], etaY=[], etaYaw=[], tauSurge=[], tauSway=[], tauYaw=[]):
    Jdot_eta = np.abs(etaX) + np.abs(etaY) + (180.0/np.pi)*np.abs(etaYaw)
    
    J_eta = cumtrapz(Jdot_eta, dx=0.02)
    
    print(J_eta)
    fig = plt.figure()
    plt.plot(time[0:-1], J_eta)