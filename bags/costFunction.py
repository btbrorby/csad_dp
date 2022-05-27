#!/usr/bin/env python
from cProfile import label
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




def costFunctionEta(title, ax, time, etaX=[], etaY=[], etaYaw=[], init=False):
    Jdot_eta = np.abs(etaX)# + np.abs(etaY) + (180.0/np.pi)*np.abs(etaYaw)
    
    J_eta = cumtrapz(Jdot_eta, dx=0.02)
    
    
    # Jarray = np.re
    if init==True:
        fig, ax = plt.subplots(1,1)
        ax.plot(time[0:-1], J_eta, label=title)
    else:
        ax.plot(time[0:-1], J_eta, label=title)
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(0,0))
    ax.legend(loc='upper right')
    ax.grid()
    ax.set_xlim([0,time[-1]])
    
    print(title, ": seconds: ", time[-1], " J_eta: ", J_eta)
    return ax, J_eta

def costFunctionTau(title, ax, time, tauSurge=[], tauSway=[], tauYaw=[], init=False):
    Jdot_tauuv = np.abs(tauSurge)# + np.abs(tauSway)
    # Jdot_taur = np.abs(tauYaw)
    
    J_tauuv = cumtrapz(Jdot_tauuv, dx=0.02)
    
    
    # J_taur = cumtrapz(Jdot_taur, dx=0.02)
    if init==True:
        fig, ax = plt.subplots(1,1)
        ax.plot(time[0:-1], J_tauuv, label=title)
        # ax[1].plot(time[0:-1], J_taur, label=title)
    else:
        ax.plot(time[0:-1], J_tauuv, label=title)
        # ax[1].plot(time[0:-1], J_taur, label=title)
    
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(0,0))
    ax.legend(loc='upper right')
    ax.grid()
    ax.set_xlim([0,time[-1]])
    
    print(title, ": seconds: ", time[-1], " J_tau: ", J_tauuv)
        
    
    return ax, J_tauuv #, J_taur


  

def normalizeCost(time, title, array, type):

    I = np.argmax(array[:,-1])
    normArray = array.copy()
    count = 0
    np.seterr(invalid='ignore')
    for i in array[:]:
        normArray[count] = np.divide(i,array[I,-1])
        count += 1
    
    fig, ax = plt.subplots(1,1)
    count = 0
    for j in normArray[:]:
        ax.plot(time, j, label=title[count])
        
        if type=='tau':
            tIndex = np.argmin(np.abs(np.array(time) - 500.0))
            print(title[count], " seconds: ", time[tIndex] ,"J_tau:",j[tIndex])
            tIndex = np.argmin(np.abs(np.array(time) - time[-1]))
            print(title[count], " seconds:", time[tIndex] ,"J_tau:",j[tIndex])
        elif type == 'eta':
            tIndex = np.argmin(np.abs(np.array(time) - 500.0))
            print(title[count], " seconds: ", time[tIndex] ,"J_eta:",j[tIndex])
            tIndex = np.argmin(np.abs(np.array(time) - time[-1]))
            print(title[count], " seconds:", time[tIndex] ,"J_eta:",j[tIndex])
        
        count += 1
    
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(0,0))
    ax.legend(loc='upper left')
    ax.grid()
    ax.set_xlim([0,time[-1]])
    if type == 'eta':
        ax.set_ylabel(r'$J_{\eta}^c(t)$')
    elif type == 'tau':
        ax.set_ylabel(r'$J_{\tau}^c(t)$')
        
    ax.set_xlabel("Time [s]")
    
        
        