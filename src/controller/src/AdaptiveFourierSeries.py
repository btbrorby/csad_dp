#!/usr/bin/env python
from tkinter import Label
import rospy
import numpy as np
import math
from lib import odometry, observer, reference, ps4, u_data, gains, tau
#from src.lib import tau
from math_tools import Rzyx, rad2pipi
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from matplotlib import pyplot as plt


class AdaptiveFourierSeries:
    def __init__(self, dt, eta0=np.zeros([3,1]), nu0=np.zeros([3,1]), eta_d=np.zeros([3,1]), eta_d_dot=np.zeros([3,1]), eta_d_dotdot=np.zeros([3,1])):
        self.dt = dt
        self.time = 0.0
        
        self.eta = eta0
        self.nu = nu0
        self.eta_d = eta_d
        self.eta_d_dot = eta_d_dot
        self.eta_d_dotdot = eta_d_dotdot
        
        self.MRB = np.array([[127.5, 0.0, 0.0],
                           [0.0, 127.5, 0.0],
                           [0.0, 0.0, 61.8]])
        self.MA = np.array([[3.698, 0.0, 0.0],
                            [0.0, 29.118, 1.156],
                            [0.0, 1.156, 12.61]])
        self.M = np.add(self.MRB, self.MA)
        
        self.D_viscous = np.array([[16.43, 0.0, 0.0],
                                   [0.0, 128.77, 0.0],
                                   [0.0, 0.0, 50.51]])
        self.D = self.D_viscous #+...
        
        
        #Tuning parameters
        self.N = 1
        self.C1 = 10.0*np.ones([3,3])
        self.C2 = 10.0*np.ones([3,3])
        self.K = 5.0*np.ones([3,3])
        self.Gamma = 0.1*np.ones([2*self.N + 1, 2*self.N + 1])
        self.dw = 0.1
        self.wLower = 0.2
        self.wUpper = 9.3
        
        self.frequencies = np.arange(self.wLower, self.wUpper, self.dw)

        self.theta_hat = np.zeros([2*self.N+1, 1])
        

        
        
    def updateStates(self, eta, nu):
        self.eta = eta
        self.nu = nu

    def controller(self, time):
        R = Rzyx(self.eta[2])
        R_transpose = np.transpose(R)
        S = np.array([[0.0, self.nu[2], 0.0],
                      [-self.nu[2], 0.0, 0.0],
                      [0.0, 0.0, 0.0]])
        
        regressor_transpose = self.getRegressor(self.frequencies, time, self.N)
        Phi_transpose = np.array([regressor_transpose,
                                  regressor_transpose,
                                  regressor_transpose])
        Phi = np.transpose(Phi_transpose)
        
        z1 = np.matmul(R_transpose, self.eta - self.eta_d)
        theta_hat_dot = np.matmul(np.matmul(self.Gamma, Phi), z1)
        self.theta_hat += theta_hat_dot*self.dt
        alpha0 = -np.matmul(self.K, z1)
        alpha = -np.matmul(self.C1, z1) + np.matmul(R_transpose, self.eta_d_dot + alpha0)
        z2 = self.nu - alpha
        z1_dot = -np.matmul(S, z1) + z2 - np.matmul((self.C1 + self.K), z1)
        alpha_dot = -np.matmul(self.C1 + self.K, z1_dot) -np.matmul(np.matmul(S, R_transpose), self.eta_d_dot) + np.matmul(R_transpose, self.eta_d_dotdot)        
        
        tau = -np.matmul(self.C2, z2) + np.matmul(self.D, alpha) + np.matmul(self.M, alpha_dot) - np.matmul(Phi_transpose, self.theta_hat)

        return tau

    def getRegressor(self, frequencies, time, N):
        regressor = [1.0]
        for i in range(N):
            regressor.append(np.cos(frequencies[i]*time))
            regressor.append(np.sin(frequencies[i]*time))
        return regressor

# controller = AdaptiveFourierSeries(0.01, eta0=np.array([1.0, 2.0, 3.0]).reshape((3,1)))


# time = 0.0
# timeVec = []
# tauVec = []
# resVec = []
# etaVec = []
# eta = np.zeros([3, 1])
# nu = np.zeros([3, 1])
# while time < 500:
#     x = 100.0 + 100.0*np.math.cos(controller.frequencies[3]*time)
#     eta[0] = x
#     etaVec.append(x)
#     controller.updateStates(eta, nu)
#     tau = controller.controller(time)
#     tauVec.append(tau[0])
#     timeVec.append(time)
#     time += 0.01
    
# fig = plt.figure()
# plt.plot(timeVec, tauVec, Label="tau")
# plt.plot(timeVec, etaVec)
# plt.legend()
# plt.show()

# # fig = plt.figure()
# # plt.plot(timeVec, tauVec)
# # plt.show()

# # F = np.fft.fft(tauVec)
# n = len(timeVec)
# fhat = np.fft.fft(tauVec, n)
# PSD = fhat*np.conj(fhat)/n
# F = (2*np.pi/(0.01*n))*np.arange(n)
# L = np.arange(1,np.floor(n/2), dtype='int')

# freg = np.fft.ifft(fhat, n)
# fig = plt.figure()
# plt.plot(F[L], PSD[L])

# fig = plt.figure()
# plt.plot(timeVec, tauVec)
# plt.plot(timeVec, freg, ls='--')
# plt.show()
    
    