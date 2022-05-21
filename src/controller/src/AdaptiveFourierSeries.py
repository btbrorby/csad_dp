#!/usr/bin/env python
import rospy
import numpy as np
from lib import observer, reference, ps4, u_data, gains, tau
#from src.lib import tau
from math_tools import Rzyx, rad2pipi
from matplotlib import pyplot as plt
import yaml
import os

path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/simulator/include/seastateDefinitions.yaml".format(path)) as file:
    states = yaml.load(file, Loader=yaml.Loader)
class AdaptiveFourierSeries:
    def __init__(self, dt, seastate='calm', eta0=np.zeros([3,1]), nu0=np.zeros([3,1]), eta_d=np.zeros([3,1]), eta_d_dot=np.zeros([3,1]), eta_d_dotdot=np.zeros([3,1])):
        self.dt = dt
        self.time = 0.0
        
        self.eta = eta0
        self.nu = nu0
        self.eta_d = eta_d
        self.eta_d_dot = eta_d_dot
        self.eta_d_dotdot = eta_d_dotdot
        
        # self.MRB = np.array([[127.5, 0.0, 0.0],
        #                    [0.0, 127.5, 0.0],
        #                    [0.0, 0.0, 61.8]])
        # self.MA = np.array([[3.698, 0.0, 0.0],
        #                     [0.0, 29.118, 1.156],
        #                     [0.0, 1.156, 12.61]])
        # self.M = np.add(self.MRB, self.MA)
        
        # self.D_viscous = np.array([[16.43, 0.0, 0.0],
        #                            [0.0, 128.77, 0.0],
        #                            [0.0, 0.0, 50.51]])
        # self.D = self.D_viscous #+...
        
        self.M = np.array([[140.0, 0.0, 0.0],
                           [0.0, 228.0, 7.34],
                           [0.0, 7.34, 104.9]])
        self.M_inv = np.linalg.inv(self.M)
        self.D = np.array([[80.66, 0.0, 0.0],
                           [0.0, 632.09, 34.67],
                           [0.0, 34.67, 228.07]])
        
        
        #Tuning parameters
        self.N = 15
        self.C1 = np.diag([10.0, 1.0, 1.0])
        self.C2 = np.diag([10.0, 1.0, 1.0])
        self.K = np.diag([1.0, 1.0, 1.0])
        self.Gamma = 5.0*np.eye(2*self.N + 1)
        
        Tp_min = 2.0
        Tp_max = 20.0
        self.wLower = 2.0*np.pi/Tp_max
        self.wUpper = 2.0*np.pi/Tp_min
        self.dw = (self.wUpper-self.wLower)/self.N
        print(self.dw)
        
        self.frequencies = np.arange(self.wLower, self.wUpper, self.dw)
        print(self.frequencies)
        self.theta_hat = 0.0*np.ones([2*self.N+1, 1])
        

        
        
    def updateStates(self, eta, nu, eta_d=np.zeros([3,1]), eta_d_dot=np.zeros([3,1]), eta_d_dotdot=np.zeros([3,1])):
        eta = np.resize(eta, (3,1))
        nu = np.resize(nu, (3,1))
        eta_d = np.resize(eta_d, (3,1))
        eta_d_dot = np.resize(eta_d_dot, (3,1))
        eta_d_dotdot = np.resize(eta_d_dotdot, (3,1))
        
        self.eta = eta
        self.nu = nu
        self.eta_d = eta_d
        self.eta_d_dot = eta_d_dot
        self.eta_d_dotdot = eta_d_dotdot


    def getControllerOutput(self):
        R = Rzyx(self.eta[2])
        R_transpose = np.transpose(R)
        S = np.array([[0.0, self.nu[2], 0.0],
                      [-self.nu[2], 0.0, 0.0],
                      [0.0, 0.0, 0.0]])
        
        regressor_transpose = self.getRegressor(self.frequencies, self.time, self.N)
        
        Phi_transpose = np.array([regressor_transpose,
                                  regressor_transpose,
                                  regressor_transpose])
        Phi = np.transpose(Phi_transpose)
        
        z1 = np.matmul(R_transpose, self.eta - self.eta_d)
        z1 = z1.astype(float)
        
        theta_hat_dot = np.matmul(np.matmul(self.Gamma, Phi), z1)
        theta_hat_dot = theta_hat_dot.astype(float)
        
        self.theta_hat += theta_hat_dot*self.dt
        
        alpha0 = -np.matmul(self.K, z1)
        alpha0 = alpha0.astype(float)
        
        alpha = -np.matmul(self.C1, z1) + np.matmul(R_transpose, self.eta_d_dot) + alpha0
        alpha = alpha.astype(float)
        
        z2 = self.nu - alpha
        
        z1_dot = -np.matmul(S, z1) + z2 - np.matmul(self.C1 + self.K, z1)
        z1_dot = z1_dot.astype(float)
        
        alpha_dot = -np.matmul(self.C1 + self.K, z1_dot) - np.matmul(np.matmul(S, R_transpose), self.eta_d_dot) + np.matmul(R_transpose, self.eta_d_dotdot)        
        alpha_dot = alpha_dot.astype(float)
        
        tau = -np.matmul(self.C2, z2) + np.matmul(self.D, alpha) + np.matmul(self.M, alpha_dot) - np.matmul(Phi_transpose, self.theta_hat)
        tau = tau.astype(float)
        d = np.matmul(Phi_transpose, self.theta_hat)
        
        self.time += self.dt
        
        return tau, d

    def getRegressor(self, frequencies, time, N):
        regressor = [1.0]
        for i in range(N):
            regressor.append(np.cos(frequencies[i]*time))
            regressor.append(np.sin(frequencies[i]*time))
        return regressor
    
    
path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/controller/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)

dt = 1.0/params["runfrequency"]
controller = AdaptiveFourierSeries(dt, seastate='rough')


def loop():
    eta_hat = observer.eta_hat
    nu_hat = observer.nu_hat
    bias_hat = observer.bias_hat
    
    controller.updateStates(eta_hat, nu_hat)
    [controlOutput, d] = controller.getControllerOutput()
    tau.publish(controlOutput, controller.time)
    tau.publishHelper(d, controller.time)
    
    

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
    
    