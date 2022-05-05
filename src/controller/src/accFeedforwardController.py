#!/usr/bin/env python
import numpy as np
import math
from lib import odometry, observer, reference, ps4, u_data, gains, tau, imu1, imu2, imu3, imu4
from math_tools import Rzyx, rad2pipi
from matplotlib import pyplot as plt
import yaml
import os

import rospy


class AccFeedforwardController:
    def __init__(self, dt, eta0 = np.zeros([3, 1]), nu0 = np.zeros([3, 1]), eta_d = np.zeros([3, 1]), eta_d_dot = np.zeros([3, 1]), eta_d_dotdot = np.zeros([3, 1])):
        
        self. dt = dt
        
        self.eta0 = eta0
        self.nu0 = nu0
        self.eta_d = eta_d
        self.eta_d_dot = eta_d_dot
        self.eta_d_dotdot = eta_d_dotdot
        
        #States:
        self.nu_hat = nu0
        self.p_nu_hat = np.zeros([3, 1])
        self.g_hat = np.zeros([3, 1])
        self.g_hat[2] = -9.81
        self.b_l_hat = np.zeros([3, 1])
        self.omega_hat = np.zeros([3, 1])
        self.b_w_hat = np.zeros([3, 1])
        
        self.tau = np.zeros([3, 1])
        
        #Selection matrices:
        self.B_1 = np.zeros([3, 12])
        self.B_2 = np.zeros([3, 12])
        self.B_1[0,0] = self.B_1[1,0] = self.B_1[2,0] = 1.0
        self.B_2[0,1] = self.B_2[1,1] = self.B_2[2,1] = 1.0
        
        #Tuning gains:
        I = np.eye(3)
        self.k1 = np.array([1.0, 1.0, 1.0])*I
        self.k2 = np.array([1.0, 1.0, 1.0])*I
        
        self.D = np.array([[16.43, 0.0, 0.0],
                           [0.0, 128.77, 0.0],
                           [0.0, 0.0, 50.51]])
        
        # self.imu1 = np.zeros([6, 1])   
        # self.imu2 = np.zeros([6, 1])   
        # self.imu3 = np.zeros([6, 1])   
        # self.imu4 = np.zeros([6, 1])   
        
        
    
        
        
    def getW(self, id, paramsImu):
        
        I_33 = np.eye(3)
        
        imuName = 'imu'+str(id)
        locations = paramsImu['imu_locations'][imuName]
        lx = locations['x']
        ly = locations['y']
        lz = locations['z']
        
        l = math.sqrt(lx**2 + ly**2 + lz**2)
        #is this rigth?
        S_l_transposed = np.array([[0.0, -l, 0.0],
                                   [l, 0.0, 0.0],
                                   [0.0, 0.0, 0.0]])
        S = self.getS(np.array([lx, ly, lz]))
        S_l_transposed = np.transpose(S)
        
        H_l = np.array([[0.0, -lx, -lx, ly, lz, 0.0],
                        [-ly, 0.0, -ly, lx, 0.0, lz],
                        [-lz, -lz, 0.0, 0.0, lx, ly]])
        
        W_l = np.concatenate((I_33, S_l_transposed, H_l), axis=1)
        
        return W_l
        
        
    def getControlOutput(self, measurements, paramsImu):
            
        G = np.zeros([4*3, 12])
        for i in range(3, 15, 3):
            W_l = self.getW(i/3, paramsImu)
            G[i-3:(i)][:] = W_l
            
        G_inverted = np.linalg.inv(G)
        
        acc_hat = controller.stateObserver(G_inverted, measurements) #measurements are 12 element vector
        
        p_nu_d = np.matmul(np.transpose(Rzyx(self.eta_d[-1])), self.eta_d)
        nu_d = np.matmul(np.transpose(Rzyx(self.eta_d[-1])), self.eta_d_dot)
        nu_d_dot = np.matmul(np.transpose(Rzyx(self.eta_d[-1])), self.eta_d_dotdot)
        
        p_tilde = self.p_nu_hat - p_nu_d
        nu_tilde = self.nu_hat - nu_d
        
        Gamma = - np.matmul(self.k1, p_tilde) - np.matmul(self.k2, nu_tilde) + nu_d_dot - np.matmul(self.D, self.nu_hat)
        Delta = acc_hat - self.tau
        
        self.tau = Gamma - Delta
        
        return self.tau
    
    def getS(self, vec):
        p = vec[0]
        q = vec[1]
        r = vec[2]
        S = np.array([[0.0, -r, q],
                      [r, 0.0, -p],
                      [-q, p, 0.0]])
        return S
        
    
    def stateObserver(self, G_inverse, a_mc):
        
        b_w_dot = 0.0
        b_l_dot = 0.0
        omega_dot = self.b_w_hat + np.matmul(np.matmul(self.B_2, G_inverse), a_mc)
        self.omega_hat += omega_dot*self.dt
        
        S_omega = self.getS(self.omega_hat)
        
        g_dot = -np.matmul(S_omega, self.g_hat)
        g_dot = g_dot.astype(float)
        self.g_hat += g_dot*self.dt
        
        nu_dot = -np.matmul(S_omega, self.nu_hat) - self.b_l_hat - self.g_hat + np.matmul(np.matmul(self.B_1, G_inverse), a_mc)
        nu_dot = nu_dot.astype(float)
        self.nu_hat += nu_dot*self.dt
        
        p_nu_dot = -np.matmul(S_omega, self.p_nu_hat) + self.nu_hat
        p_nu_dot = p_nu_dot.astype(float)
        self.p_nu_hat += p_nu_dot*self.dt
        
        # x = np.transpose(np.array([self.p_nu_hat, self.nu_hat, self.b_l_hat, self.g_hat, self.omega_hat, self.b_w_hat]))
        return nu_dot
    


path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/controller/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)
dt = 1.0/params["runfrequency"]

controller = AccFeedforwardController(dt)

paramPath = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/simulator/src/sensorParams.yaml".format(paramPath)) as file:
    paramsImu = yaml.load(file, Loader=yaml.Loader)
        
def loop():
    eta_hat = observer.eta_hat
    nu_hat = observer.nu_hat
    bias_hat = observer.bias_hat
    
    measurement_imu1 = imu1.getImuMeasurement()
    measurement_imu2 = imu2.getImuMeasurement()
    measurement_imu3 = imu3.getImuMeasurement()
    measurement_imu4 = imu4.getImuMeasurement()
    
    imu1_acc = measurement_imu1[0:3]
    imu2_acc = measurement_imu2[0:3]
    imu3_acc = measurement_imu3[0:3]
    imu4_acc = measurement_imu4[0:3]
    measurements_acc = np.concatenate((measurement_imu1[0:3], measurement_imu2[0:3], measurement_imu3[0:3], measurement_imu4[0:3]))

    controlOutput = controller.getControlOutput(measurements_acc, paramsImu)
    tau.publish(controlOutput)
    # rospy.spin()