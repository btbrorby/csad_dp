#!/usr/bin/env python
import numpy as np
import math
from lib import qualisys, observer, reference, ps4, u_data, gains, tau, imu1, imu2, imu3, imu4, thrust
from math_tools import Rzyx, rad2pipi
from matplotlib import pyplot as plt
import yaml
import os

import rospy

paramPath = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/simulator/src/sensorParams.yaml".format(paramPath)) as file:
    paramsImu = yaml.load(file, Loader=yaml.Loader)
class AccFeedforwardController:
    def __init__(self, dt=0.02, eta0 = np.zeros([3, 1]), nu0 = np.zeros([3, 1]), eta_d = np.zeros([3, 1]), eta_d_dot = np.zeros([3, 1]), eta_d_dotdot = np.zeros([3, 1])):
        
        self.dt = dt
        self.time = 0.0
        self.eta0 = eta0
        self.nu0 = nu0
        self.eta_d = eta_d
        self.eta_d_dot = eta_d_dot
        self.eta_d_dotdot = eta_d_dotdot
        
        #States:
        self.nu = nu0
        self.p_nu = np.zeros([3, 1])
        self.v = np.zeros([3, 1])
        self.g = np.zeros([3, 1])
        self.g[2] = 9.81
        self.b_l = np.zeros([3, 1])
        self.omega = np.zeros([3, 1])
        self.b_w = np.zeros([3, 1])
        
        self.delta = np.zeros([3, 1])
        self.tau = np.zeros([3, 1])
        
        #Selection matrices:
        self.B_1 = np.zeros([3, 12])
        self.B_2 = np.zeros([3, 12])
        self.B_1[0,0] = self.B_1[1,1] = self.B_1[2,2] = 1.0
        self.B_2[0,3] = self.B_2[1,4] = self.B_2[2,5] = 1.0
        
        #Tuning gains:
        self.Kp = 2.0*np.pi*2.0*np.pi*np.diag([1.8982, 1.0, 1.0])
        # self.Kd = np.diag([10.3, 0.0, 0.0])
        self.Kd = np.diag([204.8, 100.0, 100.0])
        self.mu = 0.0
        
        self.M = np.array([[140.0, 0.0, 0.0],
                           [0.0, 228.0, 7.34],
                           [0.0, 7.34, 104.9]])
        
        self.D = np.array([[80.66, 0.0, 0.0],
                           [0.0, 632.09, 34.67],
                           [0.0, 34.67, 228.07]])
        
        
        G = np.zeros([4*3, 12])
        for i in range(3, 15, 3):
            W_l = self.getW(i/3, paramsImu)
            G[i-3:(i)][:] = W_l
        self.G_inverse = np.linalg.inv(G)
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
        
        # l = math.sqrt(lx**2 + ly**2 + lz**2)
        # #is this rigth?
        # S_l_transposed = np.array([[0.0, -l, 0.0],
        #                            [l, 0.0, 0.0],
        #                            [0.0, 0.0, 0.0]])
        S = self.getS(np.array([lx, ly, lz]))
        S_l_transposed = np.transpose(S)
        
        H_l = np.array([[0.0, -lx, -lx, ly, lz, 0.0],
                        [-ly, 0.0, -ly, lx, 0.0, lz],
                        [-lz, -lz, 0.0, 0.0, lx, ly]])
        
        W_l = np.concatenate((I_33, S_l_transposed, H_l), axis=1)
        
        return W_l
        
        
    # def getControlOutput(self, measurements, paramsImu):
    #     G = np.zeros([4*3, 12])
    #     for i in range(3, 15, 3):
    #         W_l = self.getW(i/3, paramsImu)
    #         G[i-3:(i)][:] = W_l
            
    #     G_inverted = np.linalg.inv(G)
    #     acc_hat = self.stateObserver(G_inverted, measurements) #measurements are 12 element vector
        
    #     p_nu_d = np.matmul(np.transpose(Rzyx(self.eta_d[-1])), self.eta_d)
    #     nu_d = np.matmul(np.transpose(Rzyx(self.eta_d[-1])), self.eta_d_dot)
    #     nu_d_dot = np.matmul(np.transpose(Rzyx(self.eta_d[-1])), self.eta_d_dotdot)
        
    #     p_tilde = self.p_nu_hat - p_nu_d
    #     nu_tilde = self.nu_hat - nu_d
        
    #     S = self.getS(np.array([0.0, 0.0, self.nu_hat[2]]))
        
    #     Gamma = - np.matmul(self.k1, p_tilde) - np.matmul(self.k2, nu_tilde) + nu_d_dot - np.matmul(self.D, self.nu_hat)
        
        
    #     Delta = acc_hat - self.tau
        
    #     self.tau = Gamma - Delta
        
    #     return self.tau
    
    def getControlOutput(self, eta, nu, a_mc, thrustMeasured):
        
        R = Rzyx(eta[5])
        R_transpose = np.transpose(R)
        
        [p_nu, v, omega, v_dot, omega_dot] = self.stateObserver(a_mc, eta, nu) 
        v_dot = np.resize(v_dot, (3,1))
        omega_dot = np.resize(omega_dot, (3,1))
        
        a = np.array([v_dot[0], v_dot[1], omega_dot[2]])
        a = np.resize(a, (3,1))
        
        eta_nu_hat = np.array([p_nu[0], p_nu[1], eta[5]])
        eta_nu_hat = np.resize(eta_nu_hat, (3,1))
        eta_body_tilde = p_nu - np.matmul(R_transpose, self.eta_d)
        
        nu_hat = np.array([v[0], v[1], omega[2]])
        nu_hat = np.resize(nu_hat, (3,1))
        nu_tilde =  nu_hat - np.matmul(R_transpose, self.eta_d_dot)
        S_r = self.getS(np.array([0.0, 0.0, omega[2]]))
        Gamma = - np.matmul(self.Kp, eta_body_tilde) - np.matmul(self.Kd, nu_tilde)
        Gamma = Gamma.astype(float)
        # Gamma = -np.matmul((np.matmul(self.Kd, self.Kp) - np.matmul(np.matmul(self.M, self.Kp), S_r)), eta_body_tilde) - np.matmul((self.Kd + np.matmul(self.M, self.Kp)), nu_tilde)
        tau0 = np.array([thrustMeasured[0], thrustMeasured[1], thrustMeasured[5]])
        tau0 = np.resize(tau0, (3,1))
        
        delta_dot = self.mu*(np.matmul(self.M, a) - Gamma) # - self.delta + np.matmul(self.D, nu_hat))
        delta_dot = delta_dot.astype(float)
        self.delta += delta_dot*self.dt
        
        self.tau = Gamma - self.delta
        
        return self.tau, a
    
    def getS(self, vec):
        p = vec[0]
        q = vec[1]
        r = vec[2]
        S = np.array([[0.0, -r, q],
                      [r, 0.0, -p],
                      [-q, p, 0.0]])
        return S
        
    
    def stateObserver(self, a_mc, eta, nu):
        
        R = Rzyx(eta[5])
        R_transposed = np.transpose(R)
        
        p_nu = np.matmul(R_transposed, eta[0:3])
        p_nu = np.resize(p_nu, (3,1))
        
        omega = nu[3:6]
        omega = np.resize(omega, (3,1))
        v = nu[0:3]
        v = np.resize(v, (3,1))
        
        S_omega = self.getS(omega)
        
        p_nu_dot = -np.matmul(S_omega, p_nu) + v
        p_nu_dot = p_nu_dot.astype(float)
        
        v_dot = -np.matmul(S_omega, v) - self.b_l - self.g + np.matmul(np.matmul(self.B_1, self.G_inverse), a_mc)
        v_dot = v_dot.astype(float)
        
        g_dot = -np.matmul(S_omega, self.g)
        g_dot = g_dot.astype(float)
        
        b_l_dot = np.zeros([3,1])
        
        omega_dot = self.b_w + np.matmul(np.matmul(self.B_2, self.G_inverse), a_mc)
        omega_dot = omega_dot.astype(float)
        
        b_w_dot = np.zeros([3,1])
        
        # self.b_l += b_l_dot*self.dt
        # self.b_w += b_w_dot*self.dt
        # self.g += g_dot*self.dt
        # self.p_nu += p_nu_dot*self.dt
        # self.v += v_dot*self.dt
        # self.omega += omega_dot*self.dt
        
        
        return p_nu, v, omega, v_dot, omega_dot
        
    


path = os.path.dirname(os.getcwd())
with open(r"{0}/csad_dp_ws/src/controller/src/params.yaml".format(path)) as file:
    params = yaml.load(file, Loader=yaml.Loader)
timestep = 1.0/params["runfrequency"]

controller = AccFeedforwardController(dt = timestep)


        
def loop():
    # eta_hat = observer.eta_hat
    # eta_hat = np.resize(eta_hat, (3,1))
    # nu_hat = observer.nu_hat
    # nu_hat = np.resize(nu_hat, (3,1))
    # bias_hat = observer.bias_hat
    # bias_hat = np.resize(bias_hat, (3,1))
    
    
    [eta, nu] = qualisys.getQualisysOdometry()
    eta = np.resize(eta, (6,1))
    nu = np.resize(nu, (6,1))
    
    measurement_imu1 = imu1.getImuMeasurement()
    measurement_imu2 = imu2.getImuMeasurement()
    measurement_imu3 = imu3.getImuMeasurement()
    measurement_imu4 = imu4.getImuMeasurement()
    
    # thrustMeasured = thrust.getThrustLoads()
    # thrustMeasured = np.resize(thrustMeasured, (6,1))
    thrustMeasured = np.zeros([6,1])
    
    # imu1_acc = measurement_imu1[0:3]
    # imu2_acc = measurement_imu2[0:3]
    # imu3_acc = measurement_imu3[0:3]
    # imu4_acc = measurement_imu4[0:3]
    a_mc = np.concatenate((measurement_imu1[0:3], measurement_imu2[0:3], measurement_imu3[0:3], measurement_imu4[0:3]))
    # controlOutput = controller.getControlOutput(measurements_acc, paramsImu)
    [controlOutput, a] = controller.getControlOutput(eta, nu, a_mc, thrustMeasured)
    
    tau.publish(controlOutput, controller.time)
    # tau.publish(a, controller.time)
    controller.time += timestep
    # rospy.spin()