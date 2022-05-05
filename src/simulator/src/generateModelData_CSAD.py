#!/usr/bin/env python
from fileConfig import loadmat
import numpy as np
import os

file_name_ABC = "include/CSAD_modelABC.mat"
file_name = "include/CSAD_model.mat"

path = os.path.dirname(os.getcwd())
vessel = loadmat(r"{0}/csad_dp_ws/src/simulator/include/CSAD_model.mat".format(path))['vessel']             # Loads MSS vessel structure
vesselABC = loadmat(r"{0}/csad_dp_ws/src/simulator/include/CSAD_modelABC.mat".format(path))['vesselABC']    # Loads fluid memmory state-space model



name = vessel['main']['name']       # Name of ship
Lpp = vessel['main']['Lpp']         # Length between perpendiculars
T = vessel['main']['T']             # Draught
B = vessel['main']['B']             # Breadth
CG = vessel['main']['CG']           # Center of gravity w.r.t baseline and Lpp/2
m = vessel['main']['m']             # Mass
k44 = vessel['main']['k44']         # Radius of gyration in roll (m)
k55 = vessel['main']['k55']         # Radius of gyration in pitch (m)
k66 = vessel['main']['k66']         # Radius of gyration in yaw (m)
nabla = vessel['main']['nabla']     # Volume displacement
GM_L = vessel['main']['GM_L']       # Lateral metacentric height
GM_T = vessel['main']['GM_T']       # Transverse metacentric height
C_B = vessel['main']['C_B']         # Block coefficient
CB = vessel['main']['CB']           # Center of bouancy w.r.t baseline and Lpp/2
Lwl = vessel['main']['Lwl']         # Length of water line
S = vessel['main']['S']             # Wetted surface
MRB = vesselABC['MRB']              # Rigid body system inertia matrix
MA = vesselABC['MA']                # Added mass matrix

A = vessel['A']                     # Added mass 6DOF for 47 frequencies
B = vessel['B']                     # Potential + viscous damping 6DOF for 47 frequencies
C = vessel['C']                     # Restoring forces 6DOF for 47 frequencies
frequencies = vessel['freqs']       # Frequencies defined for A, B and C

RAO_FORCE = vessel['forceRAO']
RAO_MOTION = vessel['motionRAO']

headingsData = vessel['headings']
driftForceFreq = vessel['driftfrc']['w']
driftForceAmpX = vessel['driftfrc']['amp'][0][:,:,0]
driftForceAmpY = vessel['driftfrc']['amp'][1][:,:,0]
driftForceAmpPsi = vessel['driftfrc']['amp'][2][:,:,0]


rho = vessel['main']['rho']           # water density
g = vessel['main']['g']               # Acceleration of gravity

Xu = -2.33
Xuu = 0.0
Xuuu = -8.56
Xv = 0.0
Xvv = 0.0
Xvvv = 0.0
Yv = -4.67
Yvv = 0.398
Yvvv = -313.0
Yr = -7.25
Yrr = -3.45
Yrrr = 0.0
Yrv = -0.805
Yvr = -0.845
Nv = 0.0
Nvv = -0.209
Nvvv = 0.0
Nr = -0.0168
Nrr = -0.0115
Nrrr = -0.000358
Nrv = 0.08
Nvr = 0.08

#Loaction of thrusters w.r.t center of 
Lx = np.array([1.0678, 0.9344, 0.9344, -1.1644, -0.9911, -0.9911])
Ly = np.array([0.0, 0.11, -0.11, 0.0, -0.1644, 0.1644])
K = np.diag([0.3763, 0.3901, 0.3776, 0.5641, 0.4799, 0.5588]) # Thrust koefficient

propellerDiameter = 0.03    #[m]
n_dot_max = 5.0             #[1/s^2]
alpha_dot_max = 2.0         #[1/s]
thrust_max = 1.5            #[N]