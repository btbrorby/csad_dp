#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: B. T. Brørby
# Created Date: 2022-02-18
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
# Tested:  <date> <developer>
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------

from imghdr import what
from json.tool import main
from fileConfig import loadmat
import numpy as np
import pickle
import os

file_name_ABC = "include/CSAD_modelABC.mat"
file_name = "include/CSAD_model.mat"

path = os.path.dirname(os.getcwd())
vessel = loadmat(r"{0}/csad_dp_ws/src/simulator/include/CSAD_model.mat".format(path))['vessel']
vesselABC = loadmat(r"{0}/csad_dp_ws/src/simulator/include/CSAD_modelABC.mat".format(path))['vesselABC']

name = vessel['main']['name']

Lpp = vessel['main']['Lpp']
T = vessel['main']['T']
B = vessel['main']['B']
CG = vessel['main']['CG']
m = vessel['main']['m']

k44 = vessel['main']['k44']
k55 = vessel['main']['k55']
k66 = vessel['main']['k66']

nabla = vessel['main']['nabla']

GM_L = vessel['main']['GM_L']
GM_T = vessel['main']['GM_T']

C_B = vessel['main']['C_B']
CB = vessel['main']['CB']
Lwl = vessel['main']['Lwl']
S = vessel['main']['S']

#What is correct?
MRB = vessel['MRB']
A = vessel['A']
B = vessel['B']
C = vessel['C']


rho = vessel['main']['g']
g = 9.81

Lx = np.array([1.0678, 0.9344, 0.9344, -1.1644, -0.9911, -0.9911])
Ly = np.array([0.0, 0.11, -0.11, 0.0, -0.1644, 0.1644])
K = np.array([0.3763, 0.3901, 0.3776, 0.5641, 0.4799, 0.5588]) # Thrust koefficient

propellerDiameter = 0.03    #[m]
n_dot_max = 5               #[1/s^2]
alpha_dot_max = 2           #[1/s]
thrust_max = 1.5              #[N]


# if __name__ == '__main__':
#     # print(out['vessel']['main']['g'])
#     print('hei')