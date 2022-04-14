#!/usr/bin/env python
import numpy as np
from cmath import pi
import math_tools
from matplotlib import pyplot as plt
       

f1 = []
f2 = []
f3 = []
f = []
f_noise = []
time = []
freq1 = 0
freq2 = 2*pi/30
freq3 = 2*pi/5
t = 0
dt = 0.001
while t < 100:
    time.append(t)
    f1.append(np.cos(freq1*t))
    f2.append(np.cos(freq2*t))
    f3.append(np.cos(freq3*t))
    f.append(np.cos(freq1*t)+np.cos(freq2*t)+np.cos(freq3*t))
    f_noise.append(np.cos(freq1*t)+np.cos(freq2*t)+np.cos(freq3*t))# + np.random.normal(0.0, 0.1))
    t += dt
    
n = len(time)
fhat = np.fft.fft(f, n)
PSD = fhat*np.conj(fhat)/n
F = (2*pi/(dt*n))*np.arange(n)
L = np.arange(1,np.floor(n/2), dtype='int')

indice = PSD > 100
PSDclean = indice*PSD
ffilt = indice*fhat
rec = np.fft.ifft(ffilt)

fig,axs = plt.subplots(3,1)
plt.sca(axs[0])
plt.plot(time, f_noise)
plt.plot(time, f, LineWidth=1.5)
plt.grid()

plt.sca(axs[1])
plt.plot(F[L], PSD[L])
plt.plot(F[L], PSDclean[L], LineWidth=1.5)

plt.sca(axs[2])
plt.plot(time, f)
plt.plot(time, rec, LineWidth=1.5)

plt.show()

