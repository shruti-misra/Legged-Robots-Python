#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  7 05:05:20 2017

@author: Shruti
"""

import numpy as np
from numpy import linalg as la
from PassiveDynamicBiped import PassiveDynamicBiped
from HybridDynamics import HybridDynamics, obs
import pylab as plt
import scipy
import sys
from copy import deepcopy
import math

#Initialize simulation parameters
pdb = PassiveDynamicBiped()
p = pdb.SysParamDef()
y0 = pdb.ContStateDef()
z0 = pdb.DiscStateDef()
dt  = 1e-2
rx = 1e-2
K = 3
T = 5


#Simulate trajectories using the Hybrid Dynamics Function 
trjs = HybridDynamics(y0, z0, p, K, T, rx,dt)
k = []; t = []; j = []; x = []
#Extract trajectories
k,t,j,x_c = obs(trjs,True, False, None, p)

#*************PLOT FIGURES************

plt.figure(1)
plt.plot(t,x_c)
plt.legend(labels=['gamma', 'dgamma', 'alpha', 'dalpha'])

plt.xlabel('Time (s)')
plt.ylabel('States')
plt.title('All states in the non-periodic motion of a Passive Dynamic Biped')

x1 =[]
x2 = []

for x in x_c:
    x1.append(x[0])
    x2.append(x[2])


plt.figure(2)
plt.plot(t, x1)
plt.plot(t, x2)
plt.legend(labels=['gamma','alpha'])
plt.xlabel('Time (s)')
plt.ylabel('States')
plt.title('Relevant states in the non-periodic motion of a Passive Dynamic Biped')