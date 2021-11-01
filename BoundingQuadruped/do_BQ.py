#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  7 05:50:31 2017

@author: Shruti
"""

import numpy as np
from numpy import linalg as la
from BoundingQuadruped import BoundingQuadruped
from HybridDynamics import HybridDynamics, obs
import pylab as plt
import scipy
import sys
from copy import deepcopy
import math



bq = BoundingQuadruped()
p = bq.SysParamDef()
y0 = bq.ContStateDef()
z0 = bq.DiscStateDef()
dt  = 1e-2
rx = 1e-6
K = 5
T = 5
strideFreq, sinalphaF, cosalphaF, sinlF, coslF, sinalphaB, cosalphaB, sinlB, coslB = bq.ExctParamDef()



trjs = HybridDynamics(y0, z0, p, K, T, rx,dt)

x1 = []
x2 = []
x3 = []
x4 = []
x5 = []
x6 = []
x7 = []
x8 = []


k = []; t = []; j = []; x = []
t1 = []; O = []
k,t,j,x_d = obs(trjs,True, False, None, p)

for x in x_d:
    x1.append(x[0])
    x2.append(x[2])
    x3.append(x[4])
    x4.append(x[6])
    x5.append(x[8])
    x6.append(x[10])
    x7.append(x[12])
    x8.append(x[14])
    
    

plt.figure(1)
plt.plot(t,x1)
plt.plot(t,x2)
plt.plot(t,x3)
plt.legend(labels=['x', 'y', 'phi'])
plt.xlabel('Time (s)')
plt.ylabel('States')
plt.title('States in the non-periodic motion of a Bounding Quaduped')

plt.figure(2)
plt.plot(t,x4)
plt.plot(t,x5)
plt.plot(t,x6)
plt.legend(labels=['alphaF', 'lF', 'alphaB'])
plt.xlabel('Time (s)')
plt.ylabel('States')
plt.title('States in the non-periodic motion of a Bounding Quaduped')

plt.figure(3)
plt.plot(t,x7)
plt.plot(t,x8)
plt.legend(labels=['lB', 'time'])
plt.xlabel('Time (s)')
plt.ylabel('States')
plt.title('States in the non-periodic motion of a Bounding Quaduped')
