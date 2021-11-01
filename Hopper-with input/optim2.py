#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 25 20:04:01 2018

@author: Shruti
"""

from __future__ import division
import numpy as np
from numpy import linalg as la
from Hopper import Hopper
from HybridDynamics import HybridDynamics, obs
import pylab as plt
import scipy
from scipy.optimize import minimize
import sympy as sy
import sys
from copy import deepcopy
import math
import matplotlib.animation as animation

hop = Hopper()
p = hop.SysParamDef()
y0 = hop.ContStateDef()
z0 = hop.DiscStateDef()
dt  = 1e-2
rx = 1e-6
K = 1
T = 3

m = p[0]  #Upper body mass
M = p[1]  #Lowe body mass
k = p[2] #Spring constant
b = p[3]  #Viscuous drag of lower mass
l = p[4]  #Nominal length of spring
a = p[5]  #When lower mass is in contact with the ground, stiffness
g = p[6]

A = np.asarray([[0, 0, 1, 0, 0, 0, 0, 0], 
                [0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, -k/M, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, -k/M],
                [0, 0, 0, 0, -1, 0, 0, 0],
                [0, 0, 0, 0, 0, -1, 0, 0]])

#Matrix exponential
eA = scipy.linalg.expm(A)
x0 = np.asarray([0, 1, 0, 0, 0, 2, 0, 1])


def solve(x, eA):
    
    xf = np.asarray([0, 3, 0, x[3], x[4], x[5], x[6], x[7]])
    return x - np.dot(eA, xf)


soln = solve(x0, eA)

y0 = soln
y = [y0]
 
for t in range(0, 150):

        ydot = np.dot(A, y0)*(t/100)
        y_next = y0 + ydot
        y.append(y_next)
        y0 = y_next
        
plt.plot(y)
    
    
    








