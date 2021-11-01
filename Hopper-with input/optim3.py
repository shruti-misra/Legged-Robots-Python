#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 27 17:38:17 2018

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
from scipy.optimize import fmin

hop = Hopper()
p = hop.SysParamDef()
y0 = hop.ContStateDef()
z0 = hop.DiscStateDef()
u0 = 0.8


def f(x):
    
    p = hop.SysParamDef()
    y_bar = 10
    #unpack 
    y = x[0:4]
    z = x[4]
    z = 2
    
    print(z)
        
    u = x[5]
    dt  = 1e-2
    rx = 1e-6
    K = 1
    T = 3
    
    trjs1, event1 = HybridDynamics(y, z, 0.4, p, K, 0.,T, rx,dt)
    k1,t1,j1,x_d1, u1 = obs(trjs1,True, False, None, p)
    y01 = np.asarray(x_d1[len(x_d1) - 1])
    z01 = np.asarray(j1[len(j1) - 1])
    t01 = np.asarray(t1[len(t1) - 1])
    u01 = np.asarray(u1[len(t1) - 1])
    

   
   
    trjs2, event2= HybridDynamics(y01, z01, u01, p, K, t01, T, rx,dt)
    k2,t2,j2,x_d2, u2 = obs(trjs2,True, False, None, p)
    
    y_end = x_d2[len(x_d2) - 1][1] 
    wf = u2[len(u2) - 1]
    
    J = (y_bar - y_end) + (j1[len(j1)-1] - j2[0]) + wf
    return J


    
x_init = np.append(y0, z0)
x_init = np.append(x_init, u0)    
xopt = fmin(f, x_init,xtol=1e-6, disp=True)

print "Results", xopt, "\n\n"
    
