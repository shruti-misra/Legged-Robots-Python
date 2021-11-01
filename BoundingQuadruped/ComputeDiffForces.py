#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May 31 20:20:33 2017

@author: Shruti
"""
import numpy as np
import pylab as plt
from numpy import linalg as la
import matplotlib 
from matplotlib import rc
import math
from F_CoriGravW import F_CoriGravWrapper


def ComputeDifferentiableForces(y,u,p):
    
      
    # Compute the viscous damping coefficient of the spring, according to the desired
    # damping ratio:
    j_leg   = p[13] + ((p[1] - p[9])**2)*p[6] + p[12] + (p[8]**2)*p[5] # total leg inertia wrt the hip
    balpha  = p[15]*2*(math.sqrt(p[14]*j_leg)) 
    bl      = p[17]*2*(math.sqrt(p[16]*p[6])) 
    
#    # Compute spring and damping forces:
#    # Forces front leg:
    F_lF  = p[16]*(p[1]+ u[2] - y[8]) + bl*(u[3] - y[9])
    T_alphaF = p[14]*(p[2] + u[0]  - y[6]) + balpha*(u[1] - y[7])
    # Forces back leg:
    F_lB     = p[16]*(p[1] + u[6]  - y[12]) + bl*(u[7] - y[13])
    T_alphaB = p[14]*(p[3] + u[4] - y[10]) + balpha*(u[5] - y[11])
#    
#     # Graviational and coriolis forces:
    f_act = np.asarray([[[0], [0], [0], [T_alphaF], [F_lF], [T_alphaB], [F_lB]]])
#    # Graviational and coriolis forces:
    f_cg = F_CoriGravWrapper(y,p)
#    # All differentiable forces:
    f_diff = f_cg + f_act
    
    return f_diff, F_lF, T_alphaF, F_lB, T_alphaB