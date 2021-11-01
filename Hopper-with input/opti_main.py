#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  7 05:50:31 2017

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
u0 = 0.8
dt  = 1e-2
rx = 1e-6
K = 1
T = 3



def f(x):
    
    p = hop.SysParamDef()
    y_bar = 10
    #unpack 
    y = x[0:4]
    z = x[4]
    u = x[5]
    
    
    #Run first hybrid 
    trjs1, event1 = HybridDynamics(y, z, u, p, K, 0.,T, rx,dt)
    k1,t1,j1,x_d1, u1 = obs(trjs1,True, False, None, p)
#    y01 = np.asarray(x_d1[len(x_d1) - 1])
#    z01 = np.asarray(j1[len(j1) - 1])
#    t01 = np.asarray(t1[len(t1) - 1])
#    u01 = np.asarray(u1[len(t1) - 1])
#    
#    trjs2, event2 = HybridDynamics(y01, z01, u01, p, K, t01, T, rx,dt)
#    k2,t2,j2,x_d2, u2 = obs(trjs2,True, False, None, p)
    
    cost = np.sum(0.5*np.square(u1))

    
    return cost


def eq_constraint1(x):
    
    p = hop.SysParamDef()
    
    #unpack 
    y = x[0:4]
    z = x[4]
    u = x[5]
    
    
    trjs1, event1 = HybridDynamics(y, z, u, p, K, 0.,T, rx,dt)
    k1,t1,j1,x_d1, u1 = obs(trjs1,True, False, None, p)
#    y01 = np.asarray(x_d1[len(x_d1) - 1])
#    z01 = np.asarray(j1[len(j1) - 1])
#    t01 = np.asarray(t1[len(t1) - 1])
#    u01 = np.asarray(u1[len(t1) - 1])
#    
#    trjs2, event2 = HybridDynamics(y01, z01, u01, p, K, t01, T, rx,dt)
#    k2,t2,j2,x_d2, u2 = obs(trjs2,True, False, None, p)
    
    c2 = 2 - x_d1[len(x_d1) - 1][1]
    return c2

def eq_constraint2(x):
    
    p = hop.SysParamDef()
    
    #unpack 
    y = x[0:4]
    z = x[4]
    u = x[5]
    
    
    trjs1, event1 = HybridDynamics(y, z, u, p, K, 0.,T, rx,dt)
    k1,t1,j1,x_d1, u1 = obs(trjs1,True, False, None, p)
#    y01 = np.asarray(x_d1[len(x_d1) - 1])
#    z01 = np.asarray(j1[len(j1) - 1])
#    t01 = np.asarray(t1[len(t1) - 1])
#    u01 = np.asarray(u1[len(t1) - 1])
#    
#    trjs2, event2 = HybridDynamics(y01, z01, u01, p, K, t01, T, rx,dt)
#    k2,t2,j2,x_d2, u2 = obs(trjs2,True, False, None, p)


        
    c4 = x_d1[len(x_d1) - 1][0]

    return c4




#Pack everything into one variable 
x_init = np.append(y0, z0)
x_init = np.append(x_init, u0)

#con = ({'type': 'eq', 'fun': eq_constraint1, 'jac': np.eye(4)},
#       {'type': 'eq', 'fun': eq_constraint2, 'jac': np.eye(1)})
con = ({'type': 'eq', 'fun': eq_constraint1}, {'type': 'eq', 'fun': eq_constraint2})
res = minimize(f, x_init, method='SLSQP', constraints=con, options={'disp': True})
#x = res.x
#print(res.fun)