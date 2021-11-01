#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  7 05:50:31 2017

@author: Shruti
"""

import numpy as np
from numpy import linalg as la
from Hopper import Hopper
from HybridDynamics import HybridDynamics, obs
import pylab as plt
import scipy
import sys
from copy import deepcopy
import math
import matplotlib.animation as animation


#plt.figure(1)
#plt.plot(t,x1)
#plt.plot(t,x2)
#plt.plot(t,x3)
#plt.plot(t,x4)
#plt.legend(labels=['x', 'y','dx', 'dy'])
#plt.xlabel('Time (s)')
#plt.ylabel('States')
#
#plt.figure(2)
#plt.plot(event)
#
##plt.figure(2)
##plt.plot(t, x1)
##plt.plot(t, x2)
##plt.legend(labels=['x','y'])
##plt.xlabel('Time (s)')
##plt.ylabel('States')
##
##plt.figure(3)
##plt.plot(x1, x3)
##plt.xlabel('x')
##plt.ylabel('dx')
##
##plt.figure(4)
##plt.plot(x2, x4)
##plt.xlabel('y')
##plt.ylabel('dy')



def step(action):
    
    
    hop = Hopper()
    p = hop.SysParamDef(action)
    y0 = hop.ContStateDef()
    z0 = hop.DiscStateDef()
    dt  = 1e-2
    rx = 1e-6
    K = 3
    T = 3
    
    
    
    trjs, event = HybridDynamics(y0, z0, p, K, T, rx,dt, action)
    event = np.vstack(event)
    
    x1 = []
    x2 = []
    x3 = []
    x4 = []
    
    k = []; t = []; j = []; x = []
    t1 = []; O = []
    k,t,j,x_d = obs(trjs,True, False, None, p)
    
    for x in x_d:
        x1.append(x[0])
        x2.append(x[1])
        x3.append(x[2])
        x4.append(x[3])
    plt.figure(1)
    plt.plot(t,x1)
    plt.plot(t,x2)
    plt.plot(t,x3)
    plt.plot(t,x4)
    plt.legend(labels=['x', 'y','dx', 'dy'])
    plt.xlabel('Time (s)')
    plt.ylabel('States')
    
    plt.figure(2)
    plt.plot(event)
    
    #plt.figure(2)
    #plt.plot(t, x1)
    #plt.plot(t, x2)
    #plt.legend(labels=['x','y'])
    #plt.xlabel('Time (s)')
    #plt.ylabel('States')
    #
    #plt.figure(3)
    #plt.plot(x1, x3)
    #plt.xlabel('x')
    #plt.ylabel('dx')
    #
    #plt.figure(4)
    #plt.plot(x2, x4)
    #plt.xlabel('y')
    #plt.ylabel('dy')
        
#    fig = plt.figure()
#    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 5))
#    ax.grid()
#    
#    line, = ax.plot([], [], 'o-', lw=2)
#    time_template = 'time = %.1fs'
#    time_text = ax.text(0.01, 0.9, '', transform=ax.transAxes)
#    Time = T*1000        
#        
#    def init():
#        line.set_data([], [])
#        time_text.set_text('')
#        return line, time_text
#    
#    
#    def animate(i):
#        thisx = [0, 0]
#        thisy = [x1[i], x2[i]]
#    
#        line.set_data(thisx, thisy)
#        time_text.set_text(time_template % (i*dt))
#        return line, time_text
#    
##    plt.figure(1)
##    plt.plot(t,x1)
##    plt.plot(t,x2)
##    plt.plot(t,x3)
##    plt.plot(t,x4)
##    plt.legend(labels=['x', 'y','dx', 'dy'])
##    plt.xlabel('Time (s)')
##    plt.ylabel('States')   
#        
#    ani = animation.FuncAnimation(fig, animate, np.arange(1, Time),
#        interval=15, blit=True, init_func=init)
#    
#    
#    plt.show()
    return x2[np.argmax(x2)]
    
    
reward = step(10)
print reward