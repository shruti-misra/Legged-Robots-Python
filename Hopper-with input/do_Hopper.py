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



    
hop = Hopper()
p = hop.SysParamDef()
y0 = hop.ContStateDef()
z0 = hop.DiscStateDef()
dt  = 1e-2
rx = 1e-6
K = 1
T = 3
#U = np.zeros(int(T*(1/dt)))    
    
    
trjs1, event1 = HybridDynamics(y0, z0, 0.4, p, K, 0.,T, rx,dt)
k1,t1,j1,x_d1, u1 = obs(trjs1,True, False, None, p)
y01 = np.asarray(x_d1[len(x_d1) - 1])
z01 = np.asarray(j1[len(j1) - 1])
t01 = np.asarray(t1[len(t1) - 1])
u01 = np.asarray(u1[len(t1) - 1])
    
trjs2, event2= HybridDynamics(y01, z01, u01, p, K, t01, T, rx,dt)
k2,t2,j2,x_d2, u2 = obs(trjs2,True, False, None, p)
y_end = x_d2[len(x_d2) - 1][1] 
wf = u2[len(u2) - 1]
    
J = (10 - y_end) + wf
print(J)
    
x1 = []
x2 = []
x3 = []
x4 = []
t = []


    
for x in x_d1:
    x1.append(x[0])
    x2.append(x[1])
    x3.append(x[2])
    x4.append(x[3])
 
for x in x_d2:
    x1.append(x[0])
    x2.append(x[1])
    x3.append(x[2])
    x4.append(x[3]) 
    
    
for time in t1:
    t.append(time)

 
for time in t2:
   t.append(time)
   
      
plt.figure(1)
plt.plot(t,x1)
plt.plot(t,x2)
plt.plot(t,x3)
plt.plot(t,x4)
plt.legend(labels=['x', 'y','dx', 'dy'])
plt.xlabel('Time (s)')
plt.ylabel('States')
    
#plt.figure(2)
#plt.plot(event)
#    
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
        
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 5))
ax.grid()
    
line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.01, 0.9, '', transform=ax.transAxes)
Time = T*1000        
        
def init():
    line.set_data([], [])
    time_text.set_text('')
    return line, time_text
    
    
def animate(i):
    thisx = [0, 0]
    thisy = [x1[i], x2[i]]

    line.set_data(thisx, thisy)
    time_text.set_text(time_template % (i*dt))
    return line, time_text
 
        
ani = animation.FuncAnimation(fig, animate, np.arange(1, Time),
        interval=25, blit=True, init_func=init)
    
    
plt.show()
print(x2[np.argmax(x2)])
