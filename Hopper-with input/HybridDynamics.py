#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Jun  5 10:32:49 2017

@author: Shruti
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon May  8 22:30:16 2017

@author: Shruti
"""
import numpy as np
from numpy import linalg as la
from Hopper import Hopper
import pylab as plt
import scipy
import sys
from copy import deepcopy
import math



def obs(trjs, insert_nans, only_disc, include, p):
    """
    observations
    (k,t),(j,o) = obs(trjs, [insert_nans, only_disc, include, params])
    inputs:
      trjs - struct array - trajectories
    
    optional inputs:
      insert_nans - bool - insert np.nan at discrete transitions ?
      only_disc - bool - only include discrete transitions ?
      include - list - discrete modes to include
    outputs:
      k - Nt list - observation indices
      t - 1  x Nt - observation times
      j - Nt list - discrete states
      o - Nt x No - observations of continuous states
    """
    K = []; T = []; J = []; O = []; U = []

    for trj in trjs:
      if include is None or trj.j in include:
        k = trj['k']; j = trj['j']
        if only_disc: # only include discrete transitions
          # initial trj state
          (t,x, u) = (trj['t'][0],trj['x'][0], trj['u'][0])
          K.append( k )
          T.append( t )
          J.append( j )
          O.append(x)
          U.append(u)
          # final trj state
          (t,x, u) = (trj['t'][-1],trj['x'][-1], trj['u'][-1])
          K.append( k )
          T.append( t )
          J.append( j )
          O.append(x)
          U.append(u)
        else:
          for (t,x,u) in zip(trj['t'],trj['x'], trj['u']):
            K.append( k )
            T.append( t )
            J.append( j )
            O.append(x)
            U.append(u)
          if insert_nans:
            K.append( K[-1] )
            T.append( T[-1] )
            J.append( J[-1] )
            O.append(x)
            U.append(u)

    return K,np.asarray(T),J, O, U
    

#This is the same as Sam's Hybrid class sim function, just modified
#a bit to accomodate the system
    
#def HybridDynamics(yIN, zIN, p, K, T, rx,dt, calc_cost):
def HybridDynamics(yIN, zIN, u0, p, K, tIN, tMAX, rx,dt):
    
    hop = Hopper()
    dt0 = dt 
    eventval = []
    
    k = 0
    t = [tIN]
    j = deepcopy(int(zIN))
    disc1 = [deepcopy(zIN)]
    x = [deepcopy(yIN)]
    u = [u0]
     
    trj = dict(k=k,t=t,j=j,x=x, u=u)
    trjs = []
   

      

    
    cost = 0
    lower_limit = 0
    upper_limit = 10

    while ( trj['t'][-1] <= tMAX # don't exceed max continuous time
            and trj['k'] < K): # don't allow discrete state is None
    
      k0 = trj['k']
      t0 = trj['t'][-1]
      j0 = trj['j']
      x0 = trj['x'][-1]
      u0 = trj['u'][-1]
      disc1.append(deepcopy(j0))
      #u = U[int(t0*(1/dt))]

          
      if 0: # forward Euler
        dx = hop.FlowMap(yIN, p,j0)
      else: # 4th-order Runge-Kutta
        dx1 = hop.FlowMap(x0, p,j0, u0)* dt
        dx2 = hop.FlowMap(x0 + .5*dx1, p,j0,u0) * dt
        dx3 = hop.FlowMap(x0 + .5*dx2, p,j0,u0) * dt
        dx4 = hop.FlowMap(x0 + dx3, p,j0,u0) * dt
        dx = (1./6.)*( dx1 + 2*dx2 + 2*dx3 + dx4 )/dt
        du1 = hop.FlowInput(u0, j0)* dt
        du2 = hop.FlowInput(u0 + .5*du1, j0) * dt
        du3 = hop.FlowInput(u0 + .5*du2, j0) * dt
        du4 = hop.FlowInput(u0 + du3, j0) * dt
        du = (1./6.)*( du1 + 2*du2 + 2*du3 + du4 )/dt
        

      k = k0
      j = j0
      t = t0 + dt
      x = x0 + dt * dx
      u = u0 + dt * du
      g = hop.JumpSet(x,p,j,u)
      eventval.append(g)
      # halve step size until trajectory doesn't violate guard more than rx
      i = 0
      imax = 50
      while np.any(g < -rx) and (i <= imax):
        #print(g)
        dt  = dt/2.
        t  = t0 + dt
        x = x0 + dt * dx
        g = hop.JumpSet(x,p,j,u)
        eventval.append(g)
        i += 1

      if (i >= imax):
        raise RuntimeError,'(sim)  guard iterations exceeded -- you probably have a buggy guard'

      # append state to trj
      trj['t'].append(t)
      trj['x'].append(x)
      trj['u'].append(u)

      # if in guard
          
      if np.any(g < 0) : 
        # spend time in guard
        if i >= imax:
          t = t + rx
        else:
          t = t + (rx + g.min())
        trj['t'].append(t)
        trj['x'].append(x)
        trj['u'].append(u)

        # append trj to trjs
        trjs.append(trj)
        
        #Compute event that has been triggered by checking which index of g 
        #is negative
        index = 0
        for i in range(g.size):
            if g[i] < 0:
                index = i
                
        # apply reset to modify trj
        yPlus, zPlus = hop.JumpMap(x,j,p,index+1,u)
      
        trj = dict(k=k,t=[t],j=int(zPlus),x=[yPlus], u = [u])
        trj['k'] += 1
        # re-initialize step size
        dt = dt0

    trjs.append(trj)

    return trjs, eventval

def sim(T, yIN, p,z):
    """
    numerical simulation
    trjs = sim(T, x0, [params])
    inputs:
      T   - scalar - maximum simulation time
      x0  - object - initial discrete state
    optional inputs:
      (all named arguments will be passed to methods as params dict)
    outputs:
      trj - trajectory dict
        .t - list - continuous times
        .x - list - continuous states
    """
    t = [0.]
    x = [deepcopy(yIN)]


    while ( t[-1] < T): # don't allow np.nan in state
    
      t0 = t[-1]
      x0 = x[-1]
      dt = 1e-2
      if 0: # forward Euler
        dx = hop.FlowMap(yIN, p,z)
      else: # 4th-order Runge-Kutta 
        dx1 = hop.FlowMap(x0, p,z)* dt
        dx2 = hop.FlowMap(x0 + .5*dx1, p,z) * dt
        dx3 = hop.FlowMap(x0 + .5*dx2, p,z) * dt
        dx4 = hop.FlowMap(x0 + dx3, p,z) * dt
        dx = (1./6.)*( dx1 + 2*dx2 + 2*dx3 + dx4 )/dt
      t1 = t0 + dt
      x1 = x0 + dt * dx
      t.append(t1)
      x.append(x1)

    return t,x

        


        
        