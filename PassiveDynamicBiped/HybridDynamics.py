#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon May  8 22:30:16 2017

@author: Shruti
"""
import numpy as np
from numpy import linalg as la
from PassiveDynamicBiped import PassiveDynamicBiped
import pylab as plt
import scipy
import sys
from copy import deepcopy
import math


#Taken from Sam's Hybrid System library, but is modified a little to simplify     
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
    K = []; T = []; J = []; O = []

    for trj in trjs:
      if include is None or trj.j in include:
        k = trj['k']; j = trj['j']
        if only_disc: # only include discrete transitions
          # initial trj state
          (t,x) = (trj['t'][0],trj['x'][0])
          K.append( k )
          T.append( t )
          J.append( j )
          O.append(x)
          # final trj state
          (t,x) = (trj['t'][-1],trj['x'][-1])
          K.append( k )
          T.append( t )
          J.append( j )
          O.append(x)
        else:
          for (t,x) in zip(trj['t'],trj['x']):
            K.append( k )
            T.append( t )
            J.append( j )
            O.append(x)
          if insert_nans:
            K.append( K[-1] )
            T.append( T[-1] )
            J.append( J[-1] )
            O.append(x)

    return K,np.asarray(T),J, O
    

#Same as Sam's Hybrid System library sim function     
def HybridDynamics(yIN, zIN, p, K, T, rx,dt):
#def HybridDynamics(yIN, zIN, p, tIN, tMAX, rx,dt):
    
    pdb = PassiveDynamicBiped()
    dt0 = dt 

    k = 0
    t = [0.]
    j = deepcopy(zIN)
    x = [deepcopy(yIN)]

    trj = dict(k=k,t=t,j=j,x=x)
    trjs = []
    mm = 0

    while ( trj['t'][-1] <= T # don't exceed max continuous time
            and trj['k'] < K): # don't allow discrete state is None
    
      k0 = trj['k']
      t0 = trj['t'][-1]
      j0 = trj['j']
      x0 = trj['x'][-1]

      if 0: # forward Euler
        dx = pdb.FlowMap(yIN, p)
      else: # 4th-order Runge-Kutta 
        dx1 = pdb.FlowMap(x0, p)* dt
        dx2 = pdb.FlowMap(x0 + .5*dx1, p) * dt
        dx3 = pdb.FlowMap(x0 + .5*dx2, p) * dt
        dx4 = pdb.FlowMap(x0 + dx3, p) * dt
        dx = (1./6.)*( dx1 + 2*dx2 + 2*dx3 + dx4 )/dt

      k = k0
      j = j0
      t = t0 + dt
      x = x0 + dt * dx
      g = pdb.JumpSet(x,p)
      # halve step size until trajectory doesn't violate guard more than rx
      i = 0
      imax = 50
      while np.any(g < -rx) and (i <= imax):
        dt  = dt/2.
        t  = t0 + dt
        x = x0 + dt * dx
        g = pdb.JumpSet(x,p)
        i += 1

      if (i >= imax):
        raise RuntimeError,'(sim)  guard iterations exceeded -- you probably have a buggy guard'

      # append state to trj
      trj['t'].append(t)
      trj['x'].append(x)
      
      # if in guard    
      if np.any(g < 0) : 
        # spend time in guard
        if i >= imax:
          t = t + rx
        else:
          t = t + (rx + g.min())
        trj['t'].append(t)
        trj['x'].append(x)


        # append trj to trjs
        trjs.append(trj)

        # apply reset to modify trj
        yPlus, zPlus = pdb.JumpMap(x,j,p)
        trj = dict(k=k,t=[t],j=zPlus,x=[yPlus])
        #Increment the discrete time step as it is not automatically done in 
        #the PassiveDynamic class.
        trj['k'] += 1
        # re-initialize step size
        dt = dt0


    trjs.append(trj)

    return trjs

def sim(T, yIN, p):
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
        dx = pdb.FlowMap(yIN, p)
      else: # 4th-order Runge-Kutta 
        dx1 = pdb.FlowMap(x0, p)* dt
        dx2 = pdb.FlowMap(x0 + .5*dx1, p) * dt
        dx3 = pdb.FlowMap(x0 + .5*dx2, p) * dt
        dx4 = pdb.FlowMap(x0 + dx3, p) * dt
        dx = (1./6.)*( dx1 + 2*dx2 + 2*dx3 + dx4 )/dt
      t1 = t0 + dt
      x1 = x0 + dt * dx
      t.append(t1)
      x.append(x1)

    return t,x









        


        
        