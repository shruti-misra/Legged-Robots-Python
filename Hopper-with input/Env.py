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
    

#This is the same as Sam's Hybrid class sim function, just modified
#a bit to accomodate the system
    
def HybridDynamics(yIN, zIN, p, K, T, rx,dt, action):
#def HybridDynamics(yIN, zIN, p, tIN, tMAX, rx,dt):
    
    hop = Hopper()
    dt0 = dt 
    eventval = []
    
    k = 0
    t = [0.]
    j = deepcopy(zIN)
    disc1 = [deepcopy(zIN)]
    x = [deepcopy(yIN)]

    trj = dict(k=k,t=t,j=j,x=x)
    trjs = []

    while ( trj['t'][-1] <= T # don't exceed max continuous time
            and trj['k'] < K): # don't allow discrete state is None
    
      k0 = trj['k']
      t0 = trj['t'][-1]
      j0 = trj['j']
      print('mode:' , j0)
      x0 = trj['x'][-1]
      disc1.append(deepcopy(j0))


      if 0: # forward Euler
        dx = hop.FlowMap(yIN, p,j, action)
      else: # 4th-order Runge-Kutta 
        dx1 = hop.FlowMap(x0, p,j0, action)* dt
        dx2 = hop.FlowMap(x0 + .5*dx1, p,j0, action) * dt
        dx3 = hop.FlowMap(x0 + .5*dx2, p,j0, action) * dt
        dx4 = hop.FlowMap(x0 + dx3, p,j0, action) * dt
        dx = (1./6.)*( dx1 + 2*dx2 + 2*dx3 + dx4 )/dt

      k = k0
      j = j0
      t = t0 + dt
      x = x0 + dt * dx
      g = hop.JumpSet(x,p,j, action)
      eventval.append(g)
      # halve step size until trajectory doesn't violate guard more than rx
      i = 0
      imax = 50
      while np.any(g < -rx) and (i <= imax):
        #print(g)
        dt  = dt/2.
        t  = t0 + dt
        x = x0 + dt * dx
        g = hop.JumpSet(x,p,j, action)
        eventval.append(g)
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
        
        #Compute event that has been triggered by checking which index of g 
        #is negative
        index = 0
        for i in range(g.size):
            if g[i] < 0:
                index = i
                
        # apply reset to modify trj
        yPlus, zPlus = hop.JumpMap(x,j,p,index+1, action)
        trj = dict(k=k,t=[t],j=zPlus,x=[yPlus])
        trj['k'] += 1
        print(index)
        print(zPlus)
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
        dx = hop.FlowMap(yIN, p,z, action)
      else: # 4th-order Runge-Kutta 
        dx1 = hop.FlowMap(x0, p,z, action)* dt
        dx2 = hop.FlowMap(x0 + .5*dx1, p,z, action) * dt
        dx3 = hop.FlowMap(x0 + .5*dx2, p,z, action) * dt
        dx4 = hop.FlowMap(x0 + dx3, p,z, action) * dt
        dx = (1./6.)*( dx1 + 2*dx2 + 2*dx3 + dx4 )/dt
      t1 = t0 + dt
      x1 = x0 + dt * dx
      t.append(t1)
      x.append(x1)

    return t,x


class Env(object):

  def __init__(self):

    from Hopper import Hopper

    #Define Hopper class in this space
    self.hop = Hopper()
    self.ConstStateDef = self.hop.ConstStateDef
    self.DiscStateDef = self.hop.DiscStateDef
    self.SysParamDef = self.hop.SysParamDef
    self.FlowMap = self.hop.FlowMap
    self.JumpSet = self.hop.JumpSet
    self.JumpMap = self.hop.JumpMap

    #Set dt value for one Runge Kutta Step
    self.dt = 10e-3

    #Initialize States
    


  def step(self, action):

        dx1 = self.FlowMap(x0, p,j0, action)* self.dt
        dx2 = self.FlowMap(x0 + .5*dx1, p,j0, action) * self.dt
        dx3 = self.FlowMap(x0 + .5*dx2, p,j0, action) * self.dt
        dx4 = self.FlowMap(x0 + dx3, p,j0, action) * self.dt
        dx = (1./6.)*( dx1 + 2*dx2 + 2*dx3 + dx4 )/self.dt

    pass

  def reset(self):

    pass

        


        
        