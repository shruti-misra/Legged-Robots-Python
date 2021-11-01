#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Dec 13 11:06:07 2017

@author: Shruti
"""
from __future__ import division
from modelClass import Models
import numpy as np
import pylab as plt
from numpy import linalg as la
import matplotlib 
from matplotlib import rc
from scipy.integrate import ode


class Hopper(Models):
    
    def ContStateDef(self):
        
        #Initialize in aerial mode?
        x = 0
        y = 1.5
        dx = 0
        dy = 0
        
        ContStates = np.asarray([x, y, dx, dy])
        return ContStates
        
    def DiscStateDef(self):
        
        phase = 2
        DiscStates = np.array([phase])
        return DiscStates 
    
    def SysParamDef(self, action):
        
        m = 1  #Lower body mass
        u = 3  #Upper body mass
        k = 10 #Spring constant
        b = 5  #Viscuous drag of lower mass
        l = 2  #Nominal length of spring
        a = action #When lower mass is in contact with the ground, stiffness
        g = 2
        
        SysParams = np.asarray([m, u, k, b, l, a, g])
        return SysParams
        
    def FlowMap(self, Y, p, z, action):
        
      ContStates = self.ContStateDef()
      DiscStates = self.DiscStateDef()
      SysParams = self.SysParamDef(action)
        
      if (Y.size == 0): 
          s = ContStates
      else:
          
          s = Y
      
      if (p.size == 0):
          p = SysParams
    
      if (z.size == 0):
          z = DiscStates
       
      m, u, k, b, l, a, g = unpack_param(p) 
      
      x = s[0]
      y = s[1]
      dx = s[2]
      dy = s[3]
            
      #Before touch down
      if (z == 1):
          
          ddx = (1/m)*(-k*(l-(y-x)) - b*dx - m*g)
          ddy = (1/u)*(k*(l - (y-x)) - u*g)
          
          dydt = np.asarray([dx, dy, ddx, ddy])
          print('Air dynamics')
          
      #Liftoff    
      elif (z == 2):    
        
          ddx = 0
          ddy = (1/u)*(a*k*(l - y) - u*g)
          print(ddy)

          
          dydt = np.asarray([0, dy, ddx, ddy])
          print('Ground dynamics')
          
#      else:
#          
#          dydt = dydt = np.asarray([0, 0, 0, 0])
          
      return dydt    
          
                        
    def JumpMap(self, yMINUS, zMINUS,p, event, action):
        
      ContStates = self.ContStateDef()
      DiscStates = self.DiscStateDef()
      SysParams = self.SysParamDef(action)
        
      if (yMINUS.size == 0): 
          s = ContStates
          
      else:
          
          s = yMINUS
    
      if (zMINUS.size == 0):
          z = DiscStates
          
      else:
          
          z = zMINUS
       
      m, u, k, b, l, a, g = unpack_param(SysParams) 
      
      x = s[0]
      y = s[1]
      dx = s[2]
      dy = s[3]
      
            #Air to ground Phase
      if (event == 1 and z == 1):
                    
          yPLUS = np.asarray([0, y, 0, dy])
          zPLUS = np.asarray([2])
          print('air to ground')
          
      #Ground to air phase    
      elif (event == 1 and z == 2):    
        
          
          yPLUS = np.asarray([0, y, 0, dy])
          zPLUS = np.asarray([1])
          print('ground to air')
          
      else:
          
          yPLUS = yMINUS
          zPLUS = zMINUS
          
      return yPLUS, zPLUS 
      
      
        
    def JumpSet(self, Y, p, z, action):
        
      ContStates = self.ContStateDef()
      DiscStates = self.DiscStateDef()
      SysParams = self.SysParamDef(action)
        
      if (Y.size == 0): 
          s = ContStates
      else:
          
          s = Y
      
      if (p.size == 0):
          p = SysParams
    
      if (z.size == 0):
          z = DiscStates
       
      m, u, k, b, l, a, g = unpack_param(p) 
        
      x = s[0]
      y = s[1]
      dx = s[2]
      dy = s[3]
      
      n_events = 1;
      evntVal = np.zeros((n_events,1));
        
      if (z == 1):
            
          evntVal[0] = x
        
      elif (z == 2): 
            
          evntVal[0] = k*(l-y) + m*g
            
            
      return evntVal

def unpack_param(p):
    
        m = p[0]  #Upper body mass
        u = p[1]  #Lowe body mass
        k = p[2] #Spring constant
        b = p[3]  #Viscuous drag of lower mass
        l = p[4]  #Nominal length of spring
        a = p[5]  #When lower mass is in contact with the ground, stiffness
        g = p[6]
        
        return m, u, k, b, l, a, g

    