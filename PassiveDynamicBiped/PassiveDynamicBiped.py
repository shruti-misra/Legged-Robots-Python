#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat May  6 15:55:41 2017

@author: Shruti
"""

from modelClass import Models
import numpy as np
import pylab as plt
from numpy import linalg as la
import matplotlib 
from matplotlib import rc
from MassMatrixWrapper import MassWrapper
from F_CoriGravWrapper import F_CoriGravWrapper
from ContactDynamicsWrapper import ContactDynamicsWrapper
from scipy.integrate import ode

#Note: All the definitions and initial values are taken from Remy's MATLAB Code
class PassiveDynamicBiped(Models):
    
  #Intiial conditions and definitions for the Continuous States  
  def ContStateDef(self):
      
      gamma   = + 0.3  # [rad] angle of the stance leg wrt the ground (0 = straight up)
      dgamma  = - 0.5  # [rad/sqrt(l_0/g)] ... angular velocity thereof
      alpha   = - 0.6  # [rad] inter leg angle
      dalpha  = + 0.5
      
      ContStates = np.asarray([gamma, dgamma, alpha, dalpha])
      return ContStates
      
  #Initial conditions for the discrete states          
  def DiscStateDef(self):
      
      #No discrete states
      DiscStates = np.array([])
      return DiscStates 
  
  #System parameter definitions     
  def SysParamDef(self):
      
    delta = 1*np.pi/180;
    gx        = +1*np.sin(delta) # [g] gravity in horizontal direction
    gy        = -1*np.cos(delta) #% [g] gravity in vertical direction
    # Parameter of the model
    l_0       = 1     # [l_0] leg length
    m1        = 0.20  # [m_0] mass of the main body
    m2        = 0.40  # [m_0] mass of the legs
    l2x       = 0.5   # [l_0] distance between hip joint and CoG of the legs (along the legs)
    l2y        = 0     # [l_0] distance between hip joint and CoG of the legs (perpendicular to the legs)
    rFoot     = 0.5   # [l_0] foot radius
    j2        = 0.002     # [m_0*l_0^2] inertia of the legs
    
    SysParams = np.asarray([gx,gy, l_0, m1, m2, l2x, l2y, rFoot, j2])
    return SysParams
  
    
  def FlowMap(self, y, p):

      ContStates = self.ContStateDef()
      if (y.size == 0):
          dydt_gamma = ContState[1]
          dydt_alpha = ContState[3]
      else:
          dydt_gamma = y[1]
          dydt_alpha = y[3]

      if (p.size == 0):
          p = self.SysParamDef()
 
      M = MassWrapper(y,p)
      f_cg = F_CoriGravWrapper(y,p)
      dd_q = la.solve(M,f_cg)      
      dydt_dgamma = dd_q[0]
      dydt_dalpha = dd_q[1]
      dydt = np.asarray([dydt_gamma, dydt_dgamma, dydt_alpha, dydt_dalpha])
      return dydt
      
  def JumpMap(self, yMINUS, zMINUS,p):
      
      ContStates = self.ContStateDef()
      SysParams = self.SysParamDef()
      if (yMINUS.size == 0):
          yMINUS = ContStates
      
      if(p.size == 0):
          p = SysParams
      
      #The (empty) discrete state vector remains untouched:
      zPLUS = zMINUS
      #The role of swing and stance leg switches, so we can directly set the
      #new angles:
          
      plus_gamma = yMINUS[2] + yMINUS[0]
      plus_alpha = -yMINUS[2]
      yPLUS   = np.asarray([plus_gamma, yMINUS[2], plus_alpha, yMINUS[3]])  
      dqMINUS = np.zeros((4,1))
      dqMINUS[0] = -(p[7]+ (np.cos(yMINUS[0])*(p[2] - p[7])))*yMINUS[1]
      dqMINUS[1] = -(np.sin(yMINUS[0])*(p[2] - p[7]))*yMINUS[1]
      dqMINUS[2] = yMINUS[3] + yMINUS[1]
      dqMINUS[3] = -yMINUS[3]
      [contHeight, M_cont, J_cont] = ContactDynamicsWrapper(yPLUS, p)
      ex11 = np.dot(J_cont, la.solve(M_cont, np.transpose(J_cont)))
      ex22 = np.dot(np.transpose(J_cont), la.solve(ex11, J_cont))
      ex33 = np.identity(4) - la.solve(M_cont, ex22)
      dqPLUS = np.dot(ex33, dqMINUS)
      yPLUS   = np.asarray([plus_gamma, dqPLUS[2], plus_alpha, dqPLUS[3]])
      yPLUS = np.transpose(yPLUS)     

      return  yPLUS, zPLUS
      
  def JumpSet(self,y,p):
   
    # Get a mapping for the state vector.
    contState = self.ContStateDef()
    if (y.size == 0):
          y = contState
    if (p.size == 0):
        p = self.SysParamDef()        
    #Event 1: Detect touchdown (this is also the final event)
    n_events = 1;
    evntVal = np.zeros((n_events,1));


    [contHeight, M_cont, J_cont] = ContactDynamicsWrapper(y, p)
    # Event 1: Detect touchdown 
    if (y[2] > 0.5): #(only if swing leg is in front of stance leg)
    
        #This has been slightly altered from Remy's code, the sign for contHeight has been flipped
        #This is because, we want to detect touchdown, so we want to detect when the height becomes
        #zero or slightly negative
        
        evntVal[0] = contHeight + 1e-12
    else:
        evntVal[0] = 1

    #Returns a 1 when an event is detected
    return evntVal[0]

    
pdb = PassiveDynamicBiped()    

#******** TESTING THE FLOW MAP ***********#
#p = pdb.SysParamDef()
#y0 = pdb.ContStateDef()
#y = [y0]
#t_ = 0
#dt = 0.01
#t_max = 5
#t = [t_]
#
#while(t_*dt < t_max):
#    t__ = int(t_*dt)
#    t.append((t_+ 1)*dt)
#    y_next = y[-1]  + dt*pdb.FlowMap(y[-1], p)
#    y.append(y_next)
#    t_ += 1
#        
#plt.plot(y)    

#******** WAS ATTEMPTING TO USE ODE, NEED TO FIX ***********#

#solver = ode(pdb.FlowMap).set_integrator('dopri5')
#solver.set_initial_value(y0, t_)
#while t < t_max:
#    y = solver.integrate(t+dt)
#    print(y)
#    t += dt     
#y0 = np.asarray([-0.07962203, -0.21755956,  0.3276293,  0.99076629])
#y0 = np.asarray([-0.0805,-0.2168,0.013,0.9884])
#
#
#[contHeight, M_cont, J_cont] = ContactDynamicsWrapper(y0, p)
#dx1 = pdb.FlowMap(y0, p)* dt
#dx2 = pdb.FlowMap(y0 + .5*dx1, p) * dt
#dx3 = pdb.FlowMap(y0 + .5*dx2, p) * dt
#dx4 = pdb.FlowMap(y0 + dx3, p) * dt
#dx = (1./6.)*( dx1 + 2*dx2 + 2*dx3 + dx4 )/dt
#x = y0 + dt * dx
#print(contHeight)
      
      
      
    
      
    
    