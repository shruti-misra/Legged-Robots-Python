#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May 31 18:46:33 2017

@author: Shruti
"""

from modelClass import Models
import numpy as np
import pylab as plt
from numpy import linalg as la
import matplotlib 
from matplotlib import rc
from ComputeDiffForces import ComputeDifferentiableForces
from MassMatrixW import MassMatrixWrapper
from ContactKinWB import ContactKinematicsWB
from ContactKinWF import ContactKinematicsWF
from ExcitationFn import ExcitationFunction


#Note: All the definitions and initial values are taken from Remy's MATLAB Code
class BoundingQuadruped(Models):
    
  #Intiial conditions and definitions for the Continuous States  
  def ContStateDef(self):
      
    x       =  0.00       # [l_0] horizontal position of the main body CoG
    dx      =  0.70       # [sqrt(g*l_0)] ... velocity thereof
    y       =  1.15       # [l_0] vertical position of the main body CoG
    dy      =  0.00       # [sqrt(g*l_0)] ... velocity thereof
    phi     =  0.01       # [rad] angle of the main body wrt horizontal (pitch)
    dphi    =  0.15       # [rad/sqrt(l_0/g)] ... angular velocity thereof
    alphaF  = -0.32       # [rad] angle of the front leg wrt the main body (0 = straight down)
    dalphaF =  0.64       # [rad/sqrt(l_0/g)] ... angular velocity thereof
    lF      =  1.07       # [l_0] length of the front leg (hip-joint to foot-center)
    dlF     = -0.57       # [sqrt(g*l_0)] ... velocity thereof
    alphaB  =  0.01       # [rad] angle of the back leg wrt the main body (0 = straight down)
    dalphaB =  0.89       # [rad/sqrt(l_0/g)] ... angular velocity thereof
    lB      =  0.97       # [l_0] length of the back leg (hip-joint to foot-center)
    dlB     =  0.10       # [sqrt(g*l_0)] ... velocity thereof 
    time    =  0.00       # [sqrt(l_0/g)] time that has passed since the start of the step
    posWork =  0.00       # [m_0*g*l_0] positive mechanical work of the actuators
      
    ContStates = np.asarray([x, dx, y, dy, phi, dphi, alphaF, dalphaF, lF, dlF, alphaB, dalphaB, lB, dlB, time, posWork])
    return ContStates
      
  #Initial conditions for the discrete states          
  def DiscStateDef(self):
      
    phaseF = 2 # ['1','2'] The current phase of the front leg (stance = 1) (flight = 2)
    phaseB = 2 # ['1','2'] The current phase of the back leg (stance = 1) (flight = 2)
    COT    = 0 #S [m_0*g]     Cost of transportation (posWork/distance traveled)
    DiscStates = np.asarray([phaseF, phaseB, COT])
    return DiscStates
  
  #System parameter definitions     
  def SysParamDef(self):
      
    # Physics:
    g         = 1     # [g] gravity
    # Parameter of the model
    l_0       = 1     # [l_0] uncompressed leg length
    alphaF_0  = 0     # [rad] resting front leg angle
    alphaB_0  = 0     # [rad] resting back leg angle
    m1        = 0.70  # [m_0] mass of the main body
    m2        = 0.10  # [m_0] mass of the upper leg segments
    m3        = 0.05  # [m_0] mass of the lower leg segments
    l1        = 0.75  # [l_0] distance between CoG of the main body and hip joints
    l2        = 0.25  # [l_0] distance between hip joints and CoG of the upper leg segments
    l3        = 0.25  # [l_0] distance between foot points and CoG of the lower leg segments
    rFoot     = 0.05  # [l_0] foot radius
    j1        = 0.4   # [m_0*l_0^2] inertia of the main body
    j2        = 0.002 # [m_0*l_0^2] inertia of the upper leg segments
    j3        = 0.002 # [m_0*l_0^2] inertia of the lower leg segments
    kalpha    = 5     # [m_0*g*l_0/rad] rotational spring stiffness in the hip joints
    balphaRat = 0.2   # [*] damping ratio.  Use 20% of critical damping
    kl        = 10    # [m_0*g/l_0] linear spring stiffness in the prismatic joints
    blRat     = 0.2   # [*] damping ratio.  Use 20% of critical damping
    
    Params = np.asarray([g, l_0, alphaF_0, alphaB_0, m1, m2, m3, l1, l2, l3, rFoot, j1, j2, j3, kalpha, balphaRat,kl,blRat])
    return Params
    
  def ExctStateDef(self):
      
    ualphaF  = 0 # [rad] motor angle of the front rotational actuator 
    dualphaF = 0 # [rad/sqrt(l_0/g)] ... velocity thereof
    ulF      = 0 # [l_0] motor position of the front linear actuator 
    dulF     = 0 # [sqrt(g*l_0)] ... velocity thereof
    ualphaB  = 0 # [rad] motor angle of the back rotational actuator 
    dualphaB = 0 # [rad/sqrt(l_0/g)] ... velocity thereof
    ulB      = 0 # [l_0] motor position of the back linear actuator 
    dulB     = 0 # [sqrt(g*l_0)] ... velocity thereof
    
    ExctState = np.asarray([ualphaF, dualphaF, ulF, dulF, ualphaB, dualphaB, ulB, dulB])
    return ExctState
      
  def ExctParamDef(self):
      
    strideFreq = 0.5                   # [sqrt(g/l_0)] stride frequency. 
    sinalphaF  = np.asarray([ 0.12, 0.02,0,0,0,0,0,0,0,0]) # [rad] amplitude of the sine-terms for front leg rotation
    cosalphaF  = np.asarray([-0.27,-0.03,0,0,0,0,0,0,0,0]) # [rad] amplitude of the cosine-terms for front leg rotation
    sinlF      = np.asarray([-0.03,-0.03,0,0,0,0,0,0,0,0]) # [l_0] amplitude of the sine-terms for front leg extension
    coslF      = np.asarray([ 0.05, 0.02,0,0,0,0,0,0,0,0]) # [l_0] amplitude of the cosine-terms for front leg extension
    sinalphaB  = np.asarray([ 0.19, 0.04,0,0,0,0,0,0,0,0]) # [rad] amplitude of the sine-terms for back leg rotation
    cosalphaB  = np.asarray([ 0.10,-0.02,0,0,0,0,0,0,0,0]) # [rad] amplitude of the cosine-terms for back leg rotation
    sinlB      = np.asarray([-0.03, 0.01,0,0,0,0,0,0,0,0]) # [l_0] amplitude of the sine-terms for back leg extension
    coslB      = np.asarray([-0.01,-0.02,0,0,0,0,0,0,0,0]) # [l_0] amplitude of the cosine-terms for back leg extens
      #, cosalphaF, sinlF, coslF, sinalphaB, cosalphaB, sinlB, coslB]])
    return strideFreq, sinalphaF, cosalphaF, sinlF, coslF, sinalphaB, cosalphaB, sinlB, coslB
      
    
  def FlowMap(self, y, p, z):
      
      ContStates = self.ContStateDef()
      DiscStates = self.DiscStateDef()
      SysParams = self.SysParamDef()
      strideFreq, sinalphaF, cosalphaF, sinlF, coslF, sinalphaB, cosalphaB, sinlB, coslB = self.ExctParamDef()
     
      
      if (y.size == 0): 
          y = ContStates
      
      if (p.size == 0):
          p = SysParams
    
      if (z.size == 0):
          z = DiscStates
          
      dydt_x = y[1]
      dydt_y = y[3]
      dydt_phi = y[5]
      dydt_alphaF = y[7]
      dydt_lF = y[9]
      dydt_alphaB = y[11] 
      dydt_lB = y[13]             
      dydt_time = 1
      
      u = ExcitationFunction(y,strideFreq, sinalphaF, cosalphaF, sinlF, coslF, sinalphaB, cosalphaB, sinlB, coslB)
      
      f_diff, F_lF, T_alphaF, F_lB, T_alphaB = ComputeDifferentiableForces(y, u, p)
      dydt_posWork = max(0,F_lF*u[3]) + max(0,T_alphaF*u[1]) + max(0,F_lB*u[7]) + max(0,T_alphaB*u[5])
      M = MassMatrixWrapper(y,p)
      
      curr_phase = z[0] + 2*z[1]

      if (curr_phase == 3):
          posF, JF, dJFdtTIMESdqdt = ContactKinematicsWF(y,p)
          posB, JB, dJBdtTIMESdqdt = ContactKinematicsWB(y,p)
          J  = np.asarray([[JF], [JB]])
          J = np.reshape(J, (4,7), order = 'C')
          dJdtTIMESdqdt = np.asarray([[dJFdtTIMESdqdt], [dJBdtTIMESdqdt]])
          dJdtTIMESdqdt = np.reshape(dJdtTIMESdqdt, (4,1))
          imm0 = -np.dot(J, (la.solve(M,f_diff)))
          imm0 = np.reshape(imm0, dJdtTIMESdqdt.shape)
          imm1 = imm0 - dJdtTIMESdqdt
          imm1 = np.reshape(imm1, dJdtTIMESdqdt.shape)
          imm2 = np.dot(J,la.solve(M, np.transpose(J)))
          f_contX = la.solve(imm2, imm1)
          f_contQ = np.dot(np.transpose(J), f_contX)
          
      elif (curr_phase == 4):
          
          pos, J, dJdtTIMESdqdt = ContactKinematicsWB(y,p)
          imm0 = -np.dot(J, (la.solve(M,f_diff)))
          imm0 = np.reshape(imm0, dJdtTIMESdqdt.shape)
          imm1 = imm0 - dJdtTIMESdqdt
          imm2 = np.dot(J,la.solve(M, np.transpose(J)))
          f_contX = la.solve(imm2, imm1)
          f_contQ = np.dot(np.transpose(J), f_contX)

          
      elif (curr_phase == 5):
          
          pos, J, dJdtTIMESdqdt = ContactKinematicsWF(y,p)
          imm0 = -np.dot(J, (la.solve(M,f_diff)))
          imm0 = np.reshape(imm0, dJdtTIMESdqdt.shape)
          imm1 = imm0 - dJdtTIMESdqdt
          imm2 = np.dot(J,la.solve(M, np.transpose(J)))
          f_contX = la.solve(imm2, imm1)
          f_contQ = np.dot(np.transpose(J), f_contX)

    
      else:
          f_contQ = np.zeros(np.shape(f_diff))
          

      dd_q = la.solve(M, f_diff + f_contQ)
      dd_q = np.reshape(dd_q, (7,1))
      dydt_dx = dd_q[0]
      dydt_dy = dd_q[1]
      dydt_dphi = dd_q[2]
      dydt_dalphaF = dd_q[3]
      dydt_dlF = dd_q[4]
      dydt_dalphaB = dd_q[5] 
      dydt_dlB = dd_q[6]
    
      dydt = np.asarray([dydt_x,dydt_dx,dydt_y,dydt_dy,dydt_phi,dydt_dphi, dydt_alphaF, dydt_dalphaF, dydt_lF, dydt_dlF, dydt_alphaB, dydt_dalphaB, dydt_lB, dydt_dlB, dydt_time, dydt_posWork])
      return dydt
      
  def JumpMap(self, yMINUS, zMINUS, p, event):
    
    ContStates = self.ContStateDef()
    DiscStates = self.DiscStateDef()
    SysParams = self.SysParamDef()
    strideFreq, sinalphaF, cosalphaF, sinlF, coslF, sinalphaB, cosalphaB, sinlB, coslB = self.ExctParamDef()
    
      
    if (yMINUS.size == 0): 
        y = ContStates
      
    if (p.size == 0):
        p = SysParams
    
    if (zMINUS.size == 0):
        z = DiscStates
          
    u = ExcitationFunction(yMINUS,strideFreq, sinalphaF, cosalphaF, sinlF, coslF, sinalphaB, cosalphaB, sinlB, coslB)      
    yPLUS = yMINUS
    zPLUS = zMINUS
    isTerminal = False
    # Event 1: Detect touchdown front leg
    # Event 2: Detect liftoff front leg
    # Event 3: Detect touchdown back leg
    # Event 4: Detect liftoff back leg
    # Event 5: Detect stop (t == 1/strideFrequency for active systems,
    #          otherwise dy == 0 during flight with additional timing
    #          requirements)    
    
    # Event 1: detected touchdown front leg
    if (event == 1): 
        # At touchdown, the contact point comes to a complete rest, a
        # fully plastic collision is computed:
        # Compute mass matrix
        M = MassMatrixWrapper(yMINUS,p)

        # Contact Forces (with Jacobian)
        posF, JF, dJFdtTIMESdqdt = ContactKinematicsWF(yMINUS, p)

        if (zMINUS[1] == 1): 
            # Double-contact:
            posB, JB, dJBdtTIMESdqdt = ContactKinematicsWB(yMINUS, p)
            J  = np.asarray([[JF], [JB]])
            J = np.reshape(J, (4,7), order = 'C')

        else:
            # Single-contact:
            J = JF

        # Velocities before collision:
        dqMINUS = np.zeros((7,1))
        dqMINUS[0] = yMINUS[1]
        dqMINUS[1] = yMINUS[3]
        dqMINUS[2] = yMINUS[5]
        dqMINUS[3] = yMINUS[7]
        dqMINUS[4] = yMINUS[9]
        dqMINUS[5] = yMINUS[11]
        dqMINUS[6] = yMINUS[13]
        # Project EoM into the contact space:
        # Rest after contact: J*qPlus = 0
        # with: M*(qPlus-qMinus) = I_cont_q # with the contact impulse I_cont
        # qPlus = inv_M*I_cont_q + qMinus
        # -> J*inv_M*I_cont_q + qMinus = 0
        # -> J*inv_M*J'*I_cont_x + qMinus = 0
        # -> I_cont_x = -inv(J*inv_M*J')*qMinus
        # -> I_cont_q = -J'*inv(J*inv_M*J')*qMinus
        # qPlus = -inv(M)*J'*inv(J*inv(M)*J')*qMinus + qMinus
        
        imm0 = np.dot(J, la.solve(M,  np.transpose(J)))
        imm1 = la.solve(imm0, J)
        imm2 = np.dot(np.transpose(J),imm1)
        imm3 = np.identity(7) - la.solve(M, imm2)
        
        dqPLUS  = np.dot(imm3, dqMINUS)

        # Velocities after collision
        yPLUS[1]      = dqPLUS[0]
        yPLUS[3]      = dqPLUS[1]
        yPLUS[5]      = dqPLUS[2]
        yPLUS[7]      = dqPLUS[3]
        yPLUS[9]      = dqPLUS[4]
        yPLUS[11]     = dqPLUS[5]
        yPLUS[13]     = dqPLUS[6]
        # Set new phase for the front leg

        zPLUS[0] = 1
         # Intermediate event. Simulation continues
        isTerminal = False


    # Event 2: detected liftoff front leg
    elif(event == 2): 
        # Velocities remain unchanged
        # Set new phase for the front leg
        zPLUS[0]  = 2
        # Intermediate event. Simulation continues
        isTerminal = False

    # Event 3: detected touchdown back leg
    elif(event == 3): 
        # At touchdown, the contact point comes to a complete rest, a
        # fully plastic collision is computed:
        # Compute mass matrix
        M = MassMatrixWrapper(yMINUS,p)

        # Contact Forces (with Jacobian):
        posB, JB, dJBdtTIMESdqdt = ContactKinematicsWB(yMINUS, p)

        if (zMINUS[1] == 1): 
            # Double-contact
            posF, JF, dJFdtTIMESdqdt = ContactKinematicsWF(yMINUS, p)
            J  = np.asarray([[JF], [JB]])
            J = np.reshape(J, (4,7), order = 'C')
        else:
            J = JB

        # Velocities before collision:
        dqMINUS = np.zeros((7,1))
        dqMINUS[0] = yMINUS[1]
        dqMINUS[1] = yMINUS[3]
        dqMINUS[2] = yMINUS[5]
        dqMINUS[3] = yMINUS[7]
        dqMINUS[4] = yMINUS[9]
        dqMINUS[5] = yMINUS[11]
        dqMINUS[6] = yMINUS[13]

        # Project EoM into the contact space:
        # Rest after contact: J*qPlus = 0
        # with: M*(qPlus-qMinus) = I_cont_q # with the contact impulse I_cont
        # qPlus = inv_M*I_cont_q + qMinus
        # -> J*inv_M*I_cont_q + qMinus = 0
        # -> J*inv_M*J'*I_cont_x + qMinus = 0
        # -> I_cont_x = -inv(J*inv_M*J')*qMinus
        # -> I_cont_q = -J'*inv(J*inv_M*J')*qMinus
        # qPlus = -inv(M)*J'*inv(J*inv(M)*J')*qMinus + qMinus
        imm4 = np.dot(J, la.solve(M,  np.transpose(J)))
        imm5 = la.solve(imm4, J)
        imm6 = np.dot(np.transpose(J),imm5)
        imm7 = np.identity(7) - la.solve(M, imm6)
        
        dqPLUS  = np.dot(imm7, dqMINUS)
        # Velocities after collision
        yPLUS[1]      = dqPLUS[0]
        yPLUS[3]      = dqPLUS[1]
        yPLUS[5]      = dqPLUS[2]
        yPLUS[7]      = dqPLUS[3]
        yPLUS[9]      = dqPLUS[4]
        yPLUS[11]     = dqPLUS[5]
        yPLUS[13]     = dqPLUS[6]
        # Set new phase for the back leg

        zPLUS[1] = 1

        # Intermediate event. Simulation continues

        isTerminal = False

    # Event 4: detected liftoff back leg    
    elif (event == 4): 
        # Velocities remain unchanged
        # Set new phase for the back leg
        zPLUS[1]  = 2
        # Intermediate event. Simulation continues
        isTerminal = False

    # Event 5: detected stop (t == 1/strideFrequency for active systems, 
    #          otherwise dy == 0 during flight with additional timing
    #          requirements)  
    elif (event == 5):  
        
        # Subtract T_stride to force the time-variable to be periodic:
        yPLUS[14] = yMINUS[14] - 1/u[0]

        # Compute the cost of transportation.  It is important that the
        # simulation started with x = 0 in order to get this correct

        if (yMINUS[0] > 0):
                zPLUS[2] = yMINUS[15]/yMINUS[0]

        # This event stops simulation of the system
        isTerminal = True 

    return isTerminal, yPLUS, zPLUS        
      
      
      
  def JumpSet(self,y,p,z):
      ContStates = self.ContStateDef()
      DiscStates = self.DiscStateDef()
      SysParams = self.SysParamDef()
      strideFreq, sinalphaF, cosalphaF, sinlF, coslF, sinalphaB, cosalphaB, sinlB, coslB = self.ExctParamDef()
      
      
      if (y.size == 0): 
          y = ContStates
      
      if (p.size == 0):
          p = SysParams
    
      if (z.size == 0):
          z = DiscStates
          
      u = ExcitationFunction(y,strideFreq, sinalphaF, cosalphaF, sinlF, coslF, sinalphaB, cosalphaB, sinlB, coslB)    
      n_events = 5
      evntVal = np.zeros((n_events,1))
      #Pre-compute jacobians:
      if (z[0] == 1):
          
        posF, JF, dJFdtTIMESdqdt = ContactKinematicsWF(y,p)
      if (z[1] == 1): 
        
        posB, JB, dJBdtTIMESdqdt = ContactKinematicsWB(y,p)
        
      #Event 1  
      if (z[0]== 2):
        pos, JF1, dJFdtTIMESdqdt1 = ContactKinematicsWF(y, p)
        #Event is detected if front foot goes below the ground during flight 
        evntVal[0] = pos[1]
      else:
        # But only in phase 2
        evntVal[0] = 1

      #***********************************#
       
      #Event 2
      if (z[0] == 1):
#        if isempty(exctFcnHndl)
#            # Use standard values, if no function was provided:
#            u = exctStateVec;
#        else
#            # Evalute the excitation function
#            u = exctFcnHndl(y, z, s);
        
        f_diff, F_lF, T_alphaF, F_lB, T_alphaB = ComputeDifferentiableForces(y,u,p);
        # Mass matrix
        M = MassMatrixWrapper(y,p)
        # Contact Jacobian:
        if (z[1] == 1): 
            # Double-contact
            J  = np.asarray([[JF], [JB]])
            J = np.reshape(J, (4,7), order = 'C')
            dJdtTIMESdqdt = np.asarray([[dJFdtTIMESdqdt], [dJBdtTIMESdqdt]])
            dJdtTIMESdqdt = np.reshape(dJdtTIMESdqdt, (4,1))
            
        else:
            
            J = JF
            dJdtTIMESdqdt = dJFdtTIMESdqdt
        
        imm0 = -np.dot(J, (la.solve(M,f_diff)))
        imm0 = np.reshape(imm0, dJdtTIMESdqdt.shape)
        imm1 = imm0 - dJdtTIMESdqdt
        imm2 = np.dot(J,la.solve(M, np.transpose(J)))
        # Requirement for a closed contact is that the contact point
        # acceleration is zero:
       
        f_contX = la.solve(imm2, imm1)
        
        # The f_contX vector will have two or four elements, depending on
        # the number of contact points (front, or front and back), but the
        # y-component of the back foot force is always stored in the last
        # element:
        evntVal[1] = f_contX[1] + 0.1 # Add a little offset to eliminate numerical jitter        
      else:
        # But only in phase 1
        evntVal[1] = 1

      #****************************#
    
      # Event 3: Detect touchdown back leg
      if (z[1] == 2 ):
        pos, JB, dJBdtTIMESdqdt  = ContactKinematicsWB(y, p);
        # Event is detected if back foot goes below the ground during
        # flight 
        evntVal[2] = pos[1]
      else:
        # But only in phase 2
        evntVal[2] = 1

      #*******************************#
      # Event 4: Detect liftoff back leg
      if (z[1] == 1):
        # Event is detected if the contact force becomes negative.
#        if isempty(exctFcnHndl)
#            # Use standard values, if no function was provided:
#            u = exctStateVec;
#        else
#            # Evalute the excitation function
#            u = exctFcnHndl(y, z, s);
#        end
        # Compute the differentiable force vector (i.e. coriolis, gravity, and
        # actuator forces): 
        f_diff, F_lF, T_alphaF, F_lB, T_alphaB = ComputeDifferentiableForces(y,u,p);
        # Mass matrix
        M = MassMatrixWrapper(y,p);
        # Contact Jacobian:
        if (z[0] == 1): 
            # Double-contact
            J  = np.asarray([[JF], [JB]])
            J = np.reshape(J, (4,7), order = 'C')
            dJdtTIMESdqdt = np.asarray([[dJFdtTIMESdqdt], [dJBdtTIMESdqdt]])
            dJdtTIMESdqdt = np.reshape(dJdtTIMESdqdt, (4,1))
        else:
            J = JB
            dJdtTIMESdqdt = dJBdtTIMESdqdt
        # Requirement for a closed contact is that the contact point
        # acceleration is zero:
        # J*dqddt + dJdt*dqdt = 0 
        # with EoM:
        # dqddt = M_inv*(f_diff + J'*f_cont)
        # -> J*M_inv*f_diff + J*M_inv*J'*f_cont + dJdt*dqdt = 0
        # -> f_cont = inv(J*M_inv*J')*(-J*M_inv*f_diff - dJdt*dqdt)
        imm0 = -np.dot(J, (la.solve(M,f_diff)))
        imm0 = np.reshape(imm0, dJdtTIMESdqdt.shape)
        imm1 = imm0 - dJdtTIMESdqdt
        imm2 = np.dot(J,la.solve(M, np.transpose(J)))
        f_contX = la.solve(imm2, imm1)
        # The f_contX vector will have two or four elements, depending on
        # the number of contact points (back, or front and back), but the
        # y-component of the back foot force is always stored in the last
        # element:
        evntVal[3] = f_contX[f_contX.size-1] + 0.1 # Add a little offset to eliminate numerical jitter
      else:
        # But only in phase 1
        evntVal[3] = 1

    # Event 5: detect stop (t == 1/strideFrequency for active systems,
    # otherwise dy == 0 during flight with additional timing requirements)
      if (z[0] == 2 and z[1] == 2 and y[14]>0):
            evntVal[4] = -y[14] + 1/strideFreq
      else:
            # But only in phase 2 and after some time has passed since the
            # start (to allow for all four touchdown/liftoff events)
            evntVal[4] = 1

    
      return evntVal
   

#******** TESTING THE FLOW MAP ***********#
#bq = BoundingQuadruped()
#p = bq.SysParamDef()
#y0 = bq.ContStateDef()
#z0 = bq.DiscStateDef()
#y = [y0]
#t_ = 0
#dt = 0.01
#t_max = 2
#t = [t_]
#
#while(t_*dt < t_max):
#    t__ = int(t_*dt)
#    t.append((t_+ 1)*dt)
#    y_next = y[-1]  + dt*bq.FlowMap(y[-1],p,z0)
#    y.append(y_next)
#    t_ += 1
#        
#plt.plot(y)
#event = 5
#isTerminal, yPLUS, zPLUS = bq.JumpMap(y0, z0, p,event)

