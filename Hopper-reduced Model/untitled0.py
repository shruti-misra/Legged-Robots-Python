#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Dec 13 11:06:07 2017

@author: Shruti
"""

from modelClass import Models
import numpy as np
import pylab as plt
from numpy import linalg as la
import matplotlib 
from matplotlib import rc
from scipy.integrate import ode


class Hopper(Models):
    
    def ContStateDef(self):
        
    
    def DiscStateDef(self):
    
    def SysParamDef(self):
        
        m = 1  #Upper body mass
        u = 3  #Lowe body mass
        k = 10 #Spring constant
        b = 5  #Viscuous drag of lower mass
        l = 2  #Nominal length of spring
        a = 2  #When lower mass is in contact with the ground, stiffness
        g = 9.81
        
        SysParams = np.asarray([m, u, k, b, l, a, g])
        return SysParams
        
    def FlowMap(self, y, p):
        
    def JumpMap(self, yMINUS, zMINUS, p):
        
    def JumpSet(self, y,p)