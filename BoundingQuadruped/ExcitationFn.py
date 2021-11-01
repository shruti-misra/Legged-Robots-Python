#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Jun  3 19:46:46 2017

@author: Shruti
"""

import numpy as np
import pylab as plt
from numpy import linalg as la
import matplotlib 
from matplotlib import rc

def ExcitationFunction(y,strideFreq, sinalphaF, cosalphaF, sinlF, coslF, sinalphaB, cosalphaB, sinlB, coslB):
    
    # Get a mapping for the state and parameter vectors.
    # Keep the index-structs in memory to speed up processing 
    #s is the ExctParamDef, u is the exctstatedef

    # The timing variable phi is 2*pi-periodic over one stride:
    phi  = y[14]*2*np.pi*strideFreq
    dphi = 2*np.pi*strideFreq
     # Start with a base function of 0. As no constant terms are used, the
    # average of the excitation function will remain 0 over the [0..2*pi[
    # period.
    u = np.zeros((8,1))
    
    # Create Fourier series by addition over all elements:
    
    for i in range (sinlF.size):
        u[2] = u[2] +  sinlF[i]*np.sin(phi*(i+1)) + coslF[i]*np.cos(phi*(i+1))
        u[3] = u[3] + (sinlF[i]*np.cos(phi*(i+1)) - coslF[i]*np.sin(phi*(i+1)))*dphi*(i+1)
        
    for j in range (sinalphaF.size):
        u[0] = u[0]  +  sinalphaF[j]*np.sin(phi*(j+1)) + cosalphaF[j]*np.cos(phi*(j+1))
        u[1] = u[1] + (sinalphaF[j]*np.cos(phi*(j+1)) - cosalphaF[j]*np.sin(phi*(j+1)))*dphi*(j+1)

    for k in range (sinlB.size):
        u[6]  = u[6]  +  sinlB[k]*np.sin(phi*(k+1)) + coslB[k]*np.cos(phi*(k+1))
        u[7] =  u[7] + (sinlB[k]*np.cos(phi*(k+1)) - coslB[k]*np.sin(phi*(k+1)))*dphi*(k+1)

    for l in range (sinalphaB.size):
        u[4]  = u[4]  +  sinalphaB[l]*np.sin(phi*(l+1)) + cosalphaB[l]*np.cos(phi*(l+1))
        u[5] =  u[5] + (sinalphaB[l]*np.cos(phi*(l+1)) - cosalphaB[l]*np.sin(phi*(l+1)))*dphi*(l+1)

    return u

