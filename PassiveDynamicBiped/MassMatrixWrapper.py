#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat May  6 16:28:20 2017

@author: Shruti
"""

import numpy as np
from numpy import linalg as la
import pylab as plt
from MassMatrix import MassMatrix


def MassWrapper(y, p):
    # Map the generalized coordinates:
    # Keep the index-structs in memory to speed up processing
    gamma = y[0]
    alpha = y[2]
    
    # Map the system parameters:
    # Keep the index-structs in memory to speed up processing
    gx    = p[0]
    gy    = p[1]
    l_0   = p[2]
    m1    = p[3]
    m2    = p[4]
    l2x   = p[5]
    l2y   = p[6]
    rFoot = p[7]
    j2    = p[8]
    
    # Call the auto-generated function
    M = MassMatrix(gamma,alpha,l_0,l2x,l2y,rFoot,gx,gy,m1,m2,j2);
    return M