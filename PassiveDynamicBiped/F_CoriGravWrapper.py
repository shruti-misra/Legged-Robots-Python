#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat May  6 17:43:30 2017

@author: Shruti
"""

import numpy as np
from numpy import linalg as la
import pylab as plt
from F_CoriGrav import F_CoriGrav


def F_CoriGravWrapper(y, p):
    # Map the generalized coordinates:
    # Keep the index-structs in memory to speed up processing
    gamma = y[0]
    dgamma = y[1]
    alpha = y[2]
    dalpha = y[3]
    
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
    f_cg = F_CoriGrav(gamma,alpha,dgamma,dalpha,l_0,l2x,l2y,rFoot,gx,gy,m1,m2,j2);
    return f_cg