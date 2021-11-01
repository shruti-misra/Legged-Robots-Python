#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jun  1 10:14:02 2017

@author: Shruti
"""
import numpy as np
import pylab as plt
from numpy import linalg as la
import matplotlib 
from matplotlib import rc
import math

def MassMatrix(x,y,phi,alphaF,lF,alphaB,lB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3):
    
    t557 = alphaB+phi
    t558 = np.cos(t557)
    t559 = alphaF+phi
    t560 = np.cos(t559)
    t561 = l2*m2*t560
    t562 = lF*m3*t560
    t563 = l2*m2*t558
    t564 = lB*m3*t558
    t565 = m2*2.0
    t566 = m3*2.0
    t567 = m1+t565+t566
    t568 = np.sin(t557)
    t569 = np.sin(t559)
    t570 = l2*m2*t569
    t571 = lF*m3*t569
    t572 = l2*m2*t568
    t573 = lB*m3*t568
    t587 = l3*m3*t560
    t595 = l3*m3*t558
    t574 = t561+t562+t563+t564-t587-t595
    t588 = l3*m3*t569
    t596 = l3*m3*t568
    t575 = t570+t571+t572+t573-t588-t596
    t576 = l1**2
    t577 = np.sin(alphaB)
    t578 = np.sin(alphaF)
    t579 = l2**2
    t580 = l3**2
    t581 = lF**2
    t582 = m3*t581
    t583 = m2*t579
    t584 = m3*t580
    t585 = lB**2
    t586 = m3*t585
    t589 = l1*l2*m2*t578
    t590 = l1*lF*m3*t578
    t592 = l3*lF*m3*2.0
    t591 = j2+j3+t582+t583+t584+t589+t590-t592-l1*l3*m3*t578;
    t593 = m3*t569
    t594 = np.cos(alphaF)
    t597 = l1*l3*m3*t577
    t599 = l3*lB*m3*2.0
    t598 = j2+j3+t583+t584+t586+t597-t599-l1*l2*m2*t577-l1*lB*m3*t577
    t600 = m3*t568
    t601 = np.cos(alphaB)
    t602 = l1*m3*t601
    M = np.asarray([t567,0.0,t574,t561+t562-t587,t593,t563+t564-t595,t600,0.0,t567,t575,t570+t571-t588,-m3*t560,t572+t573-t596,-m3*t558,t574,t575,j1+j2*2.0+j3*2.0+t582+t586+m2*t576*2.0+m2*t579*2.0+m3*t576*2.0+m3*t580*2.0-l3*lB*m3*2.0-l3*lF*m3*2.0-l1*l2*m2*t577*2.0+l1*l2*m2*t578*2.0+l1*l3*m3*t577*2.0-l1*l3*m3*t578*2.0-l1*lB*m3*t577*2.0+l1*lF*m3*t578*2.0,t591,-l1*m3*t594,t598,t602,t561+t562-l3*m3*t560,t570+t571-l3*m3*t569,t591,j2+j3+t582+t583+t584-t592,0.0,0.0,0.0,t593,-m3*t560,-l1*m3*t594,0.0,m3,0.0,0.0,t563+t564-l3*m3*t558,t572+t573-l3*m3*t568,t598,0.0,0.0,j2+j3+t583+t584+t586-t599,0.0,t600,-m3*t558,t602,0.0,0.0,0.0,m3])
    M = np.reshape(M, (7, 7))
    return M


def MassMatrixWrapper(y, p):
    
    x       = y[0]
    y_       = y[2]
    phi     = y[4]
    alphaF  = y[6]
    lF      = y[8]
    alphaB  = y[10]
    lB      = y[12]

    
    # Map the system paramters:
    # Keep the index-structs in memory to speed up processing

    l1    = p[7]
    l2    = p[8]
    l3    = p[9]
    rFoot = p[10]
    g     = p[0]
    m1    = p[4]
    m2    = p[5]
    m3    = p[6]
    j1    = p[11]
    j2    = p[12]
    j3    = p[13]
    
    # Call the auto-generated function
    M = MassMatrix(x,y_,phi,alphaF,lF,alphaB,lB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3)
    return M