#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jun  1 13:58:39 2017

@author: Shruti
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jun  1 13:42:02 2017

@author: Shruti
"""
import numpy as np
import pylab as plt
from numpy import linalg as la
import matplotlib 
from matplotlib import rc
import math

def ContactPointF(x,y,phi,alphaF,lF,alphaB,lB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3):


	t674 = alphaF+phi;
	cont_pointF = np.asarray([[x+l1*np.cos(phi)+lF*np.sin(t674)],[-rFoot+y-lF*np.cos(t674)+l1*np.sin(phi)]])
	return cont_pointF


def ContactJacobianFDtTIMESdqdt(x,y,phi,alphaF,lF,alphaB,lB,dx,dy,dphi,dalphaF,dlF,dalphaB,dlB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3):


	t690 = alphaF+phi
	t691 = np.sin(t690)
	t692 = np.cos(t690)
	t693 = dalphaF*lF*t691
	t694 = dlF*t691
	t695 = dalphaF*lF*t692
	dJFdtTIMESdqdt = np.asarray([[-dalphaF*(t693-dlF*t692+dphi*lF*t691)+dlF*(dalphaF*t692+dphi*t692)-dphi*(t693+dphi*(lF*t691+l1*np.cos(phi))-dlF*t692)],[dphi*(t694+t695+dphi*(lF*t692-l1*np.sin(phi)))+dlF*(dalphaF*t691+dphi*t691)+dalphaF*(t694+t695+dphi*lF*t692)]])
	return dJFdtTIMESdqdt
 
def ContactJacobianF(x,y,phi,alphaF,lF,alphaB,lB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3):


	t678 = alphaF+phi
	t679 = np.cos(t678)
	t680 = lF*t679
	t681 = np.sin(t678)
	t682 = lF*t681
	JF = np.asarray([1.0,0.0,0.0,1.0,rFoot+t680-l1*np.sin(phi),t682+l1*np.cos(phi),rFoot+t680,t682,t681,-t679,0.0,0.0,0.0,0.0])
	JF = np.reshape(JF, (2,7), order='F' )
	return JF
 
def ContactKinematicsWF(y,p):
    
    x       = y[0]
    y_      = y[2]
    phi     = y[4]
    alphaF  = y[6]
    lF      = y[8]
    alphaB  = y[10]
    lB      = y[12]
    dx      = y[1]
    dy      = y[3]
    dphi    = y[5]
    dalphaF = y[7]
    dlF     = y[9]
    dalphaB = y[11]
    dlB     = y[13]
    
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
    pos = ContactPointF(x,y_,phi,alphaF,lF,alphaB,lB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3)
    dJdtTimesdqdt = ContactJacobianFDtTIMESdqdt(x,y_,phi,alphaF,lF,alphaB,lB,dx,dy,dphi,dalphaF,dlF,dalphaB,dlB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3)
    J  = ContactJacobianF(x,y_,phi,alphaF,lF,alphaB,lB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3)
    return pos, J, dJdtTimesdqdt