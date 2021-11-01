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

def ContactPointB(x,y,phi,alphaF,lF,alphaB,lB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3):


	t676 = alphaB+phi
	cont_pointB = np.asarray([[x-l1*np.cos(phi)+lB*np.sin(t676)], [-rFoot+y-lB*np.cos(t676)-l1*np.sin(phi)]])
	return cont_pointB


def ContactJacobianBDtTIMESdqdt(x,y,phi,alphaF,lF,alphaB,lB,dx,dy,dphi,dalphaF,dlF,dalphaB,dlB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3):


	t697 = alphaB+phi
	t698 = np.sin(t697)
	t699 = np.cos(t697)
	t700 = dalphaB*lB*t698
	t701 = dlB*t698
	t702 = dalphaB*lB*t699
	dJBdtTIMESdqdt = np.asarray([[-dalphaB*(t700-dlB*t699+dphi*lB*t698)+dlB*(dalphaB*t699+dphi*t699)-dphi*(t700+dphi*(lB*t698-l1*np.cos(phi))-dlB*t699)],[dphi*(t701+t702+dphi*(lB*t699+l1*np.sin(phi)))+dlB*(dalphaB*t698+dphi*t698)+dalphaB*(t701+t702+dphi*lB*t699)]])
	return dJBdtTIMESdqdt
 
def ContactJacobianB(x,y,phi,alphaF,lF,alphaB,lB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3):


	t684 = alphaB+phi
	t685 = np.cos(t684)
	t686 = lB*t685
	t687 = np.sin(t684)
	t688 = lB*t687
	JB = np.asarray([1.0,0.0,0.0,1.0,rFoot+t686+l1*np.sin(phi),t688-l1*np.cos(phi),0.0,0.0,0.0,0.0,rFoot+t686,t688,t687,-t685])
	JB = np.reshape(JB, (2,7), order = 'F')
	return JB
 
def ContactKinematicsWB(y,p):
    
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
    pos = ContactPointB(x,y_,phi,alphaF,lF,alphaB,lB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3)
    dJdtTimesdqdt = ContactJacobianBDtTIMESdqdt(x,y_,phi,alphaF,lF,alphaB,lB,dx,dy,dphi,dalphaF,dlF,dalphaB,dlB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3)
    J  = ContactJacobianB(x,y_,phi,alphaF,lF,alphaB,lB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3)
    return pos, J, dJdtTimesdqdt