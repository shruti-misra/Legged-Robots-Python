#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May 31 21:43:06 2017

@author: Shruti
"""

import numpy as np
import pylab as plt
from numpy import linalg as la
import matplotlib 
from matplotlib import rc
import math



def F_CoriGrav(x,y,phi,alphaF,lF,alphaB,lB,dx,dy,dphi,dalphaF,dlF,dalphaB,dlB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3):


    t604 = alphaB+phi
    t605 = np.sin(t604)
    t606 = alphaF+phi
    t607 = np.sin(t606)
    t608 = np.cos(t604)
    t609 = np.cos(t606)
    t610 = dalphaB *l2 *m2*t605
    t611 = dalphaF*l2*m2*t607
    t612 = dphi*l2*m2*t605
    t613 = dphi*l2*m2*t607
    t614 = dalphaB*lB*m3*t605
    t615 = dalphaF*lF*m3*t607
    t616 = dphi*lB*m3*t605
    t617 = dphi*lF*m3*t607
    t618 = dlB*m3*t605
    t619 = dlF*m3*t607
    t620 = dalphaB*l2*m2*t608
    t621 = dalphaF*l2*m2*t609
    t622 = dphi*l2*m2*t608
    t623 = dphi*l2*m2*t609
    t624 = dalphaB*lB*m3*t608
    t625 = dalphaF*lF*m3*t609
    t626 = dphi*lB*m3*t608
    t627 = dphi*lF*m3*t609
    t628 = np.sin(alphaB)
    t629 = dphi*l3*m3*2.0
    t630 = np.sin(alphaF)
    t631 = np.cos(alphaB)
    t632 = np.cos(alphaF)
    t633 = dy*l2*m2*t609
    t634 = dy*l3*m3*t608
    t635 = dy*lF*m3*t609
    t636 = dx*l2*m2*t605
    t637 = dx*l3*m3*t607
    t638 = dx*lB*m3*t605
    t639 = dalphaF*lF*m3*2.0
    t640 = dphi*lF*m3*2.0
    t641 = dx*m3*t609
    t642 = dy*m3*t607
    t643 = l2*m2*t607
    t644 = lF*m3*t607
    t645 = dlF*dx*m3*t609
    t646 = dlF*dy*m3*t607
    t647 = dphi**2
    t648 = dalphaF*dy*l2*m2*t609
    t649 = dphi*dy*l2*m2*t609
    t650 = dalphaF*dy*lF*m3*t609
    t651 = dphi*dy*lF*m3*t609
    t652 = dalphaF*dx*l3*m3*t607
    t653 = dphi*dx*l3*m3*t607
    t654 = dphi*l1*m3*t630
    t655 = dalphaF**2
    t656 = dy*l2*m2*t608
    t657 = dy*lB*m3*t608
    t658 = dx*l3*m3*t605
    t659 = dalphaB*l3*m3*2.0
    t660 = l2*m2*t605
    t661 = lB*m3*t605
    t662 = dlB*dx*m3*t608
    t663 = dlB*dy*m3*t605
    t664 = dalphaB*dy*l2*m2*t608
    t665 = dphi*dy*l2*m2*t608
    t666 = dalphaB*dy*lB*m3*t608
    t667 = dphi*dy*lB*m3*t608
    t668 = dalphaB*dx*l3*m3*t605
    t669 = dphi*dx*l3*m3*t605
    t670 = dx*m3*t608
    t671 = dy*m3*t605
    t672 = dalphaB**2
    f_cg = np.asarray([[dphi*(t610+t611+t612+t613+t614+t615+t616+t617-dlB*m3*t608-dlF*m3*t609-dalphaB*l3*m3*t605-dalphaF*l3*m3*t607-dphi*l3*m3*t605-dphi*l3*m3*t607)-dlB*(dalphaB*m3*t608+dphi*m3*t608)-dlF*(dalphaF*m3*t609+dphi*m3*t609)+dalphaB*(t610+t612+t614+t616-dlB*m3*t608-dalphaB*l3*m3*t605-dphi*l3*m3*t605)+dalphaF*(t611+t613+t615+t617-dlF*m3*t609-dalphaF*l3*m3*t607-dphi*l3*m3*t607)], [-g*(m1+m2*2.0+m3*2.0)-dlB*(dalphaB*m3*t605+dphi*m3*t605)-dlF*(dalphaF*m3*t607+dphi*m3*t607)-dalphaB*(t618+t620+t622+t624+t626-dalphaB*l3*m3*t608-dphi*l3*m3*t608)-dalphaF*(t619+t621+t623+t625+t627-dalphaF*l3*m3*t609-dphi*l3*m3*t609)-dphi*(t618+t619+t620+t621+t622+t623+t624+t625+t626+t627-dalphaB*l3*m3*t608-dalphaF*l3*m3*t609-dphi*l3*m3*t608-dphi*l3*m3*t609)], [t645+t646+t648+t649+t650+t651+t652+t653+t662+t663+t664+t665+t666+t667+t668+t669-g*(t643+t644+t660+t661-l3*m3*t605-l3*m3*t607)-dlF*(-t629+t639+t640+t641+t642-dalphaF*l3*m3*2.0+dalphaF*l1*m3*t630+dphi*l1*m3*t630*2.0)-dphi*(t633-t634+t635-t636+t637-t638+t656+t657+t658-dx*l2*m2*t607-dy*l3*m3*t609-dx*lF*m3*t607)+dlB*(t629+t659-dalphaB*lB*m3*2.0-dphi*lB*m3*2.0-dx*m3*t608-dy*m3*t605+dalphaB*l1*m3*t628+dphi*l1*m3*t628*2.0)+dalphaB*(t634+t636+t638+dlB*l1*m3*t628-dy*l2*m2*t608-dx*l3*m3*t605-dy*lB*m3*t608+dalphaB*l1*l2*m2*t631-dalphaB*l1*l3*m3*t631+dphi*l1*l2*m2*t631*2.0-dphi*l1*l3*m3*t631*2.0+dalphaB*l1*lB*m3*t631+dphi*l1*lB*m3*t631*2.0)-dalphaF*(t633+t635+t637+dlF*l1*m3*t630-dx*l2*m2*t607-dy*l3*m3*t609-dx*lF*m3*t607+dalphaF*l1*l2*m2*t632-dalphaF*l1*l3*m3*t632+dphi*l1*l2*m2*t632*2.0-dphi*l1*l3*m3*t632*2.0+dalphaF*l1*lF*m3*t632+dphi*l1*lF*m3*t632*2.0)-dalphaB*dx*l2*m2*t605-dalphaF*dx*l2*m2*t607-dalphaB*dy*l3*m3*t608-dalphaF*dy*l3*m3*t609-dphi*dx*l2*m2*t605-dphi*dx*l2*m2*t607-dalphaB*dx*lB*m3*t605-dphi*dy*l3*m3*t608-dphi*dy*l3*m3*t609-dalphaF*dx*lF*m3*t607-dphi*dx*lB*m3*t605-dphi*dx*lF*m3*t607], [t645+t646+t648+t649+t650+t651+t652+t653-dalphaF*(t633+t635+t637-dx*l2*m2*t607-dy*l3*m3*t609-dx*lF*m3*t607+dphi*l1*l2*m2*t632-dphi*l1*l3*m3*t632+dphi*l1*lF*m3*t632)-dphi*(t633+t635+t637-dx*l2*m2*t607-dy*l3*m3*t609-dx*lF*m3*t607)-g*(t643+t644-l3*m3*t607)-dlF*(-t629+t639+t640+t641+t642+t654-dalphaF*l3*m3*2.0)-dalphaF*dx*l2*m2*t607-dalphaF*dy*l3*m3*t609+dlF*dphi*l1*m3*t630-dphi*dx*l2*m2*t607-dphi*dy*l3*m3*t609-dalphaF*dx*lF*m3*t607-dphi*dx*lF*m3*t607+l1*l2*m2*t632*t647-l1*l3*m3*t632*t647+l1*lF*m3*t632*t647+dalphaF*dphi*l1*l2*m2*t632-dalphaF*dphi*l1*l3*m3*t632+dalphaF*dphi*l1*lF*m3*t632], [-dphi*(t641+t642)-dalphaF*(t641+t642+t654)+g*m3*t609-l3*m3*t647-l3*m3*t655+lF*m3*t647+lF*m3*t655-dalphaF*dphi*l3*m3*2.0+dalphaF*dphi*lF*m3*2.0+dalphaF*dx*m3*t609+dalphaF*dy*m3*t607+dphi*dx*m3*t609+dphi*dy*m3*t607+l1*m3*t630*t647+dalphaF*dphi*l1*m3*t630], [t662+t663+t664+t665+t666+t667+t668+t669+dalphaB*(t634+t636+t638-t656-t657-t658+dphi*l1*l2*m2*t631-dphi*l1*l3*m3*t631+dphi*l1*lB*m3*t631)-g*(t660+t661-l3*m3*t605)+dphi*(t634+t636+t638-t656-t657-t658)-dlB*(-t629-t659+t670+t671+dalphaB*lB*m3*2.0+dphi*lB*m3*2.0-dphi*l1*m3*t628)-dalphaB*dx*l2*m2*t605-dalphaB*dy*l3*m3*t608-dlB*dphi*l1*m3*t628-dphi*dx*l2*m2*t605-dalphaB*dx*lB*m3*t605-dphi*dy*l3*m3*t608-dphi*dx*lB*m3*t605-l1*l2*m2*t631*t647+l1*l3*m3*t631*t647-l1*lB*m3*t631*t647-dalphaB*dphi*l1*l2*m2*t631+dalphaB*dphi*l1*l3*m3*t631-dalphaB*dphi*l1*lB*m3*t631], [-dphi*(t670+t671)-dalphaB*(t670+t671-dphi*l1*m3*t628)+g*m3*t608-l3*m3*t647-l3*m3*t672+lB*m3*t647+lB*m3*t672-dalphaB*dphi*l3*m3*2.0+dalphaB*dphi*lB*m3*2.0+dalphaB*dx*m3*t608+dalphaB*dy*m3*t605+dphi*dx*m3*t608+dphi*dy*m3*t605-l1*m3*t628*t647-dalphaB*dphi*l1*m3*t628]])
    return f_cg

    
def F_CoriGravWrapper(y, p):

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
    f_cg = F_CoriGrav(x,y_,phi,alphaF,lF,alphaB,lB,dx,dy,dphi,dalphaF,dlF,dalphaB,dlB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3)
    return f_cg
    