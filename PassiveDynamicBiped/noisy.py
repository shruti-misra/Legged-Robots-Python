#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 00:29:39 2017

@author: Shruti
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  7 05:05:20 2017

@author: Shruti
"""

import numpy as np
from numpy import linalg as la
from PassiveDynamicBiped import PassiveDynamicBiped
from HybridDynamics import HybridDynamics, obs
import pylab as plt
import scipy
import sys
from copy import deepcopy
import math
from sklearn.cluster import spectral_clustering

#Initialize simulation parameters
pdb = PassiveDynamicBiped()
p = pdb.SysParamDef()
y0 = pdb.ContStateDef()
z0 = pdb.DiscStateDef()
dt  = 1e-2
rx = 1e-2
K = 2
T = 4
n_trj = 1
data = np.zeros((n_trj, 351, 4))

for i in range(n_trj):
    #Simulate trajectories using the Hybrid Dynamics Function 
    trjs = HybridDynamics(y0, z0, p, K, T, rx,dt)
    trajs.append(trjs)
    
    k = []; t = []; j = []; x = []
    #Extract trajectories
    k,t,j,x_c = obs(trjs,True, False, None, p)

    #Add noise to data points
    w = np.random.normal(0, 0.5, (len(x_c),4))
    x_noise = x_c + w
    data[i, :, :] = x_noise
    
A = np.zeros((351, 351))
beta = 1 #1.0/states?
#Compute affinity matrix
for i in range(351):
    for j in range(351):
#        
        diff = data[0,i,:] - data[0,j,:]
        diff_norm = (np.linalg.norm(diff))**2
        A[i][j] = np.exp(-beta*diff_norm)
 
labels = spectral_clustering(A, n_clusters=2, eigen_solver='arpack')
       
#k = 2
#[w, v] = np.linalg.eig(A)
#V = np.zeros((len(x_noise), k))
#V[:,0] = v[:,0]
#V[:,1] = v[:,1]



