#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat May  6 15:53:10 2017

@author: Shruti
"""

import numpy as np
from numpy import linalg as la
import pylab as plt
import sys
from copy import deepcopy

class Models(object):

  def __init__(self):
      
      pass
  
  def ContStateDef(self):
      
      return 0
      
  def DiscStateDef(self):
      
      return 0 
  
  def Input(self):
      pass
      
  def SysParamDef(self):
      
      return 0
      
  def FlowMap(self,y, p):
      return 0
      
  def JumpMap(self, yMINUS, zMINUS,p):

      return 0  
      
  def JumpSet(self, y, p):
      return 0
  

      
      