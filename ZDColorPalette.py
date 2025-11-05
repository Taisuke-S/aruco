# -*- coding: utf-8 -*-
"""
Created on Mon Aug  8 12:48:47 2022

@author: nextEDGE Technology
"""

__author__ = "nextEDGE Technology"
__copyright__ = "Copyright (C) 2022 nextEDGE Technology K.K."
__license__ = "Public Domain"
__version__ = "1.0"

import numpy as np

ColorPalette = np.zeros((16384,4), dtype=np.uint8)
ColorPaletteValue = np.zeros((16384), dtype=np.uint32)
fCV = 180
nCenter = 1500
r1 = 0.35
r2 = 0.55
fx=0
fy = 0
R=0
G=0
B=0

def HSV_to_RGB(H, S, V):
  global R, G, B
  while H<0.0:
    H += 360.0
  while H >= 360.0:
    H -= 360.0
  H /= 60.0
  if V<0.0:
    V = 0.0
  if V>1.0:
    V = 1.0
  V *= 255.0
  if  S<0.0:
    S = 0.0
  if S>1.0:
    S = 1.0
#
  if V == 0.0:
    R = G = B = 0
  else:
    fDet = S*V
    nMax = (V)
    nMin = (V - fDet);
    if H <= 1.0:
        R = nMax
        B = nMin
        G = (H*fDet + B)
    elif H <= 2.0:
        G = nMax
        B = nMin
        R = ((2.0 - H)*fDet + B)
    elif H <= 3.0:
        G = nMax
        R = nMin
        B = ((H - 2.0)*fDet + R)
    elif H <= 4.0:
        B = nMax
        R = nMin
        G = ((4.0 - H)*fDet + R)
    elif H <= 5.0:
        B = nMax
        G = nMin
        R = ((H - 4.0)*fDet + G)
    else:
        R = nMax
        G = nMin
        B = ((6.0 - H)*fDet + G)

def BuildColorPalette():
    for i in range(16384):
      if i == nCenter:
          fy = fCV
      elif i < nCenter:
          fx = (nCenter - i) / nCenter
          fy = fCV - pow(fx, r1)*fCV
      else:
          fx = (i - nCenter) / (16384 - nCenter)
          fy = fCV + pow(fx, r2)*(256 - fCV)

      HSV_to_RGB(fy, 1.0, 1.0)
      ColorPalette[i][0] = B
      ColorPalette[i][1] = G
      ColorPalette[i][2] = R
      ColorPalette[i][3] = 255

    return ColorPalette

def BuildColorPaletteValue():
    for i in range(16384):
      if i == nCenter:
          fy = fCV
      elif i < nCenter:
          fx = (nCenter - i) / nCenter
          fy = fCV - pow(fx, r1)*fCV
      else:
          fx = (i - nCenter) / (16384 - nCenter)
          fy = fCV + pow(fx, r2)*(256 - fCV)

      HSV_to_RGB(fy, 1.0, 1.0)
      ColorPaletteValue[i] = B*65536+G*256+R

    return ColorPaletteValue
