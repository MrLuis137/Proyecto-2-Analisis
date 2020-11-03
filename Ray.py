# -*- coding: utf-8 -*-
"""
Created on Sat Oct 24 13:23:05 2020

@author: lalem
"""
from Point import *;

class Ray:
    intensity = 0.0;
    dir = Point(0,0);
    origin = Point(0,0);
    traveledDistance = 0
    
    def  __init__(self, intensity, origin, dir):
        self.intensity = intensity;
        self.origin = origin;
        self.dir = dir;
          
