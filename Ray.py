# -*- coding: utf-8 -*-
"""
Created on Sat Oct 24 13:23:05 2020

@author: lalem
"""
import Point;

class Ray():
    intencity = 0.0;
    dir = Point(0.0);
    origin = Point(0,0);
    
    def  __init__(self, intencity, origin, dir):
        self.intencity = intencity;
        self.origin = origin;
        self.dir = dir;
          