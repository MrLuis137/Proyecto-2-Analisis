# -*- coding: utf-8 -*-
"""
Created on Sat Oct 24 11:01:07 2020

@author: lalem
"""
import math
from Point import *

class Sonar:
    
    pos = Point(0,0);
    dir = Point(0,0);
    
    def  __init__(self, pos, dir ):
        self.pos = pos;
        self.dir = dir;

    def clone(self):
        return Sonar(self.pos, self.dir);