#!/usr/bin/env python
# coding=utf-8


import numpy as np
import math
import random
import time
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

class Evader(object):

    def __init__(self, id, loc):
        self.id = id
        self.loc = loc
        self.v = np.array([0, 0])


    def angle(self, a, b):
        v = np.array(b - a)

        x = 1
        y = np.linalg.norm(v)
        z = np.linalg.norm(np.array(v - [1, 0]))

        costheta = (x ** 2 + y ** 2 - z ** 2) / (2 * x * y)

        if v[1] >= 0:
            theta = np.arccos(costheta)
        else:
            theta = 2 * np.pi - np.arccos(costheta)

        return theta

    def move(self):
        self.loc = self.loc + self.v

class Evaders(object):

    def __init__(self, n):

        self.n = n
        self.evaders = []
        self.v = 0.5

    def init(self):
        loc = np.array([20, 69])
        e = Evader(0, loc)
        self.evaders.append(e)

    def move(self):
        for e in self.evaders:
            e.move()