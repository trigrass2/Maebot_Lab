# -*- coding: utf-8 -*-
"""
Created on Tue Oct 13 17:24:01 2015

@author: student
"""

import numpy
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from astar import *

#def initShowPath()
m = numpy.load("./temp_path_0.npy")
filter_mat = sum_filter(ROBO_DIAMETER, ROBO_DIAMETER)
valid_map = scipy.signal.convolve2d(m, filter_mat, 'same')

#odompath = [(350, 350), (355, 354), (356, 354), (351, 361), (351, 346), (342, 335)]
#path = [(355, 355), (360, 359), (361, 359), (356, 366), (356, 351)]
#[pathx, pathy] = zip(*path)
#[ox, oy] = zip(*odompath)

#imgplot = plt.imshow(m, cmap=cm.Greys_r)

#def showPath()
imgplot = plt.imshow(valid_map)

#plt.plot(pathy, pathx, 'red')
#plt.plot(oy, ox, 'green')
#p = astar(m, path[-1], (450, 350), False)
p2 = astar(m, (350, 350), (450, 350), False)
#px, py] = zip(*p)
[p2x, p2y] = zip(*p2)
#plt.plot(py, px, 'pink')
plt.plot(p2y, p2x, 'orange')
plt.show()