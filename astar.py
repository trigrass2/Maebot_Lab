import numpy
import scipy.signal
import scipy.ndimage.filters
from Queue import PriorityQueue as pqueue
from Queue import Queue as queue
from math import sqrt, pow
from sys import stderr
from PIL import Image

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.cm as cm

ROBO_DIAMETER = 16
VALID_THRESHOLD = 120
VALID_PATH = 200
VALID_COUNT = 5


valid_map = []


def test():
    global valid_map
    width = 2000
    height = 2000
    #m = empty_matrix(width, height)
    print >> stderr, "Map 1"
    m = numpy.load("./failed_path.npy")
    path = astar(m, (356, 349), (450, 350), False)
    
    #filter_mat = sum_filter(ROBO_DIAMETER, ROBO_DIAMETER)
    #valid_map = scipy.ndimage.filters.gaussian_filter(valid_pixels(m, VALID_THRESHOLD), 5.0)
    #valid_map = scipy.signal.convolve2d(valid_pixels(m, VALID_THRESHOLD), filter_mat, 'same')

    #print >> stderr, "Map 2"
    #m = numpy.load("./test_data/map2_3x3meters.npy")
    #path = astar(m, (236, 143), (61, 144), True)

    #print >> stderr, "Map 3"
    #m = numpy.load("./test_data/map3_3v3meters.npy")
    #path = astar(m, (274, 132), (47, 176), True)

    #print >> stderr, "Map 4"
    #m = numpy.load("./test_data/map4_3x3meters.npy")
    #path = astar(m, (281, 152), (36, 165), True)

    #The following is for printing an image of the path
    imgplot = plt.imshow(m, cmap=cm.Greys_r)
    [pathx, pathy] = zip(*path)
    plt.plot(pathy, pathx, 'red')
    plt.show()

def testLidar():
    
    img = Image.open('./current_map.png').convert('L')
    m = numpy.asarray(img)
    path = astar(m, (350, 350), (47, 176), False)
    print path
    imgplot = plt.imshow(m, cmap=cm.Greys_r)
    
    [pathx, pathy] = zip(*path)
    plt.plot(pathy, pathx)
    plt.show()

#(row, col)
def valid_neighbors(cell, width, height):
    for dr in [-1, 0, 1]:
        for dc in [-1, 0, 1]:
            if dr == 0 and dc == 0: continue
            new_cell = (cell[0]+dr, cell[1]+dc)
            if new_cell[0] >= 0 and new_cell[0] < height and \
                new_cell[1] >= 0 and new_cell[1] < width:
                yield new_cell

def heuristic(cell, end):
    dx = cell[1] - end[1]
    dy = cell[0] - end[0]
    return sqrt(pow(dx,2) + pow(dy,2))

def is_valid(cell, m):
    return m[cell[0]][cell[1]] < VALID_COUNT

@numpy.vectorize
def valid_pixels(x, t):
    if x > t:
        return 0.0
    else:
        return 1.0

def sum_filter(width, height):
    m = []
    for r in range(height):
                m.append([0.0]*width)
                for c in range(width):
                        rad = sqrt(pow(r-height/2,2) + pow(c-width/2,2))
                        if rad <= width/2:
                                m[r][c] = 1.0
    return m

def nearest_good_cell(m, start):
    q = queue()
    q.put(start)
    
    height = len(m)
    width = len(m[0])
    
    while not q.empty():
        cell = q.get()
        if is_valid(cell, m):
            return cell

        for neighbor in valid_neighbors(cell, width, height):
            q.put(neighbor)
    return None
            
            
def astar(m, start, end, debug):
    filter_mat = sum_filter(ROBO_DIAMETER, ROBO_DIAMETER)
    #valid_map = scipy.ndimage.filters.gaussian_filter(valid_pixels(m, VALID_THRESHOLD), 5.0)
    valid_map = scipy.signal.convolve2d(valid_pixels(m, VALID_THRESHOLD), filter_mat, 'same')
    if debug:
        imgplot = plt.imshow(valid_map)
        plt.show()

    q = pqueue()
    #put into the queue a weight of the distance plus the heuristics
    #and a tuple of the current distance, and the next node to expand
    if not is_valid(start, valid_map):
        print "Finding new start"
        start = nearest_good_cell(valid_map, start)
        assert(start != None)
        print "Found new start"
        
    q.put((0, (0, start)))
        
    height = len(m)
    width = len(m[0])
    closed = set()
    closed.add(start)
    parents = {}

    path_found = False
    while not q.empty():
        (weight, (dist, cell)) = q.get()
        if cell == end:
            path_found = True
            break
        for neighbor in valid_neighbors(cell, width, height):
            if is_valid(neighbor, valid_map) and (not neighbor in closed):    
                weight = dist + 1 + heuristic(neighbor, end)
                new_dist = dist + 1
                q.put((weight, (new_dist, neighbor)))
                closed.add(neighbor)
                parents[neighbor] = cell

    if path_found:
        cur_cell = end
        path = [end]
        while cur_cell != start:
            cur_cell = parents[cur_cell]
            path.append(cur_cell)

        path.reverse()
        return path
    else:
        print >> stderr, "NO PATH"
        return []

if __name__ == '__main__':
    test()
