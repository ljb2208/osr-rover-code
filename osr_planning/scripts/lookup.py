#!/usr/bin/env python2
from dubins import *
from constants import *

class Lookup():

    def __init__(self):
        self.dubins = Dubins()


    def dubinsLookup(self, lookup):
        width = DUBINS_WIDTH / CELL_SIZE
        headings = HEADINGS

        start = [0., 0., 0.]
        goal = [0, 0, 0]

        path = DubinsPath(0, 0, 0, 0)

        for x in range(width):
            start[0] = x

            for y in range(width):
                start[1] = y

                # iterate over the start headings
                for h0 in range(headings):
                    start[2] = DELTA_HEADING_RAD * h0

                    # iterate over the goal headings
                    for h1 in range(headings):
                        goal[2] = DELTA_HEADING_RAD * h1

                        # calculate the actual cost                        
                        retval, path = self.dubins.dubinsInit(start, goal, R, path)
                        lookup[x*headings*headings*width + y * headings * headings + h0 * headings + h1] = self.dubins.dubinsPathLength(path)
        
    @staticmethod
    def sign(x):
        if x >= 0.:
            return 1
        
        return -1
