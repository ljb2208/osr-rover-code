#!/usr/bin/env python2
import math
from utils import Utils

HEADINGS = 72
POSITIONS = 100
DELTA_HEADING_RAD = 2. * math.pi / float(HEADINGS)
DELTA_HEADING_NEG_RAD = 2. * math.pi - DELTA_HEADING_RAD
DUBINS_SHOT_DIST = 100

PENALTY_TURNING = 1.05
PENALTY_REVERSING = 2.
PENALTY_COD = 2.

CELL_SIZE = 1

COLOR_PINK = [249. / 255., 38. / 255., 114. / 255.]
COLOR_PURPLE = [174. / 255., 129. / 255., 255. / 255.]
COLOR_ORANGE = [253. / 255., 151. / 255., 31. / 255.]
COLOR_TEAL = [102. / 255., 217. / 255., 239. / 255.]
COLOR_GREEN = [166. / 255., 226. / 255., 46. / 255.]

BLOATING = 0
LENGTH = 2.65 + 2 * BLOATING ## NEED TO CHANGE TO BE CONFIGURABLE
WIDTH = 1.75 + 2 * BLOATING ## NEED TO CHANGE TO BE CONFIGURABLE
R = 6 ## NEED TO CHANGE TO BE CONFIGURABLE

MIN_ROAD_WIDTH = 2

REVERSE = True ## NEED TO CHANGE TO BE CONFIGURABLE

DUBINS = False
DUBINS_SHOT = True
DUBINS_WIDTH = 15
DUBINS_AREA = DUBINS_WIDTH * DUBINS_WIDTH

TWOD = True

ITERATIONS = 30000

class RelPos():
    def __init__(self, x, y):
        self.x = x 
        self.y = y

class Config():
    def __init__(self):
        self.length = 0
        self.relPos = Utils.createList(64, RelPos, callable=True)