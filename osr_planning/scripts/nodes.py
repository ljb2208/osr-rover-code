#!/usr/bin/env python

from constants import *
import random
from helper import Helper

NODE3D_DIR = 3
NODE3D_DY = [ 0,-0.0415893,0.0415893]
NODE3D_DX = [0.7068582,   0.705224,   0.705224]
NODE3D_DT = [0,         0.1178097,   -0.1178097]

class Node():
    def __init__(self ,x, y, g, h, pred):
        self.x = x
        self.y = y        
        self.g = g
        self.h = h
        self.pred = pred        
        self.idx = -1
        self.o = False
        self.c = False
        self.d = False

    def isOpen(self):
        return self.o

    def isClosed(self):
        return self.c

    def isDiscovered(self):
        return self.d

    def getC(self):
        return self.g + self.h

    def open(self):
        self.o = True
        self.c = False

    def close(self):
        self.o = False
        self.c = True
    
    def reset(self):
        self.c = False
        self.o = False

    def discover(self):
        self.d = True


class Node3D(Node):
    def __init__(self, x, y, t, g, h, pred, prim):
        super().__init__(x, y, g, h, pred)
        self.t = t
        self.prim = prim

    def isOnGrid(self, width, height):
        if self.x < 0 and self.x >= width:
            return False
        
        if self.y < 0 and self.y >= height:
            return False

        if int(self.t / DELTA_HEADING_RAD) < 0:
            return False
        
        if int(self.t / DELTA_HEADING_RAD) >= HEADINGS:
            return False

        return True

    def isInRange(self, goal):
        random.seed()
        rnum = random.random() % 10 + 1

        dx = abs(self.x - goal.x) / rnum
        dy = abs(self.y - goal.y) / rnum

        if ((pow(dx, 2) + pow(dy, 2)) < DUBINS_SHOT_DIST):
            return True
        
        return False
        
    def createSuccessor(self, i):

        if i < 3:
            # calculate successor position forward
            xSucc = self.x + NODE3D_DX[i] * math.cos(self.t) - NODE3D_DY[i] * math.sin(self.t)
            ySucc = self.y + NODE3D_DX[i] * math.sin(self.t) + NODE3D_DY[i] * math.cos(self.t)
            tSucc = Helper.normalizeHeadingRad(self.t + NODE3D_DT[i])
        else:
            # calculate successor position backward
            xSucc = self.x + NODE3D_DX[i - 3] * math.cos(self.t) - NODE3D_DY[i - 3] * math.sin(self.t)
            ySucc = self.y + NODE3D_DX[i - 3] * math.sin(self.t) + NODE3D_DY[i - 3] * math.cos(self.t)
            tSucc = Helper.normalizeHeadingRad(self.t - NODE3D_DT[i - 3])
        
        return Node3D(xSucc, ySucc, tSucc, self.g, 0, self, i)

    # movement cost calculation
    def updateG(self):
        # forward driving
        if self.prim < 3:
            # penalize turning
            if self.pred.prim != self.prim:
                # penalize change in direction
                if self.pred.prim > 2:
                    self.g += NODE3D_DX[0] * PENALTY_TURNING * PENALTY_COD
                else:
                    self.g += NODE3D_DX[0] * PENALTY_TURNING
            else:
                self.g += NODE3D_DX[0]
        else:
            # reverse driving
            # penalize turning and reversing
            if self.pred.prim != self.prim:
                # penalize change in direction
                if self.pred.prim > 2:
                    self.g += NODE3D_DX[0] * PENALTY_TURNING * PENALTY_COD * PENALTY_REVERSING
                else:
                    self.g += NODE3D_DX[0] * PENALTY_TURNING * PENALTY_REVERSING
            else:
                self.g += NODE3D_DX[0] * PENALTY_REVERSING

    def __eq__(self, other):    
        if self.x == other.x and self.y == other.y and (abs(self.t - other.t) <= DELTA_HEADING_RAD or abs(self.t - other.t) >= DELTA_HEADING_NEG_RAD):
            return True
        
        return False

NODE2D_DIR = 8
NODE2D_DX = [-1, -1, 0, 1, 1, 1, 0, -1 ]
NODE2D_DY = [0, 1, 1, 1, 0, -1, -1, -1 ]


class Node2D(Node):
    def __init__(self, x, y, g, h, pred):
        super().__init__(x, y, g, h, pred)        

    def isOnGrid(self, width, height):
        if self.x >=0 and self.x <  width and self.y >=0 and self.y < height:
            return True
        
        return False
    
    def createSuccessor(self, i):
        xSucc = self.x + NODE2D_DX[i]
        ySucc = self.y + NODE2D_DY[i]

        return Node2D(xSucc, ySucc, self.g, 0, self)

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y:
            return True

        return False

    def movementCost(self, pred):
        return math.sqrt(math.pow(self.x - pred.x, 2) + math.pow(self.y - pred.y, 2))

    
    def updateG(self):
        self.g += self.movementCost(self.pred)
        self.d = True

    def updateH(self, goal):
        self.h = self.movementCost(goal)