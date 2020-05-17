#!/usr/bin/env python2

from enum import Enum
import sys
import rospy
import math
from bpqueue import BucketPriorityQueue
from utils import Utils

MAX_INT_VAL = 37268

class IntPoint():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return "x: " + str(self.x) + " y: " + str(self.y)

class State(Enum):
    VORONOI_KEEP = -4
    FREE_QUEUED = -3
    VORONOI_RETRY = -2
    VORONOI_PRUNE = -1
    FREE = 0
    OCCUPIED = 1

class QueueingState(Enum):
    FW_NOT_QUEUED = 1
    FW_QUEUED = 2
    FW_PROCESSED = 3
    BW_QUEUED = 4
    BW_PROCESSED = 5

class ObstDataState(Enum):
    INVALID_OBST_DATA = MAX_INT_VAL / 2

class MarkerMatchResult(Enum):
    PRUNED = 0
    KEEP = 1
    RETRY = 2
  

class DataCell():
    def __init__(self):
        self.dist = float("inf")
        self.voronoi = State.FREE
        self.queueing = QueueingState.FW_NOT_QUEUED
        self.obstX = ObstDataState.INVALID_OBST_DATA
        self.obstY = ObstDataState.INVALID_OBST_DATA
        self.needsRaise = False
        self.sqdist = sys.maxint

    def __repr__(self):
        return str(self.dist) + ":" + str(self.voronoi) + ":"  + str(self.queueing) + ":" + str(self.obstX) + ":" + str(self.obstY) + ":" + str(self.needsRaise) + ":" + str(self.sqdist)


class DynamicVoronoi():
    def __init__(self):
        self.data = []
        self.gridMap = []
        self.sizeX = 0
        self.sizeY = 0
        self.addList = []
        self.removeList = []
        self.openQueue = BucketPriorityQueue()
        self.pruneQueue = []

    def initializeEmpty(self, sizeX, sizeY, initGridMap):
        self.sizeX = sizeX
        self.sizeY = sizeY
        
        if len(self.data) > 0:
            del self.data[:]
                
        # self.data = Utils.createListOfLists(sizeX, sizeY, DataCell())
        self.data = []

        for r in range(sizeX):
            self.data.append([])
            
            for c in range(sizeY):
                self.data[r].append(DataCell())        

        if initGridMap:
            if len(self.gridMap) > 0:
                del self.gridMap[:]
            
            self.gridMap = Utils.createListOfLists(sizeX, sizeY, False)        
          

    def initializeMap(self, sizeX, sizeY, gridMap):
        self.gridMap = gridMap
        self.initializeEmpty(sizeX, sizeY, False)

        for x in range(sizeX):
            for y in range(sizeY):                
                if self.gridMap[x][y]:                    
                    dataCell = self.data[x][y]                    

                    if not self.isOccupied(x, y, dataCell):                        
                        isSurrounded = True

                        for dx in range(-1, 2):
                            nx = x + dx

                            if nx <=0 or nx > sizeX-1:
                                continue

                            for dy in range(-1, 2):
                                ny  = y + dy
                                
                                if ny <= 0 or ny > sizeY - 1:
                                    continue

                                if not self.gridMap[nx][ny]:
                                    isSurrounded = False
                                    break
                        
                        if isSurrounded:
                            dataCell.obstX = x
                            dataCell.obstY = y
                            dataCell.sqdist = 0
                            dataCell.dist = 0
                            dataCell.voronoi = State.OCCUPIED
                            dataCell.queueing = QueueingState.FW_PROCESSED                            
                        else:
                            self.setObstacle(x, y, dataCell)
        
    def isOccupied(self, x, y, dataCell):
        if dataCell.obstX == x and dataCell.obstY == y:
            return True
        
        return False


    def setObstacle(self, x, y, dataCell):        
        if self.isOccupied(x, y, dataCell):
            return        
        
        self.addList.append(IntPoint(x, y))
        dataCell.obstX = x
        dataCell.obstY = y

    def commitAndColorize(self, updateRealDist):
        # add new obstacles
        for add in self.addList:
            x = add.x
            y = add.y

            c = self.data[x][y]

            if c.queueing != QueueingState.FW_QUEUED:
                if updateRealDist:
                    c.dist = 0
                c.sqdist = 0
                c.obstX = x
                c.obstY = y
                c.queueing = QueueingState.FW_QUEUED
                c.voronoi = State.OCCUPIED
                self.openQueue.push(0, add)

        # remove old obstacles
        for remove in self.removeList:
            x = remove.x
            y = remove.y
            c = self.data[x][y]

            if self.isOccupied(x, y, c):
                continue
            
            self.openQueue.push(0, remove)

            if updateRealDist:
                c.dist = float("inf")
            c.sqdist = MAX_INT_VAL
            c.needsRaise = True
        
        del self.addList[:]
        del self.removeList[:]        
            

    def update(self, updateRealDist=True):
        self.commitAndColorize(updateRealDist)        

        while not self.openQueue.empty():
            p = self.openQueue.pop()

            x = p.x
            y = p.y            

            c = self.data[x][y]            

            if c.queueing == QueueingState.FW_PROCESSED:
                continue

            if c.needsRaise:
                for dx in range(-1, 2):
                    nx = x + dx

                    if nx <= 0 or nx >= self.sizeX - 1:
                        continue
                
                    for dy in range(-1, 2):

                        if (dx == 0 and dy == 0):
                            continue

                        ny = y + dy

                        if ny <=0 or ny >= self.sizeY - 1:
                            continue

                        nc = self.data[nx][ny]

                        if nc.obstX != ObstDataState.INVALID_OBST_DATA and not nc.needsRaise:
                            if not self.isOccupied(nc.obstY, nc.obstY, self.data[nc.obstX][nc.obstY]):
                                self.openQueue.push(nc.sqdist, IntPoint(nx, ny))
                                nc.queueing = QueueingState.FW_QUEUED
                                nc.needsRaise = True
                                nc.obstX = ObstDataState.INVALID_OBST_DATA
                                nc.obstY = ObstDataState.INVALID_OBST_DATA

                                if updateRealDist:
                                    nc.dist = float("inf")
                                
                                nc.sqdist = MAX_INT_VAL
                        else:
                            if nc.queueing != QueueingState.FW_QUEUED:
                                self.openQueue.push(nc.sqdist, IntPoint(nx, ny))
                                nc.queueing = QueueingState.FW_QUEUED
                
                c.needsRaise = False
                c.queueing = QueueingState.BW_PROCESSED

            elif c.obstX != ObstDataState.INVALID_OBST_DATA and self.isOccupied(c.obstX, c.obstY, self.data[c.obstX][c.obstY]):            
                c.queueing = QueueingState.FW_PROCESSED
                c.voronoi = State.OCCUPIED

                for dx in range(-1, 2):
                    nx = x + dx

                    if nx <=0 or nx >= self.sizeX -1:
                        continue

                    for dy in range(-1, 2):

                        if dx == 0 and dy == 0:
                            continue

                        ny = y + dy

                        if ny <= 0 or ny >= self.sizeY - 1:
                            continue

                        nc = self.data[nx][ny]

                        if not nc.needsRaise:
                            distx = nx - c.obstX
                            disty = ny - c.obstY
                            
                            newSqDist = int(distx*distx + disty*disty)
                            overwrite = bool(newSqDist < nc.sqdist)

                            if not overwrite and newSqDist == nc.sqdist:
                                if nc.obstX == ObstDataState.INVALID_OBST_DATA or not self.isOccupied(nc.obstX, nc.obstY, self.data[nc.obstX][nc.obstY]):
                                    overwrite = True

                            if overwrite:
                                self.openQueue.push(newSqDist, IntPoint(nx, ny))
                                nc.queueing = QueueingState.FW_QUEUED

                                if updateRealDist:
                                    nc.dist = math.sqrt(newSqDist)

                                nc.sqdist = newSqDist
                                nc.obstX = c.obstX
                                nc.obstY = c.obstY
                            else:
                                self.checkVoronoi(x, y, nx, ny, c, nc)
        
                
    def checkVoronoi(self, x, y, nx, ny, c, nc):
        if (c.sqdist > 1 or nc.sqdist > 1) and nc.obstX != ObstDataState.INVALID_OBST_DATA:
            if abs(c.obstX - nc.obstX) > 1 or abs(c.obstY - nc.obstY) > 1:
                # compute dist from x y to obstacle nx ny
                dxy_x = x - nc.obstX
                dxy_y = y - nc.obstY

                sqdxy = dxy_x * dxy_x + dxy_y * dxy_y
                stability_xy = sqdxy - c.sqdist

                if sqdxy - c.sqdist < 0:
                    return

                # compute dist from nx ny to obstacle x y
                dnxy_x = nx - c.obstX
                dnxy_y = ny - c.obstY
                sqdnxy = dnxy_x * dnxy_x + dnxy_y * dnxy_y
                stability_nxy = sqdnxy - nc.sqdist

                if sqdnxy - nc.sqdist < 0:
                    return

                if stability_xy <= stability_nxy and c.sqdist > 2:
                    if c.voronoi != State.FREE:
                        c.voronoi = State.FREE
                        self.reviveVoronoiNeighbors(x, y)
                        self.pruneQueue.append(IntPoint(x, y))

                if stability_nxy <= stability_xy and nc.sqdist > 2:
                    if nc.voronoi != State.FREE:
                        nc.voronoi = State.FREE
                        self.reviveVoronoiNeighbors(nx, ny)
                        self.pruneQueue.append(IntPoint(x, y))

    def reviveVoronoiNeighbors(self, x, y): 
        for dx in range(-1, 2):
            nx = x + dx

            if nx <= 0 or nx >= self.sizeX - 1:
                continue

            for dy in range(-1, 2):
                if dx == 0 and dy == 0:
                    continue

                ny = y + dy

                if ny <= 0 or ny >= self.sizeY - 1:
                    continue

                nc = self.data[nx][ny]

                if nc.sqdist != MAX_INT_VAL and not nc.needsRaise and (nc.voronoi == State.VORONOI_KEEP or nc.voronoi == State.VORONOI_PRUNE):
                    nc.voronoi = State.FREE
                    self.pruneQueue.append(IntPoint(nx, ny))

    def isVoronoi(self, x, y):
        c = self.data[x][y]
        if c.voronoi == State.FREE or c.voronoi == State.VORONOI_KEEP:
            return True

        return False

    def vizualize(self, filename="voronoi.pgm"):
        fpgm = open(filename, "wb")

        fpgm.write("P6\n")
        fpgm.write(str(self.sizeX) + " " + str(self.sizeY) + " 255\n")

        rospy.loginfo("sizeX: " + str(self.sizeX))
        rospy.loginfo("sizeY: " + str(self.sizeY))

        for y in range(self.sizeY-1, -1, -1):
            for x in range(self.sizeX):                                

                if self.isVoronoi(x, y):
                    fpgm.write(chr(255))
                    fpgm.write(chr(0))
                    fpgm.write(chr(0))
                elif self.data[x][y].sqdist == 0:
                    fpgm.write(chr(0))
                    fpgm.write(chr(0))
                    fpgm.write(chr(0))                    
                else:
                    f = int(80 + self.data[x][y].dist*5)

                    if f > 255: 
                        f = 255
                    if f < 0:
                        f = 0
                    
                    fpgm.write(chr(f))
                    fpgm.write(chr(f))
                    fpgm.write(chr(f))                


        fpgm.close()

    def outputToFile(self, filename):
        outFile = open(filename, "w")        

        outFile.write(str(self.gridMap))
        outFile.close()                

    def outputDataToFile(self, filename):
        outFile = open(filename, "w")        

        outFile.write(str(self.data))
        outFile.close()                