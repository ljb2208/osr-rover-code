#!/usr/bin/env python2
import rospy
from utils import Utils

class BucketPriorityQueue():
    def __init__(self):
        self.count = 0
        self.buckets = []
        self.maxDistance = 1000
        self.numBuckets = 0
        self.nextBucket = 32768

        self.initSqrIndices()        
        self.initBuckets()        


    def empty(self):
        if self.count > 0:
            return False

        return True


    def push(self, priority, intPoint):
        if priority >= len(self.sqrIndices):
            rospy.logerr("priority not valid 1. " + str(priority) + " " + str(intPoint))
            return

        idVal = self.sqrIndices[priority]

        if idVal < 0:
            rospy.logerr("priority not valid 2. " + str(priority) + " " + str(intPoint) + " " + str(idVal))
            return

        self.buckets[idVal].append(intPoint)

        if idVal < self.nextBucket:
            self.nextBucket = idVal

        
        self.count += 1        

    def pop(self):

        i = self.nextBucket        

        while i < len(self.buckets):
            if len(self.buckets[i]) > 0:
                break      
            i += 1  

        self.nextBucket = i
        intPoint = self.buckets[i].pop(0)
        self.count -= 1                
        return intPoint

    def initSqrIndices(self):
        count = 0

        # self.sqrIndices = [-1] * (2 * self.maxDistance * self.maxDistance + 1)
        self.sqrIndices = Utils.createList(2 * self.maxDistance * self.maxDistance + 1, -1)

        for x in range(self.maxDistance + 1):
            for y in range(x + 1):
                sqr = x * x + y * y
                self.sqrIndices[sqr] = count
                count += 1
        
        self.numBuckets = count            

    def initBuckets(self):
        self.buckets = Utils.createListOfLists(self.numBuckets, 0, None)
        # self.buckets = [[]] * self.numBuckets