#!/usr/bin/env python2

class Utils():
    @staticmethod
    def createListOfLists(rows, cols, defaultValue):
        listVal = []

        for r in range(rows):
            listVal.append([])
            
            for c in range(cols):
                listVal[r].append(defaultValue)

        return listVal

    @staticmethod
    def createList(cols, defaultValue, callable=False):
        listVal = []

        for c in range(cols):
            if callable:
                listVal.append(defaultValue())
            else:
                listVal.append(defaultValue)

        return listVal