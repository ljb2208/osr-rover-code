#!/usr/bin/env python2

import math

class Helper():
    @staticmethod
    def normalizeHeadingRad(t):
        if t < 0:
            t = t - 2. * math.pi * int(t / (2. * math.pi))
            return 2. * math.pi + t

        return t - 2. * math.pi * int(t  / (2. * math.pi))

    @staticmethod
    def clamp(n, lower, upper):
        return max(lower, min(n, upper))
