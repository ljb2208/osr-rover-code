from constants import *
from vector import Vector2D
from helper import *
import rospy

class Smoother():
    def __init__(self):
        self.path = []
        self.kappaMax = 1. / (R * 1.1)
        self.obsDMax = MIN_ROAD_WIDTH
        self.vorObsDMax= MIN_ROAD_WIDTH
        self.alpha = 0.1
        self.wObstacle = 0.2
        self.wVoronoi = 0
        self.wCurvature = 0
        self.wSmoothness = 0.2
        self.voronoi = None
        self.width = 0
        self.height = 0

    def isOnGrid(self, vec):
        if vec.x >=0 and vec.x < self.width and vec.y >= 0 and vec.y < self.height:
            return True
        
        return False

    def isCusp(self, path, i):
        revim2 = False
        
        if path[i - 2].prim > 3:
            revim2 = True

        revim1 = False

        if path[i -1].prim > 3:
            revim1 = True
        
        revi = False

        if path[i].prim > 3:
            revi = True

        revip1 = False

        if path[i+1].prim > 3:
            revip1 = True

        if revim2 != revim1 or revim1 != revi or revi != revip1:
            return True
        
        return False

    def smoothPath(self, voronoi):
        self.voronoi = voronoi
        self.width = voronoi.sizeX
        self.height = voronoi.sizeY

        iterations = 0
        maxIterations = 500
        pathLength = len(self.path)

        newPath = self.path

        totalWeight = self.wSmoothness + self.wCurvature + self.wVoronoi + self.wObstacle


        while iterations < maxIterations:
            i=2
            while i < pathLength - 2:

                xim2 = Vector2D(newPath[i-2].x, newPath[i-2].y)
                xim1 = Vector2D(newPath[i-1].x, newPath[i-1].y)
                xi = Vector2D(newPath[i].x, newPath[i].y)
                xip1 = Vector2D(newPath[i+1].x, newPath[i+1].y)
                xip2 = Vector2D(newPath[i+2].x, newPath[i+2].y)
                correction = Vector2D(0, 0)

                if self.isCusp(newPath, i):
                    continue
            
                correction = correction - self.obstacleTerm(xi)

                if not self.isOnGrid(xi+correction):
                    continue

                # TODO Implement Voroni term

                correction = correction - self.smoothnessTerm(xim2, xim1, xi, xip1, xip2)

                if not self.isOnGrid(xi+correction):
                    continue

                correction = correction - self.curvatureTerm(xim1, xi, xip1)

                if not self.isOnGrid(xi+correction):
                    continue

                xi = xi + self.alpha + correction/totalWeight

                newPath[i].x = xi.x
                newPath[i].y = xi.y
                dxi = xi - xim1
                newPath[i-1].t = math.atan2(dxi.y, dxi.x)

                i += 1
            
            iterations += 1
        
        self.path = newPath

    def obstacleTerm(self, xi):
        gradient = Vector2D(0, 0)

        obstDst = self.voronoi.getDistance(xi.x, xi.y)

        x = int(xi.x)
        y = int(xi.y)

        # if node is within the map
        if x < self.width and x >=0 and y < self.height and y >=0:
            obsVct = Vector2D(xi.x - self.voronoi.data[int(xi.x)][int(xi.y)].obstX, xi.Y - self.voronoi.data[int(xi.x)][int(xi.y)].obstY)

            if obstDst < self.obsDMax:
                return self.wObstacle * 2 * (obstDst - self.obsDMax) * obsVct / obstDst

        return gradient

    def curvatureTerm(self, xim1, xi, xip1):
        gradient = Vector2D(0, 0)

        dxi = xi - xim1
        dxip1 = xip1 - xi

        absDxi = dxi.length()
        absDxip1  = dxip1.length()

        # ensure absolute values are not null
        if absDxi > 0 and absDxip1 > 0:
            # angular change at the node
            dphi = math.acos(Helper.clamp(dxi.dot(dxip1) / (absDxi * absDxip1), -1, 1))
            kappa = dphi / absDxi

            if kappa <= self.kappaMax:
                return Vector2D(0, 0)
            else:
                absDxi1Inv = 1 / absDxi
                PDphi_PCosDPhi = -1 / math.sqrt(1 - math.pow(math.cos(dphi), 2))
                u = - absDxi1Inv * PDphi_PCosDPhi

                # calculate p1 and p2 terms
                p1 = xi.ort(-xip1) / (absDxi * absDxip1)
                p2 = -xip1.ort(xi) / (absDxi * absDxip1)

                # calculate last erms
                s = dphi / (absDxi * absDxi)
                ones = Vector2D(1, 1)
                ki = u * (-p1 - p2) - (s * ones)

                kim1 = u * p2 - (s * ones)
                kip1 = u * p1

                gradient = self.wCurvature * (0.25 * kim1 + 0.5 * ki + 0.25 * kip1)

                if math.isnan(gradient.x) or math.isnan(gradient.y):
                    rospy.loginfo("nan values in curvature term")
                    return Vector2D(0, 0)
                
                return gradient
        else:
            return Vector2D(0, 0)

    def smoothnessTerm(self, xim2, xim1, xi, xip1, xip2):
        return self.wSmoothness * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2)

    def tracePath(self, path):
        self.path = path
        self.smoothPath()

    def getPath(self):
        return self.path

