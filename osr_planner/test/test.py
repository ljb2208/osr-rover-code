import matplotlib.pyplot as plt
import math
import numpy as np


class Point():
    def __init__(self, x, y, t, obstX, obstY, obstDist, vX, vY, vDist):
        self.x = float(x)
        self.origX = self.x
        self.y = float(y)
        self.origY = self.y        
        self.t = float(t)
        self.obstX = float(obstX)
        self.obstY = float(obstY)
        self.obstDist = float(obstDist)
        self.vX = float(vX)
        self.vY = float(vY)
        self.vDist = float(vDist)
    
    def __repr__(self):
        return "x: " + str(self.x) + " y: " + str(self.y) + " t: " + str(self.t) + " obstX:" + str(self.obstX) + " obstY:" + str(self.obstY) + " obstD: " + str(self.obstDist) + " vX:" + str(self.vX) + " vy:" + str(self.vY) + " vD:" + str(self.vDist)

    def getVec(self):
        return Vector2D(self.x, self.y)

class Vector2D():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __neg__(self):
        return Vector2D(-self.x, -self.y)

    def subtract(self, val):
        return Vector2D(self.x - val, self.y - val)
    
    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)

    def __rtruediv__(self, k):
        return self.__truediv__(k)

    def __truediv__(self, k):
        if isinstance(k, self.__class__):
            return Vector2D(self.x/k.x, self.y/k.y)
        else:
            return Vector2D(self.x/k, self.y/k)

    def __mul__(self, k):
        if isinstance(k, self.__class__):
            return Vector2D(self.x*k.x, self.y*k.y)
        else:
            return Vector2D(self.x*k, self.y*k)

    def __rmul__(self, k):
        return self.__mul__(k)

    def __repr__(self):
        return "X: " + str(self.x) + " Y: " + str(self.y)

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def sqlength(self):
        return self.x * self.x + self.y * self.y

    def length(self):
        return math.sqrt(math.pow(self.x, 2) + math.pow(self.y, 2))

    def ort(self, other):
        a = Vector2D(self.x, self.y)
        c = a - other * a.dot(other) / other.sqlength()
        return c

    def abs(self):
        return Vector2D(abs(self.x), abs(self.y))




class Smoother():
    def __init__(self):
        self.loadData()

    def loadData(self):
        self.points = []

        f = open("/home/lbarnett/catkin_ws/src/osr-rover-code/osr_planner/test/presmooth.csv", "r")

        for line in f:
            data = line.split(",")

            if (data[0] == "x"):
                continue
            pt = Point(data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8])
            self.points.append(pt)

    def smooth2(self):
        tolerance = 0.000001
        weight_data = 0.0
        weight_smooth = 0.0
        weight_obs = 0.0
        weight_vor = 0.0
        weight_curve = 0.2

        dMax = 10
        maxIterations = 500
        iterations = 0

        change = tolerance        

        # self.calcVoronoi(0, 10)
        # # self.calcVoronoi(10, 0)
        # self.calcVoronoi(0, 5)
        # self.calcVoronoi(5, 5)
        # self.calcVoronoi(2, 5)
        # self.calcVoronoi(2, 10)


        while change >= tolerance and iterations < maxIterations:
            change = 0.0

            for i in range(1, len(self.points) -1):
                xpre = self.points[i].x
                ypre = self.points[i].y

                # self.getCurve(i)

                #path length
                self.points[i].x += weight_data * (self.points[i].origX - self.points[i].x)
                self.points[i].y += weight_data * (self.points[i].origY - self.points[i].y)

                #smoothing
                self.points[i].x += weight_smooth * (self.points[i-1].x + self.points[i+1].x - (2.0 * self.points[i].x))
                self.points[i].y += weight_smooth * (self.points[i-1].y + self.points[i+1].y - (2.0 * self.points[i].y))

                #obstacle avoidance
                obsV = Vector2D(self.points[i].x - self.points[i].obstX, self.points[i].y - self.points[i].obstY)                

                if (obsV.length() < dMax):
                    pt = 2 * obsV * (obsV.length() - dMax) / obsV.length()
                    self.points[i].x -= weight_obs * pt.x
                    self.points[i].y -= weight_obs * pt.y                

                if (obsV.length() < dMax):
                    pt = self.getVoronoiTerm2(self.points[i])
                    self.points[i].x -= weight_vor * pt.x
                    self.points[i].y -= weight_vor * pt.y

                #curvature
                curve = self.getCurvatureTerm(self.points[i], self.points[i-1], self.points[i+1], weight_curve)

                self.points[i].x -= curve.x
                self.points[i].y -= curve.y

                change += abs(xpre - self.points[i].x)
                change += abs(ypre - self.points[i].y)

            iterations += 1
                
        
        self.calcMaxTurningAngle()
        self.plot()

    def calcMaxTurningAngle(self):

        for i in range(1, len(self.points) -1):
            dy = self.points[i].y - self.points[i-1].y
            dx = self.points[i].x - self.points[i-1].x
            dya = self.points[i+1].y - self.points[i].y
            dxa = self.points[i+1].x - self.points[i].x

            angle = math.atan(dya/dxa) - math.atan(dy/dx)
            dangle = math.degrees(angle)

            if (abs(dangle) > 50):
                x = 0

            dxi = Vector2D(self.points[i].x - self.points[i-1].x, self.points[i].y - self.points[i-1].y)
            dxia = Vector2D(self.points[i+1].x - self.points[i].x, self.points[i+1].y - self.points[i].y)

            dxit = Vector2D(dxi.y, dxi.x)

            # val = (dxi.dot(dxia)) / (dxi.length() * dxia.length())

            # nangle = math.acos(val)

            vdxi = np.array([dxi.x, dxi.y])[np.newaxis]
            vdxia = np.array([dxia.x, dxia.y])[np.newaxis]

            
            vdxit = vdxi.T

            test = np.linalg.norm(vdxia)
            test2 = dxia.length()

            val2 = vdxit * vdxia / (np.linalg.norm(vdxia) * np.linalg.norm(vdxi))
            val3 = np.arccos(val2)

            test5 = np.linalg.norm(val2)
            test6 = (np.linalg.norm(vdxia) * np.linalg.norm(vdxi))

            test3 = np.trace(val2)
            test4 = math.degrees(test3)


            print("angle: " + str(angle) + " degrees: " + str(dangle))#  + " val2: " + str(val2) + " val3: " + str(val3))

            p1 = dxi.ort(-dxia)/(dxi.length() * dxia.length())
            p2 = -dxia.ort(dxi)/(dxi.length() * dxia.length())
            p3 = -p1 - p2

            #print("orth p1: " + str(p1) + " p2: " + str(p2) + " p3: " + str(p3))


    def calcVoronoi(self, do, dv):
        alpha = 10.
        dMax = 10.      

        if (do > dMax):
            print("nothing to calc")
            return

        test = alpha/(alpha + do)
        test1 =  (dv / (do + dv))
        test2 = math.pow(do - dMax, 2) / math.pow(dMax, 2)
        
        pv = test1 * test2 * test

        print("Distance to obst: " + str(do) + " to vor: " + str(dv) + " pv: " + str(pv))

    def getVoronoiTerm2(self, point):
        alpha = 50.
        
        dMax = 10.

        xioi = Vector2D(point.x - point.obstX, point.y - point.obstY)        
        xivi = Vector2D(point.x - point.vX, point.y - point.vY)

        omega = xioi.length()
        xioi_len = xioi.length()
        xivi_len = xivi.length()

        do_dxi = xioi / xioi_len
        dv_dxi = xivi / xivi_len
        dpv_dv = alpha / (alpha + xioi_len) * math.pow(xioi_len - dMax, 2)/math.pow(dMax, 2) * xioi_len/math.pow(xioi_len + xivi_len, 2)
        dpv_do = alpha / (alpha + xioi_len) * xivi_len / (xioi_len + xivi_len) * (xioi_len - dMax) / math.pow(dMax, 2)
        
        term2 = -(xioi_len - dMax) / (alpha+xioi_len) - (xioi_len - dMax) / (xioi_len + xivi_len) + 2

        top = alpha * xivi_len * ((xivi_len + 2 * dMax + alpha) * xioi_len + (dMax + 2 * alpha) * xivi_len + alpha * dMax)
        bottom = math.pow(dMax, 2) * math.pow(xioi_len + alpha, 2) * math.pow(xioi_len + xivi_len, 2)

        newterm = top/bottom

        dpv_do = dpv_do * term2
                
        # dpv_do = alpha / (alpha + xioi.length()) * xivi.length()/ (xioi.length() + xivi.length()) * math.pow(xioi.length() - dMax, 2) / math.pow(dMax, 2)
        # dpv_do = dpv_do * (-(omega - dMax)/(alpha + omega) - (omega - dMax) / ())

        term = dpv_do*do_dxi + dpv_dv * dv_dxi
        newtermout = newterm * do_dxi + dpv_dv * dv_dxi
        
        return -newtermout


                

    def getCurve(self, i):
        xi = Vector2D(self.points[i].x, self.points[i].y)
        xia = Vector2D(self.points[i+1].x, self.points[i+1].y)
        xib = Vector2D(self.points[i-1].x, self.points[i-1].y)

        xi_xia = xi - xia
        xia_xib = xia - xib
        xib_xi = xib - xi

        a = xi_xia.length()
        b = xia_xib.length()
        c = xib_xi.length()

        s = 0.5 * (a+b+c)
        area = math.sqrt(s*(s-a)*(s-b)*(s-c))

        curve = 4.0 * area / a * b * c
        deg = math.degrees(curve)

        print("curve: " + str(curve) + " degrees: " + str(deg))
    
    def smooth(self):        
        maxIterations = 1000
        wObstacle = 0.0
        wSmoothness = 0.0
        wCurvature = 0.2
        wVoronoi = 0.0

        pointCount = len(self.points)
        iteration = 0

        tolerance = 0.0001
        change = tolerance

        while change >= tolerance and iteration < maxIterations:

            change = 0.0

            for i in range(1, pointCount - 1):
                gradient = self.getObstacleTerm(self.points[i], wObstacle)
                gradient += self.getVoronoiTerm(self.points[i], wVoronoi)

                chgya = self.points[i+1].y - self.points[i].y
                chgy = self.points[i].y - self.points[i-1].y

                chgxa = self.points[i+1].x - self.points[i].x
                chgx = self.points[i].x - self.points[i-1].x

                chta = abs(math.atan(chgya/chgxa) - math.atan(chgy/chgx))


                # if (i > 0 and i < (pointCount - 1)):
                gradient += self.getSmoothTerm(self.points[i], i, wSmoothness)
                    # gradient += self.getCurvatureTerm2(self.points[i], self.points[i-1], self.points[i+1], wCurvature)

                change += gradient.length()                
                self.points[i].x += gradient.x
                self.points[i].y += gradient.y
            
            iteration += 1

        self.plot()
    


    def getSmoothTerm(self, point, i, wSmoothness):        

        
        dVec = self.points[i-1].getVec() + self.points[i+1].getVec()
        dVec += point.getVec() * -2.0

        gradient = wSmoothness * dVec
        return gradient

        

    def getObstacleTerm(self, point, wObstacle):
        obstDMax = 5        
        gradient = Vector2D(0.0, 0.0)

        obsVct = Vector2D(point.x - point.obstX, point.y - point.obstY)        
        obsVctLen = obsVct.length()

        if (obsVctLen < obstDMax):
            gradient = wObstacle * 2. * (obstDMax - obsVctLen) * obsVct / obsVctLen

        return gradient

    def getVoronoiTerm(self, point, wVoronoi):

        alpha = 0.1
        dMax = 5

        gradient = Vector2D(0., 0.)

        vXO = Vector2D(point.x, point.y) - Vector2D(point.obstX, point.obstY)
        absXO = vXO.length()

        if (absXO > dMax):
            return gradient

        do_dxi = vXO / absXO

        vVO = Vector2D(point.x, point.y) - Vector2D(point.vX, point.vY)
        absVO = vVO.length()

        dv_dxi = vVO / absVO

        dpv_ddv = alpha / (alpha + absXO) * math.pow(absXO - dMax, 2) / math.pow(dMax, 2) * (absXO/(math.pow(absXO + absVO, 2)))
       
        obstVec = Vector2D(point.x - point.obstX, point.y - point.obstY)
        obstVecLen = obstVec.length()

        vVec = Vector2D(point.x - point.vX, point.y - point.vY)
        vVecLen = vVec.length()

        t1 = alpha / (alpha + obstVecLen)
        t2 = obstVecLen / (obstVecLen + vVecLen)
        t3 = (obstVecLen - dMax) / math.pow(dMax, 2)
        t4_1 = -(obstVecLen - dMax)/(alpha + obstVecLen)
        t4_2 = (obstVecLen - dMax) / (obstVecLen + vVecLen)

        dpv_ddo = t1 * t2 * t3 * (t4_1 - t4_2 + 2.)

        final = dpv_ddo * do_dxi + dpv_ddv * dv_dxi

        gradient = final * wVoronoi

        return gradient

    def getCurvatureTerm2(self, point, pointb, pointa, wCurvature):
        v = point.getVec()
        va = pointa.getVec()
        vb = pointb.getVec()

        gradient = Vector2D(0., 0.)

        kMax = 45

        dxi = v - vb
        dxia = va - v

        dTheta = abs(math.acos(dxi.dot(dxia)/(dxi.length() * dxia.length())))

        absDxi = dxi.length()
        dxiabs = dxi.abs()
        test1 = (2 * dTheta * (kMax * dxi.length() - dTheta))
        test2 = dxi * dxi * dxi
        gradient = (2 * dTheta * ((kMax * dxi.length()) - dTheta)) / (dxi * dxi * dxi)
        gradient = wCurvature * gradient

        return gradient

    def getCurvatureTerm(self, point, pointb, pointa, wCurvature):
        v = point.getVec()
        va = pointa.getVec()
        vb = pointb.getVec()

        p1 = v.ort(-va) / (v.length() * va.length())
        p2 = -va.ort(v) / (v.length() * va.length())
        p3 = -p1 - p2

        dxi = v - vb
        dxia = va - v

        kappaMax = 1./ 2.5

        dPhi = math.atan(dxia.y/dxia.x) - math.atan(dxi.y/dxi.x)
        # dPhi = math.acos(dxi.dot(dxia)) / (dxi.length() * dxia.length())        
        # kappa = dPhi / dxi.length()
        kappa = abs(dPhi)

        if (kappa < kappaMax):
            return Vector2D(0., 0.)


        absDxi = dxi.length()        
        absDxia = dxia.length()
        invAbsDxi = 1. / absDxi

        ones = Vector2D(1., 1.)        

        vdxi = np.array([dxi.x, dxi.y])[np.newaxis]
        vdxia = np.array([dxia.x, dxia.y])[np.newaxis]

        chgangle = (np.transpose(vdxi) * vdxia) / (absDxi * absDxia)
        chgangle = np.arccos(chgangle)


        #pDPhi_pDCosDPhi = -1. / math.sqrt((1 - math.pow(math.cos(dPhi), 2)))
        pDPhi_pDCosDPhi = -1. / np.sqrt((1 - np.power(np.cos(chgangle), 2)))

        dxixi = dxi / absDxi
        dxixib = dxi / absDxi

        kXi = -invAbsDxi * pDPhi_pDCosDPhi * p3 - dPhi / (dxi * dxi) * ones
        kXip = -invAbsDxi * pDPhi_pDCosDPhi * p2 - dPhi / (dxi * dxi) * ones
        kXia = -invAbsDxi * pDPhi_pDCosDPhi * p1

        gradient = wCurvature * (kXi + kXip + kXia)

        return gradient

    def getCurvatureTerm3(self, point, pointb, pointa, wCurvature):
        v = point.getVec()
        va = pointa.getVec()
        vb = pointb.getVec()
        kMax = 0.1

        p1 = v.ort(-va) / (v.abs() * va.abs())
        p2 = -va.ort(v) / (v.abs() * va.abs())
        p3 = -p1 - p2

        dxi = v - vb
        dxia = va - v

        absDxi = dxi.length()        
        absDxia = dxia.length()
        invAbsDxi = 1. / absDxi

        ones = Vector2D(1., 1.)

        dPhi = math.acos(dxi.dot(dxia) / (absDxi * absDxia))

        pDPhi_pDCosDPhi = -1. / math.sqrt((1 - math.pow(math.cos(dPhi), 2)))

        dxixi = dxi / absDxi
        dxixib = dxi / absDxi

        kXi = -invAbsDxi * pDPhi_pDCosDPhi * p3 - dPhi / (dxi * dxi) * ones
        kXip = -invAbsDxi * pDPhi_pDCosDPhi * p2 - dPhi / (dxi * dxi) * ones
        kXia = -invAbsDxi * pDPhi_pDCosDPhi * p1

        kX = kXi + kXip + kXia

        kval = 2 * kX(kMax * absDxi - kX) / math.pow(absDxi, 3)

        gradient = wCurvature * (kval)

        return gradient

    


    def plot(self):
        x = []
        y = []
        x_orig = []
        y_orig = []

        x_min = 1000
        x_max = 0

        y_min = 1000
        y_max = 0

        for pt in self.points:
            x.append(pt.x)
            y.append(pt.y)
            x_orig.append(pt.origX)
            y_orig.append(pt.origY)

            if pt.x > x_max:
                x_max = pt.x
            
            if pt.x < x_min:
                x_min = pt.x

            if pt.y > y_max:
                y_max = pt.y
            
            if pt.y < y_min:
                y_min = pt.y

        x_min -= 10
        x_max += 10

        y_min -= 10
        y_max += 10

        rMin = min(x_min, y_min)
        rMax = max(x_max, y_max)
        

        
        plt.plot(x, y, x_orig, y_orig)
        plt.axis([rMin, rMax, rMin, rMax])
        plt.show()


if __name__ == '__main__':
    smoother= Smoother()    

    smoother.smooth2()
    print("done")
