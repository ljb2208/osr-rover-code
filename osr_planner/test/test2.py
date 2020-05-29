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

    def ort2(self, other):
        a = Vector2D(self.x, self.y)
        c = a - other * a.dot(other) / (other.length() * other.length())
        return c

    def abs(self):
        return Vector2D(abs(self.x), abs(self.y))
    

class Smoother():
    def __init__(self):
        self.loadData()
        self.weight_length = 0.0
        self.weight_data = 0.0
        self.weight_smooth = 0.2
        self.weight_obs = 0.0
        self.weight_vor = 0.0
        self.weight_curve = 0.0
        self.change = []

    def zeroWeights(self):
        self.weight_data = 0.0
        self.weight_smooth = 0.0
        self.weight_obs = 0.0
        self.weight_vor = 0.0
        self.weight_curve = 0.0


    def loadData(self):
        self.points = []

        f = open("/home/lbarnett/catkin_ws/src/osr-rover-code/osr_planner/test/presmooth_orig.csv", "r")

        for line in f:
            data = line.split(",")

            if (data[0] == "x"):
                continue
            pt = Point(data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8])
            self.points.insert(0, pt)

    def smooth(self):
        tolerance = 0.000001

        self.change.clear()
        
        dMax = 10
        alpha = 50
        maxIterations = 50
        iterations = 0

        change = tolerance        
        min_change = 100000.
        min_change_iteration = 0

        while change >= tolerance and iterations < maxIterations:
            change = 0.0

            for i in range(1, len(self.points) -1):
                xpre = self.points[i].x
                ypre = self.points[i].y

                xi = Vector2D(self.points[i].x, self.points[i].y)
                xp = Vector2D(self.points[i-1].x, self.points[i-1].y)
                xa = Vector2D(self.points[i+1].x, self.points[i+1].y)

                #length
                lengthTerm = self.getLengthTerm(xi, xp, xa, self.weight_length)

                #smoothing
                smoothTerm = self.getSmoothnessTerm(xi, xp, xa, self.weight_smooth)
                smoothError = self.getSmoothnesResidual(xi, xp, xa, self.weight_smooth)
                self.points[i].x += smoothTerm.x
                self.points[i].y += smoothTerm.y

                #obstacles
                oi = Vector2D(self.points[i].obstX, self.points[i].obstY)
                obstTerm = self.getObstacleTerm(xi, oi, self.weight_obs, dMax)
                self.points[i].x -= obstTerm.x
                self.points[i].y -= obstTerm.y

                # voronoi field
                vi = Vector2D(self.points[i].vX, self.points[i].vY)
                vorTerm = self.getVoronoiTerm(xi, oi, vi, self.weight_vor, dMax, alpha)
                self.points[i].x -= vorTerm.x
                self.points[i].y -= vorTerm.y

                # curvature term
                dPhi = self.getAngleChange(xi, xp, xa)

                if (dPhi > 1.0):
                    dd = 0
                # print("angle before: " + str(dPhi))
                curveTerm = self.getCurvatureTerm4(xi, xp, xa, self.weight_curve)
                curveError = self.getCurvatureResidual(xi, xp, xa, self.weight_curve)
                self.points[i].x += curveTerm.x
                self.points[i].y += curveTerm.y

                xiNew = Vector2D(xi.x + curveTerm.x, xi.y + curveTerm.y)
                dPhi_New = self.getAngleChange(xiNew, xp, xa)
                # print("angle after: " + str(dPhi))

                change += abs(xpre - self.points[i].x)
                change += abs(ypre - self.points[i].y)


            if (change < min_change):
                min_change = change
                min_change_iteration = iterations

            self.change.append(change)

            iterations += 1            


        print ("Iterations: " + str(iterations) + " change: " + str(change) + " min_change: " + str(min_change) + " at iteration " + str(min_change_iteration))
    

    def getSmoothnesResidual(self, xi, xp, xa, wSmoothness):
        r = xa.dot(xa) - 4 * xa.dot(xi) + 2 * xa.dot(xp) + 4 * xi.dot(xi) - 4 * xi.dot(xp) + xp.dot(xp)
        return r

    def getCurvatureResidual(self, xi, xp, xa, wCurvature):        
        epsilon = 0.00001
        kMax = 1. / 20.
        delta_xi = Vector2D(xi.x - xp.x, xi.y - xp.y)
        delta_xi_p = Vector2D(xa.x - xi.x, xa.y - xi.y)

        delta_xi_norm  = delta_xi.length()
        delta_xi_p_norm = delta_xi_p.length()

        delta_xi_by_xi_p = delta_xi_norm * delta_xi_p_norm

        projection = delta_xi.dot(delta_xi_p) / delta_xi_by_xi_p

        delta_phi_i = math.acos(projection)

        turning_rad = delta_phi_i / delta_xi_by_xi_p

        ki_minus_kmax = turning_rad - kMax
        
        if ki_minus_kmax <= epsilon:
            r = 0
        else:
            r = wCurvature * ki_minus_kmax * ki_minus_kmax
        
        return r

    def getLengthTerm(self, xi, xp, xa, wLength):
        term = Vector2D(0., 0.)

        return term * wLength

    def getSmoothnessTerm(self, xi, xp, xa, wSmoothness):
        term = Vector2D(0., 0.)

        term.x += wSmoothness * (xp.x + xa.x - (2.0 * xi.x))
        term.y += wSmoothness * (xp.y + xa.y - (2.0 * xi.y))

        return term

    def getObstacleTerm(self, xi, oi, wObstacle, dMax):
        term = Vector2D(0., 0.)

        xioi = xi - oi
        xioi_len = xioi.length()

        if xioi_len >= dMax:
            return term

        term = wObstacle * 2 * (xioi_len - dMax) * (xioi/ xioi_len)

        return term

    def getVoronoiTerm(self, xi, oi, vi, wVoronoi, dMax, alpha):
        term = Vector2D(0., 0.)

        xioi = xi - oi
        xioi_len = xioi.length()        

        if xioi_len >= dMax:
            return term

        xivi = xi - vi
        xivi_len = xivi.length()

        if xivi_len == 0:
            return term

        do_dxi = xioi / xioi_len
        dv_dxi = xivi / xivi_len

        dpv_dv = alpha / (alpha + xioi_len) * math.pow(xioi_len - dMax, 2)/math.pow(dMax, 2) * xioi_len / math.pow(xioi_len + xivi_len, 2)

        dpv_do = alpha / (alpha + xioi_len) * xivi_len/ (xioi_len + xivi_len) * (xioi_len - dMax) / math.pow(dMax, 2) 
        dpv_do_term2 = -(xioi_len - dMax)/(alpha + xioi_len) - (xioi_len - dMax) / (xioi_len + xivi_len) + 2

        dpv_do_complete = dpv_do * dpv_do_term2

        term = dpv_do_complete * do_dxi + dpv_dv * dv_dxi

        return term * wVoronoi

    def getAngleChange(self, xi, xip, xia):
        xixip = Vector2D(xi.x - xip.x, xi.y - xip.y)
        xiaxi = Vector2D(xia.x - xi.x, xia.y - xi.y)

        fullval = xixip.dot(xiaxi)/(xixip.length() * xiaxi.length())

        if round(fullval, 5) == 1.:
            dPhi = 0.
        else:
            dPhi = math.acos(xixip.dot(xiaxi)/(xixip.length() * xiaxi.length()))

        return dPhi

    def getCurvatureTerm6(self, xi, xp, xa, wCurvature):
        epsilon = 0.00001
        kMax = 1. / 20.
        delta_xi = Vector2D(xi.x - xp.x, xi.y - xp.y)
        delta_xi_p = Vector2D(xa.x - xi.x, xa.y - xi.y)

        delta_xi_norm  = delta_xi.length()
        delta_xi_p_norm = delta_xi_p.length()

        delta_xi_by_xi_p = delta_xi_norm * delta_xi_p_norm

        projection = delta_xi.dot(delta_xi_p) / delta_xi_by_xi_p

        delta_phi_i = math.acos(projection)

        turning_rad = delta_phi_i / delta_xi_by_xi_p

        ki_minus_kmax = turning_rad - kMax
        
        if ki_minus_kmax <= epsilon:
            r = 0
        else:
            r = wCurvature * ki_minus_kmax * ki_minus_kmax

        partial_delta_phi_i_wrt_cost_delta_phi_i = -1 / math.sqrt(1 - math.pow(math.cos(delta_phi_i), 2))
        ones = Vector2D(1., 1.)

        neg_pt_plus = -1 * xa

        p1 = self.normalizedOrthogonalComplement(xi, neg_pt_plus, delta_xi_norm, delta_xi_p_norm)
        p2 = self.normalizedOrthogonalComplement(neg_pt_plus, xi, delta_xi_p_norm, delta_xi_norm)

        u = 2 * ki_minus_kmax
        common_prefix = (1/ delta_xi_norm) * partial_delta_phi_i_wrt_cost_delta_phi_i
        common_suffix = delta_phi_i / (delta_xi_norm * delta_xi_norm)

        d_delta_xi_d_xi = delta_xi / delta_xi_norm

        j = u * (common_prefix * (-p1 - p2) - (common_suffix * d_delta_xi_d_xi))
        j_im1 = u * (common_prefix * p2 + (common_suffix * d_delta_xi_d_xi))
        j_ip1 = u * (common_prefix * p1)

        term = j_im1 + 2 * j + j_ip1

        return term

    def normalizedOrthogonalComplement(self, a, b, a_norm, b_norm):
        return (a - (a.dot(b) * b / b.sqlength())) / (a_norm * b_norm)

    def getCurvatureTerm4(self, xi, xp, xa, wCurvature):
        xpxa = Vector2D(xp.x - xa.x, xp.y - xa.y)
        xpxa = xpxa / 2

        xmid = Vector2D(xp.x - xpxa.x, xp.y - xpxa.y)

        xi_xpxa = Vector2D(xi.x - xmid.x, xi.y - xmid.y)

        term =  -xi_xpxa * wCurvature


        return term

    def getCurvatureTerm3(self, xi, xip, xia, wCurvature):
        kMax = 1. / 20.

        term = Vector2D(0., 0.)

        xixip = Vector2D(xi.x - xip.x, xi.y - xip.y)
        xiaxi = Vector2D(xia.x - xi.x, xia.y - xi.y)

        xixip_len = xixip.length()
        xiaxi_len = xiaxi.length()

        xixip_sqlen = xixip.sqlength()
        xiaxi_sqlen = xiaxi.sqlength()

        dotval = xixip.dot(xiaxi)
        fullval = xixip.dot(xiaxi)/(xixip_len * xiaxi_len)

        if round(fullval, 5) == 1.:
            dPhi = 0.
        else:
            dPhi = math.acos(xixip.dot(xiaxi)/(xixip_len * xiaxi_len))

        if dPhi <= (kMax / xixip_len):
            return term
        
        d1 = (xixip * dPhi) / math.pow(xixip_sqlen, 1.5)
        d2_1 = xiaxi * xiaxi.dot(xixip) / (math.pow(xiaxi_sqlen, 1.5) * xixip_len)
        d2_2 = (-2 * xi + xip + xia) / (xiaxi_len * xixip_len)
        d2_3 = (xixip * xiaxi.dot(xixip)) / (xiaxi.length() * math.pow(xixip_sqlen, 1.5))
        d2_d1 = xixip_len
        d2_d2 = math.sqrt(1 - (math.pow(xiaxi.dot(xixip), 2) / (xiaxi_sqlen * xixip_sqlen)))        
        d3 = (dPhi / xixip_len) - kMax

        term = -d2_1 + d2_2 - d2_3/d2_d2        
        # term *= wCurvature

        xpxa = Vector2D(xp.x - xa.x, xp.y - xa.y)
        xpxa = xpxa / 2

        xmid = Vector2D(xp.x - xpxa.x, xp.y - xpxa.y)

        xi_xpxa = Vector2D(xi.x - xmid.x, xi.y - xmid.y)
        
        xi_xpxa_len = xi_xpxa.length()

        # term = term * xi_xpxa
        term.x = term.x * abs(xi_xpxa.x)
        term.y = term.y * abs(xi_xpxa.y)

        return term

    def getCurvatureTerm2(self, xi, xip, xia, wCurvature):
        kMax = 1. / 20.

        term = Vector2D(0., 0.)

        xixip = Vector2D(xi.x - xip.x, xi.y - xip.y)
        xiaxi = Vector2D(xia.x - xi.x, xia.y - xi.y)

        xixip_len = xixip.length()
        xiaxi_len = xiaxi.length()

        xixip_sqlen = xixip.sqlength()
        xiaxi_sqlen = xiaxi.sqlength()

        dotval = xixip.dot(xiaxi)
        fullval = xixip.dot(xiaxi)/(xixip_len * xiaxi_len)

        if round(fullval, 5) == 1.:
            dPhi = 0.
        else:
            dPhi = math.acos(xixip.dot(xiaxi)/(xixip_len * xiaxi_len))

        if dPhi <= (kMax / xixip_len):
            return term
        
        d1 = (xixip * dPhi) / math.pow(xixip_sqlen, 1.5)
        d2_1 = xiaxi * xiaxi.dot(xixip) / (math.pow(xiaxi_sqlen, 1.5) * xixip_len)
        d2_2 = (-2 * xi + xip + xia) / (xiaxi_len * xixip_len)
        d2_3 = (xixip * xiaxi.dot(xixip)) / (xiaxi.length() * math.pow(xixip_sqlen, 1.5))
        d2_d1 = xixip_len
        d2_d2 = math.sqrt(1 - (math.pow(xiaxi.dot(xixip), 2) / (xiaxi_sqlen * xixip_sqlen)))        
        d3 = (dPhi / xixip_len) - kMax

        tvec = 2 * ((-d1 - (d2_1 + d2_2 - d2_3)/(d2_d1*d2_d2) * d3))

        tnewvec = -d2_1 + d2_2 - d2_3/d2_d2
        tnewvec2 = tnewvec/ d2_d1

        dx1 = (xixip.x * dPhi) / math.pow(xixip_sqlen, 1.5)
        dx2_1 = (xiaxi.x * xiaxi.dot(xixip)) / (math.pow(xiaxi_sqlen, 1.5) * xixip_len)
        # dx2_1 = (xiaxi.x * (xia.x * xi.x) + (xia.y * xi.y)) / (math.pow(xiaxi_sqlen, 1.5) * xixip_len)
        dx2_2 = (-2 * xi.x + xip.x + xia.x) / (xiaxi_len * xixip_len)
        # dx2_2 = xia.x / (xiaxi_len * xixip_len)
        dx2_3 = (xixip.x * xiaxi.dot(xixip)) / (xiaxi.length() * math.pow(xixip_sqlen, 1.5))
        # dx2_3 = (xiaxi.x * (xia.x*xi.x + xia.y * xi.y))/ (xiaxi_len * math.pow(xixip_sqlen, 1.5))

        dx2_d1 = xixip_len

        test1 = math.pow(xiaxi.dot(xixip), 2)
        test2 = xiaxi_sqlen * xixip_sqlen
        test3 = test1/test2

        dx2_d2 = math.sqrt(1 - (math.pow(xiaxi.dot(xixip), 2) / (xiaxi_sqlen * xixip_sqlen)))
        # dx2_d2 = math.sqrt(1 - (math.pow(xia.x*xi.x + xia.y+xi.y, 2)/(xiaxi_sqlen * xixip_sqlen)))

        dx3 = (dPhi / xixip_len) - kMax

        tx = 2 * (-dx1 - (dx2_1 + dx2_2 - dx2_3)/(dx2_d1*dx2_d2) * dx3)

        dy1 = (xixip.y * dPhi) / math.pow(xixip_sqlen, 1.5)
        dy2_1 = (xiaxi.y * xiaxi.dot(xixip)) / (math.pow(xiaxi_sqlen, 1.5) * xixip_len)
        # dy2_1 = (xiaxi.y * (xia.x * xi.x) + (xia.y * xi.y)) / (math.pow(xiaxi_sqlen, 1.5) * xixip_len)
        # dy2_2 = xia.y / (xiaxi_len * xixip_len)
        dy2_2 = (-2 * xi.y + xip.y + xia.y) / (xiaxi_len * xixip_len)
        dy2_3 = (xixip.y * xiaxi.dot(xixip)) / (xiaxi.length() * math.pow(xixip_sqlen, 1.5))
        # dy2_3 = (xiaxi.y * (xia.x*xi.x + xia.y * xi.y))/ (xiaxi_len * math.pow(xixip_sqlen, 1.5))

        dy2_d1 = xixip_len
        dy2_d2 = math.sqrt(1 - (math.pow(xiaxi.dot(xixip), 2) / (xiaxi_sqlen * xixip_sqlen)))
        # dy2_d2 = math.sqrt(1 - (math.pow(xia.x*xi.x + xia.y+xi.y, 2)/(xiaxi_sqlen * xixip_sqlen)))

        dy3 = (dPhi / xixip_len) - kMax

        ty = 2 * (-dy1 - (dy2_1 + dy2_2 - dy2_3)/(dy2_d1*dy2_d2) * dy3)

        term.x = tx
        term.y = ty

        return term * wCurvature


    
    def getCurvatureTerm(self, xi, xip, xia, wCurvature):

        kMax = 1. / 20.
        # kMax = 0.785398163397448/10. # 45 degrees per meter adjusted for 0.05 cell size
        # vKMax = Vector2D(math.cos(kMax), math.sin(kMax))


        term = Vector2D(0., 0.)

        xixip = Vector2D(xi.x - xip.x, xi.y - xip.y)
        xiaxi = Vector2D(xia.x - xi.x, xia.y - xi.y)

        xixip_len = xixip.length()
        xiaxi_len = xiaxi.length()

        dotval = xixip.dot(xiaxi)
        fullval = xixip.dot(xiaxi)/(xixip_len * xiaxi_len)

        if round(fullval, 5) == 1.:
            dPhi = 0.
        else:
            dPhi = math.acos(xixip.dot(xiaxi)/(xixip_len * xiaxi_len))

        # if dPhi > (math.pi/2):
        #     dPhi = math.pi - dPhi

        # if dPhi < kMax or dPhi > (math.pi - kMax):
        #     return term

        if dPhi <= (kMax / xixip_len):
            return term
        
        p1 = xi.ort(-xia)/(xi.length() * xia.length())
        p2 = -xia.ort(xi)/(xi.length() * xia.length())
        p3 = -p1 - p2

        pDPhi_pDCosDPhi = -1. / math.sqrt((1 - math.pow(math.cos(dPhi), 2)))

        absXiXipInv = -1./xixip_len

        ones = Vector2D(1., 1.)

        test1 = absXiXipInv * pDPhi_pDCosDPhi
        test2 = dPhi/(xixip * xixip)

        kXi = absXiXipInv * pDPhi_pDCosDPhi * p3 - dPhi/(xixip * xixip) * ones
        kXip = absXiXipInv * pDPhi_pDCosDPhi * p2 - dPhi/(xixip * xixip) * -ones
        kXia = absXiXipInv * pDPhi_pDCosDPhi * p1

        kTotal = -2 * kXi + kXip + kXia            
        # kNew = kTotal - vKMax
        # term = wCurvature * kNew

        kTotal.x -= kMax / xixip_len
        kTotal.y -= kMax / xixip_len
        kTotal = 2 * kTotal

        new = Vector2D(kXi.x, kXi.y)

        term = wCurvature * kXi

        # print ("dPhi: " + str(dPhi) + " term: " + str(term))

        return term        


    def plotChange(self):
        iterations = []
        change_values = []
        i = 0
        for chg in self.change:
            iterations.append(i)
            change_values.append(chg)
            i += 1

        plt.plot(iterations, change_values)
        plt.show()

    def plotAngles(self):
        theta_angles = []
        phi_angles = []
        index = []
                
        for i in range(1, len(self.points) - 1):
            xi = Vector2D(self.points[i].x, self.points[i].y)
            xp = Vector2D(self.points[i-1].x, self.points[i-1].y)
            xa = Vector2D(self.points[i+1].x, self.points[i+1].y)
            dPhi = self.getAngleChange(xi, xp, xa)
            
            phi_angles.append(math.degrees(dPhi))

            dTheta = self.points[i].t - self.points[i-1].t

            theta_angles.append(math.degrees(dTheta))
            index.append(i)

            if dPhi != 0. or dTheta != 0.:
                xx = 0


        line1, line2 = plt.plot(index, phi_angles, index, theta_angles)
        line1.set_label("dPhi")
        line2.set_label("dTheta")
        plt.show()


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
        

        
        line1, line2 = plt.plot(x, y, x_orig, y_orig)
        line1.set_label("smoothed")
        line2.set_label("orig")
        plt.legend()
        plt.axis([rMin, rMax, rMin, rMax])
        plt.show()    

    

    def checkAngles(self):
        for i in range(1, len(self.points) - 1):
            xi = Vector2D(self.points[i].x, self.points[i].y)
            xp = Vector2D(self.points[i-1].x, self.points[i-1].y)
            xa = Vector2D(self.points[i+1].x, self.points[i+1].y)

            xixp = xi - xp
            xaxi = xa - xi

            test1 = xi.ort(-xa)
            test2 = -xa.ort(xi)

            test3 = test1 / (xi.length() * xa.length())
            test4 = test2 / (xi.length() * xa.length())

            test5 = xi.ort2(-xa)
            test6 = -xa.ort2(xi)

            t = math.atan(xixp.y/xixp.x)
            ta = math.atan(xaxi.y/xaxi.x)

            chgt = abs(ta - t)

            dp = xixp.dot(xaxi)
            ln = xixp.length() * xaxi.length()

            dpdlen = round(dp/ln, 6)
            
            altt = math.acos(dpdlen)

            if altt > math.pi/2:
                altt = math.pi - altt

            if (round(altt, 2) != round(chgt, 2)):
                print("diff")

            print("angle: " + str(math.degrees(chgt)))
            # print("chgt: " + str(chgt) + " altt: " + str(altt))
            


if __name__ == '__main__':
    smoother= Smoother() 

    # smoother.checkAngles()   
    # smoother.plotAngles()

    smoother.smooth()
    smoother.plot()
    smoother.plotChange()

    # smoother.checkAngles()   


    xi = Vector2D(2.0, 2.0)
    xp = Vector2D(1.0, 1.0)
    xa = Vector2D(1.0, 3.0)

    # term1 = smoother.getCurvatureTerm(xi,xp, xa, 1.)
    term1 = smoother.getCurvatureTerm4(xi,xp, xa, 1.)
    termn1 = smoother.getCurvatureTerm6(xi,xp, xa, 1.)
    terma1 = smoother.getCurvatureTerm2(xi,xp, xa, 1.)
    # terms1 = smoother.getSmoothnessTerm(xi, xp, xa, 1.)        

    xi = Vector2D(2.0, 1.0)
    xp = Vector2D(1.0, 1.0)
    xa = Vector2D(2.0, 2.0)
    
    # term1 = smoother.getCurvatureTerm(xi,xp, xa, 1.)
    term2 = smoother.getCurvatureTerm4(xi,xp, xa, 1.)
    termn2 = smoother.getCurvatureTerm6(xi,xp, xa, 1.)
    terma2 = smoother.getCurvatureTerm2(xi,xp, xa, 1.)
    # terms2 = smoother.getSmoothnessTerm(xi, xp, xa, 1.)

    print(str(term2))


    xi = Vector2D(200.589, 193.351)
    xp = Vector2D(200.577, 192.894)
    xa = Vector2D(200.203, 192.759)
    
    # term1 = smoother.getCurvatureTerm(xi,xp, xa, 1.)
    term3 = smoother.getCurvatureTerm4(xi,xp, xa, 1.)
    termn3 = smoother.getCurvatureTerm6(xi,xp, xa, 1.)
    terma3 = smoother.getCurvatureTerm2(xi,xp, xa, 1.)
    # terms3 = smoother.getSmoothnessTerm(xi, xp, xa, 1.)
    print(str(term3))

    q = 0

    # smoother.checkAngles()

    # smoother.weight_smooth = 0.2
    # smoother.weight_vor = 0.2
    # smoother.weight_obs = 0.2
    # smoother.smooth()
    # smoother.plot()

    # orig_points = smoother.points


    # orig_x = []
    # orig_y = []     
    # vor_y = []

    # total_x = []
    # total_y = []

    # for i in range(0, len(orig_points)):
    #     orig_x.append(orig_points[i].x)
    #     orig_y.append(orig_points[i].y)

    # smoother.zeroWeights()
    # smoother.weight_smooth = 0.2
    # smoother.smooth()

    # for i in range(0, len(orig_points)):
    #     smooth_x.append(smoother.points[i].x)
    #     smooth_y.append(smoother.points[i].y)

    # smooth_points = smoother.points
    
    # smoother.points = orig_points
    # smoother.zeroWeights()
    # smoother.weight_obs = 0.2
    # smoother.smooth()

    # for i in range(0, len(orig_points)):
    #     obs_x.append(smoother.points[i].x)
    #     obs_y.append(smoother.points[i].y)

    # smoother.points = orig_points
    # smoother.zeroWeights()
    # smoother.weight_vor = 0.2
    # smoother.smooth()

    # for i in range(0, len(orig_points)):
    #     vor_x.append(smoother.points[i].x)
    #     vor_y.append(smoother.points[i].y)

    # smoother.points = orig_points
    # smoother.zeroWeights()
    # smoother.weight_smooth = 0.2
    # smoother.weight_vor = 0.2
    # smoother.weight_obs = 0.2
    # smoother.smooth()


    # for i in range(0, len(orig_points)):
    #     total_x.append(smoother.points[i].x)
    #     total_y.append(smoother.points[i].y)

    # rMin = min(orig_x)
    # rMax = max(orig_x)

    # rMin = min(min(orig_y), rMin)
    # rMax = max(max(orig_y), rMax)

    # rMin -= 10
    # rMax += 10

    # line1, line2, line3, line4, line5 = plt.plot(orig_x, orig_y, total_x, total_y, smooth_x, smooth_y, obs_x, obs_y, vor_x, vor_y)
    # line1.set_label("original")
    # line2.set_label("total")
    # line3.set_label("smooth")
    # line4.set_label("obs")
    # line5.set_label("vor")
    # plt.axis([rMin, rMax, rMin, rMax])
    # plt.legend()
    # plt.show()
    


    # smoother.smooth()
    # smoother.plot()
    # smoother.plotChange()

