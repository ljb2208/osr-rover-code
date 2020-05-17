
import math

EPSILON = 10e-10

LSL = 0
LSR = 1
RSL = 2
RSR = 3
RLR = 4
LRL = 5

L_SEG = 0
S_SEG = 1
R_SEG = 2

EDUBOK = 0 # no error
UDOBCOCONFIGS = 1 # colocated configurations
EDUBPARAM = 2 # Path parameterisation error
EDUBBADRHO = 3 # Rho value is invalid
EDUBNOPATH = 4 # No path between configurations with this word

DIRDATA = [[L_SEG, S_SEG, L_SEG], [L_SEG, S_SEG, R_SEG], [R_SEG, S_SEG, L_SEG], [R_SEG, S_SEG, R_SEG], [R_SEG, L_SEG, R_SEG], [L_SEG, R_SEG, L_SEG]]

class Inputs():
    def __init__(self):
        self.sa = 0.
        self.sb = 0.
        self.ca = 0.
        self.cb = 0.
        self.c_ab = 0.

class Outputs():
    def __init__(self, t, p, q):
        self.t = t
        self.p = p
        self.q = q



class DubinsPath():
    def __init__(self, qi, param, rho, pathType):
        self.qi = qi
        self.param = param
        self.rho = rho
        self.pathType = pathType

class Dubins():
    def __init__(self):
        self.dubinsWords = []
        self.dubinsWords.append(self.dubinsLSL)
        self.dubinsWords.append(self.dubinsLSR)
        self.dubinsWords.append(self.dubinsRSL)
        self.dubinsWords.append(self.dubinsRSR)
        self.dubinsWords.append(self.dubinsRLR)
        self.dubinsWords.append(self.dubinsLRL)

    @staticmethod
    def fmodr(x, y):
        return x - y*math.floor(x/y)

    @staticmethod
    def mod2pi(theta):
        return Dubins.fmodr(theta, 2 * math.pi)

    def dubinsInitNormalised(self, alpha, beta, d, path):
        bestCost = math.inf
        bestWord = -1

        for i in range(6):
            params = [0.] * 3
            err, outParams = self.dubinsWords[i](alpha, beta, d, params)

            if err == EDUBOK:
                cost = outParams.t + outParams.p + outParams.q

                if cost < bestCost:
                    bestWord = i
                    bestCost = cost
                    path.param[0] = outParams.t
                    path.param[1] = outParams.p
                    path.param[2] = outParams.q
                    path.pathType = i

        if bestWord == -1:
            return EDUBNOPATH
        
        path.pathType = bestWord
        return EDUBOK, path

    def dubinsInit(self, q0, q1, rho, path):
        dx = q1[0] - q0[0]
        dy = q1[1] - q0[1]
        D = math.sqrt(dx*dx + dy*dy)
        d = D / rho

        if rho <= 0.:
            return EDUBBADRHO

        theta = Dubins.mod2pi(math.atan2(dy, dx))
        alpha = Dubins.mod2pi(q0[2] - theta)
        beta = Dubins.mod2pi(q1[2] - theta)

        for i in range(3):
            path.qi[i] = q0[i]

        path.rho = rho

        return self.dubinsInitNormalised(alpha, beta, d, path)

    def dubinsPathLength(self, path):
        length = 0.
        length += path.param[0]
        length += path.param[1]
        length += path.param[2]
        length = length * path.rho
        return length

    def dubinsSegment(self, t, qi, qt, pathType):

        if pathType == L_SEG:
            qt[0] = qi[0] + math.sin(qi[2] + t) - math.sin(qi[2])
            qt[1] = qi[1] - math.cos(qi[2] + t) + math.cos(qi[2])
            qt[2] = qi[2] + t
        elif pathType == R_SEG:
            qt[0] = qi[0] - math.sin(qi[2] - t) + math.sin(qi[2])
            qt[1] = qi[1] + math.cos(qi[2] - t) - math.cos(qi[2])
            qt[2] = qi[2] - t
        elif pathType == S_SEG:
            qt[0] = qi[0] + math.cos(qi[2]) * t
            qt[1] = qi[1] + math.sin(qi[2]) * t
            qt[2] = qi[2]
    
        return qt

    def dubinsPathSample(self, path, t, q):
        if t < 0 or t >= self.dubinsPathLength(path):
            return EDUBPARAM, q

        tprime = t / path.rho

        qi = [0, 0, path.qi[2]]

        types = DIRDATA[path.pathType]
        p1 = path.param[0]
        p2 = path.param[1]

        q1 = [0., 0., 0.]
        q2 = [0., 0., 0.]

        q1 = self.dubinsSegment(p1, qi, q1, types[0])
        q2 = self.dubinsSegment(p2, q1, q2, types[1])

        if tprime < p1:
            q = self.dubinsSegment(tprime, qi, q, types[1])
        elif tprime < (p1 + p2):
            q = self.dubinsSegment(tprime - p1, q1, q, types[1])
        else:
            q = self.dubinsSegment(tprime - p1 - p2, q2, q, types[2])

        q[0] = q[0] * path.rho + path.qi[0]
        q[1] = q[1] * path.rho + path.qi[1]
        q[2] = Dubins.mod2pi(q[2])

        return EDUBOK, q

    def dubinsPathSampleMany(self, path, cb, stepSize, userData):
        x = 0.
        length = self.dubinsPathLength(path)

        while x < length:
            q = [0., 0., 0.]
            retVal, q = self.dubinsPathSample(path, x, q)

            retVal = cb(q, x, userData)

            if retVal != 0:
                return retVal

            x += stepSize
        
        return EDUBOK

    def dubinsPathEndPoint(self, path, q):
        return self.dubinsPathSample(path, self.dubinsPathLength(path) - EPSILON, q)

    def dubinsExtractSubPath(self, path, t, newPath):
        tprime = t / path.rho

        newPath.qi[0] = path.qi[0]
        newPath.qi[1] = path.qi[1]
        newPath.qi[2] = path.qi[2]
        newPath.rho = path.rho
        newPath.pathType = path.pathType

        newPath.param[0] = min(path.param[0], tprime)
        newPath.param[1] = min(path.param[1], tprime - newPath.param[0])
        newPath.param[2] = min(path.param[2], tprime - newPath.param[0] - newPath.param[1])

        return EDUBOK, newPath

    def unpackInputs(self, alpha, beta):
        inp = Inputs()
        inp.sa = math.sin(alpha)
        inp.sb = math.sin(beta)
        inp.ca = math.cos(alpha)
        inp.cb = math.cos(beta)
        inp.c_ab = math.cos(alpha - beta)
        return inp

    def dubinsLSL(self, alpha, beta, d, outputs):
        inp = self.unpackInputs(alpha, beta)

        tmp0 = d + inp.sa - inp.sb
        p_squared = 2 + (d*d) - (2*inp.c_ab) + (2*d*(inp.sa - inp.sb))

        if p_squared < 0:
            return EDUBNOPATH, Outputs(0, 0, 0)

        tmp1 = math.atan2((inp.cb - inp.ca), tmp0)
        t = Dubins.mod2pi(-alpha + tmp1)
        p = math.sqrt(p_squared)
        q = Dubins.mod2pi(beta - tmp1)

        out = Outputs(t, p, q)

        return EDUBOK, out

    def dubinsRSR(self, alpha, beta, d, outputs):
        inp = self.unpackInputs(alpha, beta)
        tmp0 = d - inp.sa + inp.sb
        p_squared = 2 + (d*d) - (2*inp.c_ab) + (2*d*(inp.sb - inp.sa))

        if p_squared < 0:
            return EDUBNOPATH, Outputs(0, 0, 0)

        tmp1 = math.atan2((inp.ca - inp.cb), tmp0)
        t = Dubins.mod2pi(alpha - tmp1)
        p = math.sqrt(p_squared)
        q = Dubins.mod2pi(-beta+tmp1)

        out = Outputs(t, p, q)

        return EDUBOK, out

    def dubinsLSR(self, alpha, beta, d, outputs):
        inp = self.unpackInputs(alpha, beta)
        p_squared = -2 + (d*d) + (2*inp.c_ab) + (2*d*(inp.sa + inp.sb))

        if p_squared < 0:
            return EDUBNOPATH, Outputs(0, 0, 0)

        p = math.sqrt(p_squared)
        tmp2 = math.atan2((-inp.ca - inp.cb), (d+inp.sa+inp.sb)) - math.atan2(-2., p)
        t = Dubins.mod2pi(-alpha+tmp2)
        q = Dubins.mod2pi(-Dubins.mod2pi(beta) + tmp2)

        out = Outputs(t, p, q)

        return EDUBOK, out

    def dubinsRSL(self, alpha, beta, d, outputs):
        inp = self.unpackInputs(alpha, beta)

        p_squared = (d*d) -2 + (2*inp.c_ab) - (2*d*(inp.sa + inp.sb))

        if p_squared < 0:
            return EDUBNOPATH, Outputs(0, 0, 0)

        p = math.sqrt(p_squared)
        tmp2 = math.atan2((inp.ca + inp.cb), (d - inp.sa - inp.sb)) - math.atan2(2., p)
        t = Dubins.mod2pi(alpha - tmp2)
        q = Dubins.mod2pi(beta - tmp2)

        out = Outputs(t, p, q)

        return EDUBOK, out

    def dubinsRLR(self, alpha, beta, d, outputs):
        inp = self.unpackInputs(alpha, beta)

        tmp_rlr = (6. - d*d + 2*inp.c_ab + 2*d*(inp.sa - inp.sb)) / 8.

        if abs(tmp_rlr) > 1:
            return EDUBNOPATH, Outputs(0, 0, 0)

        p = Dubins.mod2pi(2*math.pi - math.acos(tmp_rlr))
        t = Dubins.mod2pi(alpha - math.atan2(inp.ca - inp.cb, d-inp.sa+inp.sb) + Dubins.mod2pi(p/2.))
        q = Dubins.mod2pi(alpha - beta - t + Dubins.mod2pi(p))

        out = Outputs(t, p, q)

        return EDUBOK, out

    def dubinsLRL(self, alpha, beta, d, outputs):
        inp = self.unpackInputs(alpha, beta)

        tmp_lrl = (6. - d*d + 2*inp.c_ab + 2*d*(-inp.sa + inp.sb)) / 8.

        if abs(tmp_lrl) > 1:
            return EDUBNOPATH, Outputs(0, 0, 0)

        p = Dubins.mod2pi(2*math.pi - math.acos(tmp_lrl))
        t = Dubins.mod2pi(-alpha - math.atan2(inp.ca - inp.cb, d+inp.sa-inp.sb) + Dubins.mod2pi(p/2.))
        q = Dubins.mod2pi(Dubins.mod2pi(beta) - alpha - t + Dubins.mod2pi(p))
        
        out = Outputs(t, p, q)

        return EDUBOK, out