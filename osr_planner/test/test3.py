import matplotlib.pyplot as plt
import math
import numpy as np

class CG():

    def solve(self):
        x = np.array([[2., 1.]]).T # initial guess
        print(str(x))
        a = np.array([[4., 1.],[1., 3.]]) 
        print(str(a))
        b = np.array([[1., 2.]]).T # answer
        print(str(b))

        r = b - a.dot(x)        
        p = r

        alpha = np.transpose(r).dot(r) / (np.transpose(p).dot(a.dot(p)))

        x_new = x + alpha * p

        r1 = r - alpha * a.dot(p)

        beta = (np.transpose(r1).dot(r1)) / (np.transpose(r).dot(r))

        p1 = r1 + beta * p

        alpha1 = (np.transpose(r1).dot(r1)) / (np.transpose(p1).dot(a.dot(p1)))

        x_new2 = x_new + alpha1 * p1

        print(str(x_new2))



    def solvePreCondition(self):
        x = np.array([[2., 1.]]).T # initial guess
        print(str(x))
        a = np.array([[4., 1.],[1., 3.]]) 
        print(str(a))
        b = np.array([[1., 2.]]).T # answer
        print(str(b))

        







if __name__ == '__main__':
    cg = CG()

    cg.solve()