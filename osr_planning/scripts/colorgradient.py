

class ColorPoint():
    def __init__(self, r, g, b, val):
        self.r = r
        self.g = g
        self.b = b
        self.val = val

class ColorGradient():
    def __init__(self):
        self.color = []
        self.createDefaultHeatMapGradient()

    def createDefaultHeatMapGradient(self):
        del self.color[:]

        self.color.append(ColorPoint(0, 0, 1,   0.))    #      // Blue.
        self.color.append(ColorPoint(0, 1, 1,   0.25))  #     // Cyan.
        self.color.append(ColorPoint(0, 1, 0,   0.5))   #      // Green.
        self.color.append(ColorPoint(1, 1, 0,   0.75))  #     // Yellow.
        self.color.append(ColorPoint(1, 0, 0,   1.))    #      // Red.


    def getColorAtValue(self, value):
        if len(self.color) == 0:
            return 0, 0, 0
        
        for i in range(len(self.color)):
            currC = self.color[i]

            if value < currC.val:
                prevC = self.color[max(0, i - 1)]

                valueDiff = prevC.val - currC.value
                fractBetween = 0.

                if valueDiff != 0.:
                    fractBetween = (value - currC.val) / valueDiff

                red = (prevC.r - currC.r) * fractBetween + currC.r
                green = (prevC.g - currC.g) * fractBetween + currC.g
                blue = (prevC.b - currC.b) * fractBetween + currC.b

                return red, green, blue

        
        last = self.color[len(self.color) - 1]
        return last.r, last.g, last.b