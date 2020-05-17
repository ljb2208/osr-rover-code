import math

class Vector2D():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __mul__(self, k):
        return Vector2D(self.x * k, self.y * k)

    def __div__(self, k):
        return Vector2D(self.x / k, self.y / k)

    def __sub__(self, b):
        return Vector2D(self.x - b.x, self.y - b.y)

    def __add__(self, b):
        return Vector2D(self.x + b.x, self.y + b.y)

    def __neg__(self):
        return Vector2D(-self.x, -self.y)

    def length(self):
        return math.sqrt(math.pow(self.x, 2) + math.pow(self.y, 2))

    def sqlength(self):
        return math.pow(self.x, 2) + math.pow(self.y, 2)

    def dot(self, vec):
        return self.x * vec.x + self.y * vec.y

    # orthogonal complement of two vectors
    def ort(self, vec):
        a = Vector2D (self.x, self.y)
        c = a - vec * a.dot(vec) / vec.sqlength()
        return c


