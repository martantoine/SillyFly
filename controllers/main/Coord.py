import math
import numpy as np

class Coord:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __str__(self):
        return "Point(%s,%s)"%(self.x, self.y) 

    def __add__(self, other):
        return Coord(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return Coord(self.x - other.x, self.y - other.y)
    
    def __div__(self, other):
        return Coord(self.x / other, self.y / other)

    def dist(self, other):
        dx = self.x - other.x
        dy = self.y - other.y
        return math.sqrt(dx**2 + dy**2)

    def angle(self, other):
        angle = np.arctan2((other.y - self.y), (other.x - self.x))
        return angle

    def test():
        '''Returns a point and distance'''
        p1 = Coord(1.0, 1.0)
        print(p1)
        p2 = Coord(1.0,-1.0)
        print(p2)
        print(p1 + p2)
        print(p1 - p2)
        print(np.rad2deg(Coord.angle(p1, p2)))
        print(Coord.dist(p1, p2))