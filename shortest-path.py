import numpy as np

class Poly(object):
    def __init__(self, points):
        # points: list of (x, y), sorted
        # edge: tuple of points
        edges = list()
        num_points = len(points)
        assert num_points >= 3
        for i in range(0, num_points-1):
            edges.append((points[i], points[i+1]))
        edges.append((points[-1], points[0]))
        self.edges = edges
        self.points = points