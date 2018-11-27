import numpy as np

class Poly(object):
    def __init__(self, points):
        # points: list of (x, y), sorted, anticlockwise
        # edge: tuple of points
        edges = list()
        num_points = len(points)
        assert num_points >= 3
        for i in range(0, num_points-1):
            edges.append((points[i], points[i+1]))
        edges.append((points[-1], points[0]))
        self.edges = edges
        self.points = points

def angle(x1, y1, x2, y2):
    # result in (-pi, pi)
    theta1 = np.arctan2(y1, x1)
    theta2 = np.arctan2(y2, x2)
    dtheta = theta2 - theta1
    while dtheta > np.pi:
        dtheta -= 2*np.pi
    while dtheta < -np.pi:
        dtheta += 2*np.pi
    return dtheta

def point_inside_poly(point, poly):
    sum_angle = 0
    for edge in poly.edges:
        p1, p2 = edge
        p1x, p1y = p1
        p2x, p2y = p2
        px, py = point
        x1, y1, x2, y2 = p1x - px, p1y - py, p2x - px, p2y - py
        cur_angle = angle(x1, y1, x2, y2)
        sum_angle += cur_angle
    
    # if sum_angle == 2pi then is inside, if 0 then outside
    return np.abs(sum_angle) > np.pi

points = [(0, 0), (1, 0.5), (2, 0), (0, 2)]
poly = Poly(points)
p = (1, 0.4)

print(point_inside_poly(p, poly))