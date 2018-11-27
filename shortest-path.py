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

def orientation(p1, p2, p3):
    # 0 for colinear
    # 1 for clockwise
    # 2 for counterclockwise
    
    eps = 0.0001

    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    
    val = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2)
    
    if np.abs(val) < eps:
        return 0
    
    return 1 if val > 0 else 2

def on_segment(p, q, r):
    x, y = q
    x1, y1 = p
    x2, y2 = r
    return min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2)

def line_segments_intersect(seg1, seg2):
    p1, q1 = seg1
    p2, q2 = seg2
    
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2) 
    o3 = orientation(p2, q2, p1) 
    o4 = orientation(p2, q2, q1)

    if o1 != o2 and o3 != o4:
        return True
    
    if o1 == 0 and on_segment(p1, p2, q1): return True 
    if o2 == 0 and on_segment(p1, q2, q1): return True 
    if o3 == 0 and on_segment(p2, p1, q2): return True 
    if o4 == 0 and on_segment(p2, q1, q2): return True 
  
    return False



'''
points = [(0, 0), (1, 0.5), (2, 0), (0, 2)]
poly = Poly(points)
p = (1, 0.4)

print(point_inside_poly(p, poly))

l1 = ((0, 0), (1, 1))
l2 = ((1, 1), (1, 0))
print(line_segments_intersect(l1, l2))
'''