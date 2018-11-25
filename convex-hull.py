import numpy as np

# points: list of tuples
# [(x, y), (x, y) ...]
# regular right-hand math coordinate

# for sorting points according to angle
class PointKey(object):
    def __init__(self, center):
        # center: tuple (x, y)
        self.cx, self.cy = center
    
    def __call__(self, point):
        px, py = point
        angle = np.arctan2(py-self.cy, px-self.cx) / np.pi * 180.0
        return angle

def kick_point(p1, p2, p3):
    # p1, p2, p3: tuples, (x, y)
    v1x, v1y = p1[0]-p2[0], p1[1]-p2[1]
    v2x, v2y = p3[0]-p2[0], p3[1]-p2[1]
    cross_product = v1x*v2y - v1y*v2x
    return cross_product >= 0




#print(kick_point((1, 1), (0, 0), (-1, 1)))
'''
p_key = PointKey((0, 0))
print(p_key((3**0.5, 1)))
print(p_key((1, 1)))
print(p_key((0, 1)))
print(p_key((-1, 1)))
print(p_key((-1, 0)))
print(p_key((-1, 0)))
print(p_key((-1, -0.1)))
'''