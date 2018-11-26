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

class Node(object):
    def __init__(self, val, idx):
        self.val = val
        self.idx = idx
        self.prev = None
        self.next = None

def get_center(points):
    points_np = np.array(points)
    center_np = np.mean(points_np, axis=0)
    return (center_np[0], center_np[1])

def convex_hull(points):
    center = get_center(points)
    point_key = PointKey(center)
    sorted_points = list(points)
    sorted_points.sort(key=point_key)
    
    if len(sorted_points) <= 2:
        return sorted_points
    
    N = len(sorted_points)
    nodes = list()
    nodes.append(Node(sorted_points[0], 0))
    for i in range(1, N):
        nodes.append(Node(sorted_points[i], i))
        nodes[i].prev = nodes[i-1]
        nodes[i-1].next = nodes[i]
    nodes[0].prev = nodes[-1]
    nodes[-1].next = nodes[0]

    visited = set()
    remaining = N

    p1, p2, p3 = nodes[0], nodes[1], nodes[2]
    
    while remaining >= 3 and len(visited) < N:
        pp1, pp2, pp3 = p1.val, p2.val, p3.val
        if kick_point(pp1, pp2, pp3):
            p1.next = p3
            p3.prev = p1
            p2.prev = p2.next = None
            remaining -= 1
            visited.add(p2.idx)
            p2 = p1
            p1 = p1.prev
        else:
            visited.add(p2.idx)
            p1 = p2
            p2 = p3
            p3 = p3.next
    
    ans = list()
    ans.append(p1.val)
    p = p1.next
    while p != p1:
        ans.append(p.val)
        p = p.next
    return ans


'''
print(kick_point((1, 1), (0, 0), (-1, 1)))

p_key = PointKey((0, 0))
print(p_key((3**0.5, 1)))
print(p_key((1, 1)))
print(p_key((0, 1)))
print(p_key((-1, 1)))
print(p_key((-1, 0)))
print(p_key((-1, 0)))
print(p_key((-1, -0.1)))

print(get_center([(0, 0), (1, 0), (1, 1), (0, 1)]))
print(convex_hull([(0, 0), (0, 1), (1, 1), (1, 0)]))

print(convex_hull([(0, 0), (0, 1), (1, 1), (1, 0), (0.5, 0.5), (0.5, 2)]))
'''

