import numpy as np

class Poly(object):
    def __init__(self, points):
        # points: list of (x, y), sorted, anticlockwise
        # edge: tuple of points
        edges = list()
        num_points = len(points)
        # the previous and next edge of that vertex
        point_edges = list()

        assert num_points >= 3
        for i in range(0, num_points-1):
            edges.append((points[i], points[i+1]))
            point_edges.append( (points[i-1], points[i], points[i+1]) )
        edges.append((points[-1], points[0]))
        point_edges.append( (points[num_points-2], points[num_points-1], points[0]) )
        self.edges = edges
        self.points = points
        self.point_edges = point_edges

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

class Node(object):
    def __init__(self, point, idx, poly_idx, point_edges=None):
        self.point = point
        self.idx = idx
        self.poly_idx = poly_idx
        self.adj = list()
        self.point_edges = point_edges

def line_segment_in_poly(seg, poly):
    # check if the segment has at least
    # one point inside poly
    p1, p2 = seg
    return point_inside_poly(p1, poly) or point_inside_poly(p2, poly)

def line_segment_len(seg):
    x1, y1 = seg[0]
    x2, y2 = seg[1]
    return ((x2-x1)**2 + (y2-y1)**2)**0.5

def build_visible_graph(start, goal, obstacles):
    # start, end: point
    # obstacles: list of polys
    graph = list()
    graph.append(Node(start, len(graph), -1))
    
    for idx, poly in enumerate(obstacles):
        for pid, p in enumerate(poly.points):
            graph.append(Node(p, len(graph), idx, point_edges=poly.point_edges[pid]))

    graph.append(Node(goal, len(graph), -2))

    # add edges

    for i in range(len(graph)-1):
        for j in range(i+1, len(graph)):
            node_i, node_j = graph[i], graph[j]
            
            if node_i.point_edges is not None:
                p0, pi, p1 = node_i.point_edges
                pj = node_j.point

                o1 = orientation(p0, pi, pj)
                o2 = orientation(pi, p1, pj)

                if o1 == o2 == 2:
                    continue

            path = (pi, pj)
            path_exist = True
            for poly in obstacles:
                for edge in poly.edges:
                    if line_segments_intersect(path, edge):
                        path_exist = False
                        break
                if not path_exist:
                    break

            if path_exist:
                dist = line_segment_len(path)
                node_i.adj.append((node_j, dist))
                node_j.adj.append((node_i, dist))
    
    return graph



    
    
    


'''
points = [(0, 0), (1, 0.5), (2, 0), (0, 2)]
poly = Poly(points)
p = (0, 0)
'''

print(point_inside_poly(p, poly))

'''
l1 = ((0, 0), (1, 1))
l2 = ((1, 1), (1, 0))
print(line_segments_intersect(l1, l2))
'''