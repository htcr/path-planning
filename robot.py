import numpy as np
from shortest_path import *

def get_angle(edge):
    p1, p2 = edge
    x1, y1 = p1
    x2, y2 = p2
    x, y = x2 - x1, y2 - y1
    return np.arctan2(y, x) / np.pi * 180

def get_extreme_point_id(poly, mode):
    # 0 for lb, 1 for rt
    # return extreme point id
    min_dist = float('inf')
    min_id = 0
    if mode == 0:
        tx = min([p[0] for p in poly.points])
        ty = min([p[1] for p in poly.points])
    else:
        tx = max([p[0] for p in poly.points])
        ty = max([p[1] for p in poly.points])

    t = (tx, ty)

    for idx, p in enumerate(poly.points):
        cur_dist = line_segment_len((p, t))
        if cur_dist < min_dist:
            min_dist = cur_dist
            min_id = idx
    
    return min_id

def reflect_poly(poly):
    points = poly.points
    reflected = [(-x, -y) for x, y in points]
    return Poly(reflected)

def get_dir_list(poly):
    dir_list = list()
    for edge in poly.edges:
        p1, p2 = edge
        x1, y1 = p1
        x2, y2 = p2
        dir_list.append( ((0, 0), (x2-x1, y2-y1)) )
    return dir_list

def dilate_poly(poly, robot):
    poly_extreme_id = get_extreme_point_id(poly, mode=1)
    poly_extreme = poly.points[poly_extreme_id]
    reflected_robot = reflect_poly(robot)
    robot_extreme_id = get_extreme_point_id(reflected_robot, mode=1)
    
    poly_dirs = get_dir_list(poly)
    robot_dirs = get_dir_list(reflected_robot)
    
    p1 = poly_extreme_id
    p2 = robot_extreme_id
    
    joined_dirs = list()
    
    N1 = len(poly_dirs)
    N2 = len(robot_dirs)
    c1, c2 = 0, 0
    
    N = N1 + N2
    
    while True:
        if c1 == N1 and c2 == N2:
            break
        elif c1 < N1 and c2 == N2:
            joined_dirs.append(poly_dirs[p1])
            p1 = (p1 + 1) % N1
            c1 += 1
        elif c1 == N1 and c2 < N2:
            joined_dirs.append(robot_dirs[p2])
            p2 = (p2 + 1) % N2
            c2 += 1
        else:
            d1 = poly_dirs[p1]
            d2 = robot_dirs[p2]
            pt1, pt2 = d1
            pt3 = d2[1]

            ori = orientation(pt1, pt2, pt3)
            if ori == 2:
                joined_dirs.append(poly_dirs[p1])
                p1 = (p1 + 1) % N1
                c1 += 1
            else:
                joined_dirs.append(robot_dirs[p2])
                p2 = (p2 + 1) % N2
                c2 += 1
    
    new_points = list()
    cur_x, cur_y = poly_extreme
    for d in joined_dirs:
        dx, dy = d[1]
        cur_x += dx
        cur_y += dy
        new_points.append((cur_x, cur_y))
    
    return Poly(new_points)

def plot_env2(env, idx):
    robot, goal, obs = env
    start_idx = get_extreme_point_id(robot, mode=0)
    start = robot.points[start_idx]
    new_obs = [dilate_poly(p, robot) for p in obs]
    
    graph = build_visible_graph(start, goal, new_obs)
    path = find_path(graph)

    ax = plt.subplot()
    ax.axis('equal')

    plot_visible_graph(ax, graph)
    for poly in obs:
        plot_obstacle(ax, poly, c=(0.0, 0.0, 0.0))
    for poly in new_obs:
        plot_obstacle(ax, poly, c=(1.0, 0.0, 0.0))
    plot_path(ax, path)

    plot_obstacle(ax, robot, c=(0.0, 0.0, 1.0))
    
    plt.savefig('env_2_%d.png' % idx)
    plt.clf()

'''
poly = Poly([(1, 1), (4, 1), (4, 3), (1, 3)])
robot = Poly([(0, 0), (1, 0), (0.5, 0.5)])
new_poly = dilate_poly(poly, robot)

ax = plt.subplot()
ax.axis('equal')
plot_obstacle(ax, poly)
plot_obstacle(ax, new_poly, c=(1.0, 0.0, 0.0))
plt.show()
'''

poly0 = Poly([(0, 0), (1, 0), (1, 2), (0, 2)])
poly1 = Poly([(2, 0), (3, 0), (3, 2), (2, 2)])

robot = Poly([(-1, -1), (-0.5, -1), (-1, -0.2)])
goal = (3, 3)
obs = [poly0, poly1]
plot_env2((robot, goal, obs), 0)