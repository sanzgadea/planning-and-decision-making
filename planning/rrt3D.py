'''
MIT License
3D rrt edited for PDM inspired by Copyright (c) 2019 Fanjin Zeng
This work is licensed under the terms of the MIT license, see <https://opensource.org/licenses/MIT>.  
'''
import numpy as np
from random import random
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque
import mpl_toolkits
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
import heapq
from collections import deque
from itertools import islice

class Line():
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn = self.dirn / self.dist if self.dist != 0 else np.zeros_like(self.dirn)

    def path(self, t):
        return self.p + t * self.dirn

def distance(points1, points2):
    return np.linalg.norm(np.array(points1) - np.array(points2))

def isInObstacle(vex, obstacles, radius):
    obstacles = np.array(obstacles)
    distances = np.linalg.norm(obstacles - vex, axis=1)
    return np.any(distances < radius)

def isInObstacleBox(points, centers, dimensions):
    points = np.array(points) if not isinstance(points, np.ndarray) else points
    centers = np.array(centers)
    dimensions = np.array(dimensions)
    half_dims = dimensions / 2.0 + 0.20  # Including margin

    min_bounds = centers - half_dims
    max_bounds = centers + half_dims

    # Ensure points have the correct shape for broadcasting
    if points.ndim == 1:
        points = points.reshape(1, -1)

    in_obstacle = np.all((points[:, None] >= min_bounds) & (points[:, None] <= max_bounds), axis=2)
    return np.any(in_obstacle, axis=1)

def Intersection(line, center, radius):
    a = np.dot(line.dirn, line.dirn)
    b = 2 * np.dot(line.dirn, line.p - center)
    c = np.dot(line.p - center, line.p - center) - radius ** 2
    discriminant = b ** 2 - 4 * a * c

    if discriminant < 0:
        return False

    sqrt_discriminant = np.sqrt(discriminant)
    t1 = (-b + sqrt_discriminant) / (2 * a)
    t2 = (-b - sqrt_discriminant) / (2 * a)

    return (0 <= t1 <= line.dist) or (0 <= t2 <= line.dist)

def isThruObstacle(line, centers, dimensions):
    for center, dim in zip(centers, dimensions):
        if IntersectionBox(line, center, dim):
            return True
    return False

def IntersectionBox(line, center, dimension):
    extra = 0.20  # Margin for collision
    half_dim = np.array(dimension) / 2.0 + extra
    min_bound = center - half_dim
    max_bound = center + half_dim
    tmin = (min_bound - line.p) / line.dirn
    tmax = (max_bound - line.p) / line.dirn
    tmin, tmax = np.minimum(tmin, tmax), np.maximum(tmin, tmax)
    t_enter = np.max(tmin)
    t_exit = np.min(tmax)
    return (t_enter <= t_exit) and (t_exit > 0)

class Graph:
    def __init__(self, startpos, endpos):
        self.startpos = np.array(startpos)
        self.endpos = np.array(endpos)

        self.vertices = [self.startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {tuple(startpos): 0}
        self.neighbors = {0: []}
        self.distances = {0: 0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]
        self.sz = endpos[2] - startpos[2]

        # Cache for distances between vertices
        self.distance_cache = {}
    
    def add_vex(self, pos):
        pos_tuple = tuple(pos)  # Convert numpy array to tuple
        try:
            idx = self.vex2idx[pos_tuple]
        except KeyError:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos_tuple] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))

    def randomPosition(self, sampling_radius=1.5):
        rx = random()
        ry = random()
        rz = random()

        posx = self.startpos[0] - (self.sx / 2.) + rx * self.sx * 2 * sampling_radius
        posy = self.startpos[1] - (self.sy / 2.) + ry * self.sy * 2 * sampling_radius
        posz = self.startpos[2] - (self.sz / 2.) + rz * self.sz * 2 * sampling_radius
        return posx, posy, posz

def RRT_star(startpos, endpos, obstacles, n_iter, dimensions, stepSize):
    # print(startpos, endpos, obstacles, dimensions)
    G = Graph(startpos, endpos)

    for iter in range(n_iter):
        if iter%250 == 0:
            print(f"currently in RRT iteration {iter}")
            randvex = G.endpos + np.array([0.01, 0.01, 0.01])
            # print(randvex)
            if isInObstacleBox(randvex, obstacles, dimensions):
                raise ValueError("END POSITION IS IN OBSTACLE!")
        else:
            randvex = G.randomPosition()

        if isInObstacleBox(np.array([randvex]), obstacles, dimensions)[0]:
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles, dimensions)
        if nearvex is None:
            continue

        newvex = newVertex(randvex, nearvex, stepSize)
        newidx = G.add_vex(newvex)
        dist = distance(newvex, nearvex)

        G.add_edge(newidx, nearidx, dist)
        G.distances[newidx] = G.distances[nearidx] + dist

        # update nearby vertices distance (if shorter)
        for vex in G.vertices:

            if np.array_equal(vex, newvex):
                continue

            dist = distance(vex, newvex)
            if isInObstacleBox(vex, newvex, dimensions):
                continue

            line = Line(vex, newvex)
            if isThruObstacle(line, obstacles, dimensions):
                continue

            idx = G.vex2idx[tuple(vex)]
            if G.distances[newidx] + dist < G.distances[idx]:
                G.add_edge(idx, newidx, dist)
                G.distances[idx] = G.distances[newidx] + dist

        dist = distance(newvex, G.endpos)
        if dist < 2 * stepSize:  # Adjust the threshold as needed
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            G.distances[endidx] = min(G.distances.get(endidx, float('inf')), G.distances[newidx] + dist)
            G.success = True
            print('success in finding path')
            # break
    return G

def nearest(G, vex, obstacles, dimensions):
    # Use vectorization for distance calculations
    vex_array = np.array([vex])
    vertices_array = np.array(G.vertices)
    
    # Ensure vertices_array is two-dimensional
    if vertices_array.ndim == 1:
        vertices_array = vertices_array.reshape(1, -1)

    distances = np.linalg.norm(vertices_array - vex_array, axis=1)

    # Filtering out vertices going through obstacles
    for idx, v in enumerate(G.vertices):
        line = Line(v, vex)
        if isThruObstacle(line, obstacles, dimensions):
            distances[idx] = float("inf")

    Nidx = np.argmin(distances)
    minDist = distances[Nidx]

    if minDist == float("inf"):
        return None, None
    else:
        return G.vertices[Nidx], Nidx

def newVertex(randvex, nearvex, stepSize):
    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min (stepSize, length)

    newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1], nearvex[2]+dirn[2])
    return newvex

def plot_sphere(ax, center, radius):
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)

    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))

    ax.plot_surface(x, y, z, color='b', alpha=0.5)

def plot_path(G, obstacles, dimensions, path=None):
    px = [x for x, y, z in G.vertices]
    py = [y for x, y, z in G.vertices]
    pz = [z for x, y, z in G.vertices]
    ax = plt.figure().add_subplot(projection='3d')

    for obs, dims in zip(islice(obstacles, 1, None), islice(dimensions, 1, None)):
        plot_box(ax, obs, dims)

    ax.scatter(px, py, pz, c='cyan', s=5)
    ax.scatter(G.startpos[0], G.startpos[1], G.startpos[2], c='black')
    ax.scatter(G.endpos[0], G.endpos[1], G.endpos[2], c='black')

    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    
    lc = art3d.Line3DCollection(lines, zorder=3, colors='green', linewidths=.5, alpha=.1)
    ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
        lc2 = art3d.Line3DCollection(paths, colors='red', linewidths=5)
        ax.add_collection(lc2)

    ax.autoscale()
    ax.margins(0.1)
    plt.show()

def plot_box(ax, center, dimensions):
    extra = 0.20  # margin for collision of the physical drone with the boxes
    half_dimensions = np.array(dimensions) / 2.0 + np.array([extra, extra, extra])
    
    min_bound = np.array(center) - half_dimensions
    max_bound = np.array(center) + half_dimensions
    
    x, y, z = zip(min_bound, max_bound)
    ax.bar3d(x[0], y[0], z[0], x[1]-x[0], y[1]-y[0], z[1]-z[0], color='gray', alpha=0.5)

def dijkstra(G):

    startpos_tuple = tuple(G.startpos)
    endpos_tuple = tuple(G.endpos)

    if startpos_tuple not in G.vex2idx or endpos_tuple not in G.vex2idx:
        return None  # Start or end position not in the graph

    srcIdx = G.vex2idx[startpos_tuple]
    dstIdx = G.vex2idx[endpos_tuple]

    nodes = list(G.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    priority_queue = [(0, srcIdx)]
    while priority_queue:
        current_distance, currentNode = heapq.heappop(priority_queue)

        if currentNode == dstIdx:
            break

        for neighbor, cost in G.neighbors[currentNode]:
            alternativeDistance = current_distance + cost
            if alternativeDistance < dist[neighbor]:
                dist[neighbor] = alternativeDistance
                prev[neighbor] = currentNode
                heapq.heappush(priority_queue, (alternativeDistance, neighbor))

    # retrieve path
    path = deque()
    curNode = dstIdx

    while prev[curNode] is not None:
        path.appendleft(G.vertices[curNode])
        curNode = prev[curNode]
    path.appendleft(G.vertices[curNode])

    return list(path)

def smooth_path(path, obstacles, dimensions, stepSize, maxIterations=150):
    iteration = 0
    while iteration < maxIterations:
        path_length = len(path)
        if path_length < 3:
            break  # Can't smooth a path with less than 3 points

        # Randomly select two non-adjacent points in the path
        idx1 = np.random.randint(0, path_length - 2)
        idx2 = np.random.randint(idx1 + 1, path_length)
        point1 = path[idx1]
        point2 = path[idx2]
        line = Line(point1, point2)
        
        # Check if the line intersects any obstacles
        if not isThruObstacle(line, obstacles, dimensions):
            # Create a new path skipping the points between idx1 and idx2
            new_path = path[:idx1+1] + path[idx2:]
            path = new_path  # Update the path
        iteration += 1
    return path
  
if __name__ == '__main__':
    startpos = (0., 0., 0.)
    endpos = (9., 9., 9.)
    obstacles = [(1.5, 1.5, 1.5), (6., 6., 6.)]
    n_iter = 620
    radius = 1
    dimensions = np.array([[1,1,1], [2.5,2.5,2.5]])
    print(dimensions)
    stepSize = 0.8

    G = RRT_star(startpos,endpos, obstacles, n_iter, dimensions, stepSize)

    if G.success:
        print("A path has been found")
        path = dijkstra(G)
        print("Original path:", path)
        
        # Smooth the path
        smooth_path = smooth_path(path, obstacles, dimensions, stepSize)
        print("Smoothed path:", smooth_path)
        # Plot paths
        plot_path(G, obstacles, dimensions, path)
        plot_path(G, obstacles, dimensions, smooth_path)

    else:
        print("A path was not found")
        plot_path(G, obstacles, dimensions)
