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
<<<<<<< HEAD
from itertools import islice
=======

>>>>>>> 03c9cb4209c1fef2542aa0c0a178da855c541877

class Line():
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn = self.dirn / self.dist if self.dist != 0 else np.zeros_like(self.dirn)

    def path(self, t):
        return self.p + t * self.dirn

def distance(x, y):
    return np.linalg.norm(np.array(x) - np.array(y))

def isInObstacle(vex, obstacles, radius):
    obstacles = np.array(obstacles)
    distances = np.linalg.norm(obstacles - vex, axis=1)
    return np.any(distances < radius)

def isInObstacleBox(point, centers, dimensions):
    point = np.array(point)
    for center, dim in zip(centers, dimensions):
        half_dim = np.array(dim) / 2.0 + 0.20  # Including margin
        min_bound = center - half_dim
        max_bound = center + half_dim
        if np.all((point >= min_bound) & (point <= max_bound)):
            return True
    return False

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
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]
        self.sz = endpos[2] - startpos[2]

    def add_vex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
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
<<<<<<< HEAD
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

            if isInObstacleBox(randvex, obstacles, dimensions):
                continue

=======
    print(startpos, endpos, obstacles, dimensions)
    G = Graph(startpos, endpos)

    for _ in range(n_iter):
        randvex = G.randomPosition()

        if isInObstacleBox(randvex, obstacles, dimensions):
            continue

>>>>>>> 03c9cb4209c1fef2542aa0c0a178da855c541877
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
            if vex == newvex:
                continue

            dist = distance(vex, newvex)
            if isInObstacleBox(vex, newvex, dimensions):
                continue

            line = Line(vex, newvex)
            if isThruObstacle(line, obstacles, dimensions):
                continue

            idx = G.vex2idx[vex]
            if G.distances[newidx] + dist < G.distances[idx]:
                G.add_edge(idx, newidx, dist)
                G.distances[idx] = G.distances[newidx] + dist

        dist = distance(newvex, G.endpos)
<<<<<<< HEAD
        if dist < 2 * stepSize:  # Adjust the threshold as needed
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            G.distances[endidx] = min(G.distances.get(endidx, float('inf')), G.distances[newidx] + dist)
            G.success = True
            print('success in finding path')
            # break
=======
        if isInObstacleBox(newvex, G.endpos, dimensions):
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            G.distances[endidx] = min(G.distances.get(endidx, float('inf')), G.distances[newidx]+dist)

            G.success = True
            print('success')
            break
>>>>>>> 03c9cb4209c1fef2542aa0c0a178da855c541877
    return G

def nearest(G, vex, obstacles, dimensions):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(G.vertices):
        line = Line(v, vex)
        if isThruObstacle(line, obstacles, dimensions):
            continue

        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v

    return Nvex, Nidx



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


<<<<<<< HEAD
def plot_path(G, obstacles, dimensions, path=None):
=======
def plot(G, obstacles, dimensions, path=None):
>>>>>>> 03c9cb4209c1fef2542aa0c0a178da855c541877
    px = [x for x, y, z in G.vertices]
    py = [y for x, y, z in G.vertices]
    pz = [z for x, y, z in G.vertices]
    ax = plt.figure().add_subplot(projection='3d')

<<<<<<< HEAD
    for obs, dims in zip(islice(obstacles, 1, None), islice(dimensions, 1, None)):
=======
    for obs, dims in zip(obstacles, dimensions):
>>>>>>> 03c9cb4209c1fef2542aa0c0a178da855c541877
        plot_box(ax, obs, dims)

    ax.scatter(px, py, pz, c='cyan', s=5)
    ax.scatter(G.startpos[0], G.startpos[1], G.startpos[2], c='black')
    ax.scatter(G.endpos[0], G.endpos[1], G.endpos[2], c='black')

    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    
<<<<<<< HEAD
    lc = art3d.Line3DCollection(lines, zorder=3, colors='green', linewidths=.5, alpha=.1)
=======
    lc = art3d.Line3DCollection(lines, zorder=3, colors='green', linewidths=.5)
>>>>>>> 03c9cb4209c1fef2542aa0c0a178da855c541877
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
    if G.startpos not in G.vex2idx or G.endpos not in G.vex2idx:
        return None  # Start or end position not in the graph

    srcIdx = G.vex2idx[G.startpos]
    dstIdx = G.vex2idx[G.endpos]

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

    
if __name__ == '__main__':
    startpos = (0., 0., 0.)
<<<<<<< HEAD
    endpos = (9., 9., 9.)
=======
    endpos = (3., 3., 3.)
>>>>>>> 03c9cb4209c1fef2542aa0c0a178da855c541877
    obstacles = [(1.5, 1.5, 1.5), (6., 6., 6.)]
    n_iter = 620
    radius = 1
    dimensions = np.array([[1,1,1], [2.5,2.5,2.5]])
    print(dimensions)
    stepSize = 0.8

    G = RRT_star(startpos,endpos, obstacles, n_iter, dimensions, stepSize)


    # G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
    # # G = RRT(startpos, endpos, obstacles, n_iter, radius, stepSize)

    if G.success:
        print("A path has been found")
        path = dijkstra(G)
        print(path)
<<<<<<< HEAD
        plot_path(G, obstacles, dimensions, path)
    else:
        print("A path was not found")
        plot_path(G, obstacles, dimensions)
=======
        plot(G, obstacles, dimensions, path)
    else:
        print("A path was not found")
        plot(G, obstacles, dimensions)
>>>>>>> 03c9cb4209c1fef2542aa0c0a178da855c541877
