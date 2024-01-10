# -*- coding: utf-8 -*-
"""
Created on Wed Dec 13 19:14:24 2023

@author: d-jw
"""

import math
import random
import matplotlib.pyplot as plt
import numpy as np
from scenarios import randomScenario, treeScenario, wallScenario

# to scale the voxels when plotting
from scipy.ndimage import zoom
LENGTH = 80
HEIGHT = 80
WIDTH = 80
GOAL_THRESHOLD = 5

class Node:
    
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
        self.children = []


class Graph:
    
    "Class which is the whole RRT graph"
    def __init__(self, start, goal):
        
        self.nodeArray = []
        self.edgeArray = []
        self.start = Node(start[0], start[1], start[2])
        self.addNode(self.start)
        self.goal = goal
        self.goalReached = False

        self.start_point = np.array([self.start.x, self.start.y, self.start.z])
        self.straight_line = np.linspace(self.start_point, self.goal, 50)
        self.covariance = np.eye(3)*0.01 # very small value, gets overwritten base on use case later
        self.gaussian_points = []
        self.collisioncheck = False
        self.iteration_counter = 0 # testing purposes, maybe remove later

    def addNode(self, node):
        
        self.nodeArray.append(node)
    
    def addEdge(self, nodeInGraph, nodeToAdd):
        
        self.edgeArray.append([[nodeInGraph.x, nodeToAdd.x], [nodeInGraph.y, nodeToAdd.y], [nodeInGraph.z, nodeToAdd.z]])
        
    
    def addNodetoExistingNode(self, nodeInGraph, nodeToAdd):
        
        #tempEdge = Edge(nodeInGraph, nodeToAdd)
        
        nodeInGraph.children.append(nodeToAdd)
        nodeToAdd.parent = nodeInGraph
        self.addNode(nodeToAdd)
        self.addEdge(nodeInGraph, nodeToAdd)
    
    def findNearestNode(self, x, y, z):
        minDis = 10000000
        nearestNode = Node(0,0,0)
        
        for node in self.nodeArray:
            
            distance = np.sqrt((node.x - x)**2 + (node.y -y)**2 + (node.z - z)**2)
            
            if distance < (minDis):
                minDis = distance
                nearestNode = node
            
            
        return nearestNode
    def checkCollision(self, node1 : Node, node2: Node, obs, num_points=10) -> bool:
        """
        Check the path between node1 and node2 taking node2 as endpoint.
        Returns True if collision, False if not
        """
        # assume nodes are 3D
        # TODO: this clean this

        x_path = np.linspace(node1.x, node2.x, num_points)
        y_path = np.linspace(node1.y, node2.y, num_points)
        z_path = np.linspace(node1.z, node2.z, num_points)
        
        
        for i in range(num_points-1, 0, -1):
            # print(i)
            if (obs.isCoordOccupied((x_path[i], y_path[i], z_path[i]))):
                return True
            
        return False
    
    def getOptimalPath(self):
        
        flag = True
        goalNode = self.nodeArray[-1]
        #print(goalNode)
        optimalNodes = [goalNode]
        
        # Return the optimal path
        
        while flag != False:
            #print(goalNode.x)
            #print(self.start.x)
        
            if (goalNode.x == self.start.x) and (goalNode.y == self.start.y) and (goalNode.z == self.start.z):
                optimalNodes.append(goalNode)
                flag = False
                
            else:
                parent = goalNode.parent
                optimalNodes.append(goalNode)
                goalNode = parent
        
        return optimalNodes
        
    
    
    def draw(self, min_bound: tuple, max_bound:tuple,obs=None):
        
        print("drawing")
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.set_xlim([min_bound[0], max_bound[0]])
        ax.set_ylim([min_bound[1], max_bound[1]])
        ax.set_zlim([min_bound[2], max_bound[2]])

        for edge in self.edgeArray:
            
            ax.plot(edge[0], edge[1], edge[2], color='black')
            
        optimalNodes = self.getOptimalPath()
        
        for i in range(len(optimalNodes)-1):
            
            ax.plot((optimalNodes[i].x, optimalNodes[i+1].x), (optimalNodes[i].y, optimalNodes[i+1].y), (optimalNodes[i].z, optimalNodes[i+1].z), c='red')
        
        if obs != None:
            print("drawing voxels")
            #print(obs.occ_grid.shape)
            
            world_voxels = scale_3d_matrix_values(obs.occ_grid, obs.resolution)
            #print(world_voxels.shape)
            ax.voxels(world_voxels, edgecolor='k')
            print("done")
        plt.show()

    def draw_line_samples(self):
        gauss_points = np.array(self.gaussian_points)
        x_coords, y_coords, z_coords = self.straight_line[:, 0], self.straight_line[:, 1], self.straight_line[:, 2]
        x_points, y_points, z_points = gauss_points[:, 0], gauss_points[:, 1], gauss_points[:, 2]

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.plot(x_coords, y_coords, z_coords, linestyle='-', color='blue', label='Line')
        self.ax.scatter(self.start_point[0], self.start_point[1], self.start_point[2], color='green', label='Start Point')
        self.ax.scatter(self.goal[0], self.goal[1], self.goal[2], color='red', label='End Point')
        self.ax.scatter(x_points, y_points, z_points, color='purple')
        plt.show()

def scale_3d_matrix_values(matrix, scale_factor):
    x, y, z = matrix.shape
    scaled_matrix = np.zeros((x * scale_factor, y * scale_factor, z * scale_factor), dtype=matrix.dtype)

    for i in range(x):
        for j in range(y):
            for k in range(z):
                scaled_matrix[i*scale_factor:(i+1)*scale_factor, 
                              j*scale_factor:(j+1)*scale_factor,
                              k*scale_factor:(k+1)*scale_factor] = matrix[i, j, k]

    return scaled_matrix
   
def random_sample(min_space, max_space):
    randX = np.random.uniform(min_space[0], max_space[0]-0.0001)
    randY = np.random.uniform(min_space[1], max_space[1]-0.0001)
    randZ = np.random.uniform(min_space[2], max_space[2]-0.0001)
    return randX, randY, randZ

def line_gaussian_sample(graph, mean, covariance):

    # sample random point from the straight line
    random_line_point_id = np.random.choice(graph.straight_line.shape[0],1)
    random_line_point = graph.straight_line[random_line_point_id][0]

    # sample point based on Gaussian distribution based on this point on the line
    gaussianX, gaussianY, gaussianZ = np.random.multivariate_normal(random_line_point - mean, covariance)
    return gaussianX, gaussianY, gaussianZ

def rrt_nodes(graph, x, y, z,occ_grid, goal_threshold, step, points_interp=20):
    newNode = Node(x,y,z)
    
    nearestNode = graph.findNearestNode(x, y, z)
    
    # extend from nearest Node with a fixed step
    nearest_vec = np.array([nearestNode.x, nearestNode.y, nearestNode.z])
    norm_vec = np.array([x,y,z]) - nearest_vec

    if np.linalg.norm(norm_vec) > step:
        norm_vec = norm_vec / np.linalg.norm(norm_vec)

        to_add_vec = nearest_vec + step * norm_vec
        to_add_node = Node(to_add_vec[0], to_add_vec[1], to_add_vec[2])
    else:
        to_add_node = newNode

    graph.collisioncheck = graph.checkCollision(nearestNode, to_add_node, occ_grid, num_points=points_interp)

    if not graph.collisioncheck:
        graph.addNodetoExistingNode(nearestNode, to_add_node)
        
        distanceToGoal = np.sqrt((to_add_node.x - graph.goal[0])**2 + (to_add_node.y - graph.goal[1])**2 + (to_add_node.z - graph.goal[2])**2)
        if (distanceToGoal < goal_threshold):
            graph.goalReached = True

def rrt(graph, occ_grid, goal_threshold, step, points_interp=20):
    """
        Based on a graph, sample points withing the space of occ_grid and expand the graph
    """
    min_space = occ_grid.origin
    max_space = min_space + occ_grid.dimensions

    randX, randY, randZ = random_sample(min_space, max_space)

    rrt_nodes(graph, randX, randY, randZ,occ_grid, goal_threshold, step, points_interp)

def informed_rrt(graph, occ_grid, goal_threshold, step, points_interp=20):
    """
    Based on a graph, sample points withing the space of occ_grid and expand the graph
    """
    if not graph.goalReached:
        min_space = occ_grid.origin
        max_space = min_space + occ_grid.dimensions
        randX, randY, randZ = random_sample(min_space, max_space)
        rrt_nodes(graph, randX, randY, randZ,occ_grid, goal_threshold, step, points_interp)
        graph.iteration_counter += 1
        print(graph.iteration_counter)
    else:
        graph.iteration_counter += 1
        in_ellipsoid = False
        while not in_ellipsoid:
            min_space = occ_grid.origin
            max_space = min_space + occ_grid.dimensions
            randX, randY, randZ = random_sample(min_space, max_space)
            path = graph.getOptimalPath()[::-1] # the best path

            # convert path to xyz cooridnates
            for i in range(len(path)):
                path[i] = np.array([path[i].x, path[i].y, path[i].z])

            path_length = 0
            for i in range(len(path) - 1):
                path_length += np.sqrt(np.sum((path[i+1] - path[i])**2))

            sampled_point = np.array([randX, randY, randZ])
            graph_goal = np.array(graph.goal)
            graph_start = np.array([graph.start.x, graph.start.y, graph.start.z])
            # compute distance from sampled point to start and to goal (compare with characteristic distance of an ellipsoid)
            distance_to_goal = np.linalg.norm(graph_goal - sampled_point)
            print("distance_to_goal", distance_to_goal)
            distance_to_start = np.linalg.norm(graph_start - sampled_point)
            print("distance to start is", distance_to_start)
            sum_distance = distance_to_start + distance_to_goal
            print("PATH LENGTH", path_length)
            if sum_distance > path_length:
                in_ellipsoid = False
            else:
                in_ellipsoid = True

        rrt_nodes(graph, randX, randY, randZ,occ_grid, goal_threshold, step, points_interp)

        # while not in_ellipsoid:
            # length of the path
            # distanceToGoal + distance to start = D
            # if D > length_path:
                # in_ellispoid = False
            # else:
                # in_ellipsoid = True
                # rrt_nodes(graph, randX, randY, randZ,occ_grid, goal_threshold, step, points_interp)


def rrt_gaussian(graph, occ_grid, goal_threshold, step, covariance: str, points_interp=20):
    """
        Based on a graph, sample points withing the space of occ_grid and expand the graph
    """
    mean = 0
    line_magnitude = np.linalg.norm(graph.straight_line[-1] - graph.straight_line[0])
    if covariance == "line":
        graph.covariance = np.eye(3)*0.05*line_magnitude
    elif covariance == "varying":
        graph.covariance = np.clip(graph.covariance,np.eye(3)*0.01,np.eye(3)*0.05*line_magnitude)

    randX, randY, randZ = line_gaussian_sample(graph, mean, graph.covariance)
    in_grid = occ_grid.inOccGrid((randX, randY, randZ))
    counter = 0
    while not in_grid:
        randX, randY, randZ = line_gaussian_sample(graph, mean, graph.covariance)
        in_grid = occ_grid.inOccGrid((randX, randY, randZ))
        counter += 1
        print(counter)
    graph.gaussian_points.append(np.array([randX, randY, randZ]))

    rrt_nodes(graph, randX, randY, randZ,occ_grid, goal_threshold, step, points_interp)
    graph.draw_line_samples()

    if graph.collisioncheck and covariance == "varying":
        graph.covariance *= 1.1   


        
def main():
    
    scene_ids, occ_grid = treeScenario(4, [0.,0.,0.], [80,80,80], size=10)
    
    start = [1,1,1]
    goal = [75, 75,75]
    min_space = occ_grid.origin
    max_space = min_space + occ_grid.dimensions

    graph = Graph(start, goal)

    while graph.goalReached != True:
        
        rrt(graph, occ_grid, 10, 1)
          
    graph.draw(min_space, max_space, obs=occ_grid)
    optimalNodes = graph.getOptimalPath()
    
    return 0


if __name__ == '__main__':
    
    main()
