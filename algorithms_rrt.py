# -*- coding: utf-8 -*-
"""
Created on Sun Jan  7 04:39:27 2024

@author: d-jw
"""

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
GOAL_THRESHOLD = 1

class Node:
    
    def __init__(self, pos: np.array):
        self.pos = pos
        self.parent = None
        self.children = []
        self.cost = 0

    def __eq__(self, __value: object) -> bool:
        if __value == None:
            return False
        if self.pos[0] != __value.pos[0]:
            return False
        if self.pos[1] != __value.pos[1]:
            return False
        if self.pos[2] != __value.pos[2]:
            return False
        return True
    def __ne__(self, __value: object) -> bool:
        return not self.__eq__(__value)
    
class Graph:
    
    "Class which is the whole RRT graph"
    def __init__(self, start, goal):
        
        self.nodeArray = np.array([])
        self.edgeArray = []
        self.start = Node(start)
        self.addNode(self.start)
        self.goal = Node(goal)
        self.goalReached = False

        self.straight_line = np.linspace(self.start.pos, self.goal.pos, 50)
        self.covariance = np.eye(3)*0.1 # very small value, gets overwritten base on use case later
        self.gaussian_points = []
        
    def calcDist(self, node1, node2):
        
        "Calculate distance between the two nodes"
        
        distance = np.sqrt((node1.pos[0] - node2.pos[0])**2 + (node1.pos[1] - node2.pos[1])**2 + (node1.pos[2] - node2.pos[2])**2)
        
        return distance
        
    
    def addNode(self, node):
        
        "Append node to the node array"
        
        self.nodeArray = np.append(self.nodeArray, node)
    
    def addEdge(self, nodeInGraph, nodeToAdd):
        
        "Add an edge to the edge array"
        
        self.edgeArray.append([[nodeInGraph.pos[0], nodeToAdd.pos[0]], [nodeInGraph.pos[1], nodeToAdd.pos[1]], [nodeInGraph.pos[2], nodeToAdd.pos[2]]])
    def deleteEdge(self, node1, node2):
        
        "Add an edge to the edge array"
        for i in range(len(self.edgeArray)):
            e = np.array(self.edgeArray[i])
            edge_node_1 = Node(e[:,0])
            edge_node_2 = Node(e[:,0])
            if edge_node_1 == node1 and edge_node_2 == node2:
                self.edgeArray.remove(i)
                return
        return
        
    
    def addNodetoExistingNode(self, nodeInGraph, nodeToAdd):
        
        "Initially add a parent node to the new node, update costs"
        
        nodeInGraph.children.append(nodeToAdd)
        nodeToAdd.parent = nodeInGraph
        nodeToAdd.cost += nodeInGraph.cost
        nodeToAdd.cost += self.calcDist(nodeToAdd, nodeInGraph)
        self.addNode(nodeToAdd)
        self.addEdge(nodeInGraph, nodeToAdd)
    
    
    def findNearestNode(self, newNode):
        
        "Find the nearest node to the new node"
        minDis = 10000000
        nearestNode = Node([0,0,0])
        for node in self.nodeArray:
            
            distance = self.calcDist(node, newNode)
            
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
            
        x_path = np.linspace(node1.pos[0], node2.pos[0], num_points)
        y_path = np.linspace(node1.pos[1], node2.pos[1], num_points)
        z_path = np.linspace(node1.pos[2], node2.pos[2], num_points)
        
        
        for i in range(num_points-1, 0, -1):
            # print(i)
            if (obs.isCoordOccupied((x_path[i], y_path[i], z_path[i]))):
                return True
            
        return False
    
    def findBestNode(self, goalThreshold) -> Node:
        
        "Finds the node closest to the goal region"
        
        bestCost = 100000000
        bestIdx = 0
        
        for idx, node in np.ndenumerate(self.nodeArray):
            
            goalDist = self.calcDist(node, self.goal)
            
            if goalDist < goalThreshold:
                
                if node.cost < bestCost:
                
                    bestCost = node.cost
                    bestIdx  = idx
        
        return self.nodeArray[bestIdx]
            
            
    
    def getPath(self, goal_threshold):
        
        "Returns the path from the node closest to the goal, back to the start"
        
        flag = True
        goalNode = self.findBestNode(goal_threshold)
        pathNodes = []
        
        parent1 = goalNode.parent
        parent2 = goalNode.parent
        
        while flag != False:
            if (goalNode == None):
                break
            pathNodes.append(goalNode)
            if (goalNode == self.start):
                
                flag = False
                
            else:
                # print("Goal node is ", goalNode.pos)
                goalNode = goalNode.parent
        
        return pathNodes
        
    def getPathLen(self, goal_threshold):

        total_length = 0
        path = self.getPath(goal_threshold)[::-1]
        for i in range(len(path)):
            path[i] = np.array([path[i].pos[0], path[i].pos[1], path[i].pos[2]])

        for i in range(len(path) - 1):
            total_length += np.sqrt(np.sum((path[i+1] - path[i])**2))
        return total_length
    
    def draw(self, min_bound: tuple, max_bound:tuple, goal_threshold, obs=None):
        
        "Draws the graph"
        
        print("drawing")
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.set_xlim([min_bound[0], max_bound[0]])
        ax.set_ylim([min_bound[1], max_bound[1]])
        ax.set_zlim([min_bound[2], max_bound[2]])

        # for edge in self.edgeArray:
            
        #     ax.plot(edge[0], edge[1], color='black')
        
        for node in self.nodeArray[1:]:
            
            parent = node.parent
            
            ax.plot((node.pos[0], parent.pos[0]), (node.pos[1], parent.pos[1]),  (node.pos[2], parent.pos[2]), color='black')
            
        pathNodes = self.getPath(goal_threshold)
        #print("test1")
        
        for i in range(len(pathNodes)-1):
            
            ax.plot((pathNodes[i].pos[0], pathNodes[i+1].pos[0]), (pathNodes[i].pos[1], pathNodes[i+1].pos[1]), (pathNodes[i].pos[2], pathNodes[i+1].pos[2]), c='red')
        
        if obs != None:
            print("drawing voxels")
            print(obs.occ_grid.shape)
            
            world_voxels = scale_3d_matrix_values(obs.occ_grid, obs.resolution)
            print(world_voxels.shape)
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
        self.ax.scatter(self.start.pos[0], self.start.pos[1], self.start.pos[2], color='green', label='Start Point')
        self.ax.scatter(self.goal.pos[0], self.goal.pos[1], self.goal.pos[2], color='red', label='End Point')
        self.ax.scatter(x_points, y_points, z_points, color='purple')
        plt.show()
        
        
    
    def findCloseNodes(self, radius, centerNode):
        
        "Finds all nodes in a sphere with defined radius around the given node"
        
        xCeil = centerNode.pos[0] + radius
        xFloor = centerNode.pos[0] - radius
        yCeil = centerNode.pos[1] + radius
        yFloor = centerNode.pos[1] - radius
        zCeil = centerNode.pos[2] + radius
        zFloor = centerNode.pos[2] - radius
        indexList = []

        for idx, node in np.ndenumerate(self.nodeArray):
            
            if ((xFloor <= node.pos[0] <= xCeil) and (yFloor <= node.pos[1] <= yCeil) and (zFloor <= node.pos[2] <= zCeil) and (node != centerNode)):
                
                indexList.append(idx)
        
        return indexList
    
    
    def updateChildCosts(self, parent):
        
        "Updates costs of all child nodes"
        
        for node in self.nodeArray:
            
            if node.parent == parent:
                
                node.cost = parent.cost + self.calcDist(parent, node)
                
                self.updateChildCosts(node)
        
    
    def chooseParent(self, radius, centerNode, occ_grid):
        
        "Chooses the best parent of the new node"
        
        
        indexList = self.findCloseNodes(radius, centerNode)

        cost = centerNode.cost
        bestIdx = -1
        
        for ind in indexList:
            
            node = self.nodeArray[ind]
            # TODO: Add collision here
            if not self.checkCollision(centerNode, node, occ_grid):
            
                travelCost = self.calcDist(centerNode, node)
            
                if (travelCost + node.cost) < cost:
                    cost = cost
                    bestIdx = ind
                
            
        if bestIdx == -1:
            return centerNode
        else:
            centerNode.cost = cost
            centerNode.parent = self.nodeArray[bestIdx]
            self.addEdge(self.nodeArray[bestIdx], centerNode)
            self.nodeArray[-1] = centerNode
        
        return centerNode
            
    def rewire(self, radius, centerNode, occ_grid):
        
        "Reconnects nodes in sphere around the given node if that node is a better parent"
        
        indexList = self.findCloseNodes(radius, centerNode)
        
        for ind in indexList:
            
            node = self.nodeArray[ind]
            
            newCost = self.calcDist(centerNode, node) + centerNode.cost
                        
            if not self.checkCollision(centerNode, node, occ_grid):
            
                if node.cost > newCost:
                    # delete prev edge

                    self.deleteEdge(node.parent, node)

                    node.parent = centerNode
                    node.cost = newCost
                
                    self.nodeArray[ind] = node
                    self.updateChildCosts(node)
                    self.addEdge(node.parent, node)
                    
    def extend(self, node1 : Node, node2: Node, step):
        extendedNode = Node(np.array([0., 0.]))

        # get direction
        direction = np.array(node2.pos) - np.array(node1.pos)

        # norm and scale by step if norm of direction is greater than step
        if np.linalg.norm(direction) >= step:
            extension = step * direction / np.linalg.norm(direction)
        else:
            return node2
        
        extendedNode = Node(np.array(node1.pos) + extension)

        return extendedNode
    
def rrt_star(graph, occ_grid, goal_threshold, step, rewire_rad, sample_node=None, points_interp=20):
    """
        Based on a graph, sample points withing the space of occ_grid and expand the graph
    """
    min_space = occ_grid.origin
    max_space = min_space + occ_grid.dimensions
    
    #min_space = [0,0]
    #max_space = [80, 80]
    if sample_node == None:
        randX = np.random.uniform(min_space[0], max_space[0]-0.0001)
        randY = np.random.uniform(min_space[1], max_space[1]-0.0001)
        randZ = np.random.uniform(min_space[2], max_space[2]-0.0001)
        
        newNode = Node([randX, randY, randZ])
    else:
        newNode = sample_node

    nearestNode = graph.findNearestNode(newNode)

    # sample the point with a fixed step size
    newNode = graph.extend(nearestNode, newNode, step)

    if not graph.checkCollision(nearestNode, newNode, occ_grid, num_points=points_interp):
        
        graph.addNodetoExistingNode(nearestNode, newNode)
        
        updatedNode = graph.chooseParent(rewire_rad, newNode, occ_grid)
        #print(newNode)
        graph.rewire(rewire_rad, updatedNode, occ_grid)
        
        
        distanceToGoal = graph.calcDist(updatedNode, graph.goal)
        if (distanceToGoal < goal_threshold):
            
            graph.goalReached = True

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
   
def line_gaussian_sample(graph, mean, covariance, covariance_type:str):
    # sample random point from the straight line
    random_line_point_id = np.random.choice(graph.straight_line.shape[0],1)
    random_line_point = graph.straight_line[random_line_point_id][0]

    ratio_point_on_line = random_line_point_id/graph.straight_line.shape[0]

    if covariance_type == "diverging_cone":
        diverging_covariance = ratio_point_on_line*covariance
        gaussianX, gaussianY, gaussianZ = np.random.multivariate_normal(random_line_point - mean, diverging_covariance)
        return diverging_covariance, gaussianX, gaussianY, gaussianZ
    elif covariance_type == "converging_cone":
        converging_covariance = (1-ratio_point_on_line)*covariance
        gaussianX, gaussianY, gaussianZ = np.random.multivariate_normal(random_line_point - mean, converging_covariance)
        return converging_covariance, gaussianX, gaussianY, gaussianZ
    else:
        # sample point based on Gaussian distribution based on this point on the line
        gaussianX, gaussianY, gaussianZ = np.random.multivariate_normal(random_line_point - mean, covariance)
        return gaussianX, gaussianY, gaussianZ

def rrt_star_gaussian(graph, occ_grid, goal_threshold, step, rewire_rad, covariance_type: str, points_interp=20):
    """
        Based on a graph, sample points withing the space of occ_grid and expand the graph
    """
    mean = 0
    line_magnitude = np.linalg.norm(graph.straight_line[-1] - graph.straight_line[0])

    if covariance_type == "line":
        covariance = np.eye(3)*0.05*line_magnitude
        randX, randY, randZ = line_gaussian_sample(graph, mean, covariance, covariance_type)
    elif covariance_type == "varying":
        graph.covariance = np.clip(graph.covariance,np.eye(3)*0.01,np.eye(3)*0.05*line_magnitude)
        randX, randY, randZ = line_gaussian_sample(graph, mean, graph.covariance, covariance_type)
    elif covariance_type == "diverging_cone":
        covariance = np.eye(3)*0.05*line_magnitude
        graph.covariance, randX, randY, randZ = line_gaussian_sample(graph, mean, covariance, covariance_type)
    elif covariance_type == "converging_cone":
        covariance = np.eye(3)*0.05*line_magnitude
        graph.covariance, randX, randY, randZ = line_gaussian_sample(graph, mean, covariance, covariance_type)

    in_grid = occ_grid.inOccGrid((randX, randY, randZ))

    newNode = Node([randX,randY,randZ])

    nearestNode = graph.findNearestNode(newNode)

    newNode = graph.extend(nearestNode, newNode, step)

    if in_grid and not graph.checkCollision(nearestNode, newNode, occ_grid, num_points=points_interp):

        graph.gaussian_points.append(np.array([randX, randY, randZ])) # for visualization purposes

        graph.addNodetoExistingNode(nearestNode, newNode)

        updatedNode = graph.chooseParent(rewire_rad, newNode, occ_grid)

        graph.rewire(rewire_rad, updatedNode, occ_grid)

        distanceToGoal = np.sqrt((newNode.pos[0] - graph.goal.pos[0])**2 + (newNode.pos[1] - graph.goal.pos[1])**2 + (newNode.pos[2] - graph.goal.pos[2])**2)
        if (distanceToGoal < goal_threshold):

            graph.goalReached = True
            print("Node array:", graph.nodeArray)
    elif covariance_type == "varying":
        graph.covariance*=1.1 
        # print("collision")            

def informed_rrt_star(graph : Graph, occ_grid, goal_threshold, step, rewire_rad, points_interp=20):
    """
    Compute one iter of Informed RRT*
    """

    if graph.goalReached:
        path_len = graph.getPathLen(goal_threshold)
        min_space = occ_grid.origin
        max_space = min_space + occ_grid.dimensions

        randX = np.random.uniform(min_space[0], max_space[0]-0.0001)
        randY = np.random.uniform(min_space[1], max_space[1]-0.0001)
        randZ = np.random.uniform(min_space[2], max_space[2]-0.0001)
        rand_sample = np.array([randX, randY, randZ])
        d_start = np.sqrt(np.sum((rand_sample-graph.start.pos)**2))

        # use the goal, instead of the last node
        d_goal = np.sqrt(np.sum((rand_sample-graph.goal.pos)**2))
        print("path length = ", path_len)
        #print("charact dis = ", d_start + d_goal)

        if d_start + d_goal <= path_len:
            node = Node(rand_sample)
            rrt_star(graph, occ_grid, goal_threshold, step, rewire_rad, sample_node=node)
    else:
        rrt_star(graph, occ_grid, goal_threshold, step, rewire_rad)

def main():
    
    scene_ids, occ_grid = treeScenario(4, [0.,0.,0.], [10,10,10], size=1)
    
    start = [1,1,1]
    goal = [7, 7, 7]
    min_space = occ_grid.origin
    max_space = min_space + occ_grid.dimensions
    
    graph = Graph(start, goal)

    

    #while graph.goalReached != True:
        
    #    rrt(graph, occ_grid, 10, 1)
    
    #while graph.goalReached != True:
        
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlim([min_space[0], max_space[0]])
    ax.set_ylim([min_space[1], max_space[1]])
    ax.set_zlim([min_space[2], max_space[2]])
        
    for i in range(0, 1000):
        
        #if graph.goalReached == True:
        #    break
        #b graph.draw(min_space, max_space, ax)
        graph.rrt(occ_grid, 10, 1)
        # plt.pause(0.05)
        
    
    print("finished")

    print(len(graph.nodeArray))
    graph.draw(min_space, max_space, ax)
    plt.show()
    #optimalNodes = graph.getOptimalPath()
    
    return 0


if __name__ == '__main__':
    
    main()
