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
    
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
        self.children = []
        self.cost = 0


class Graph:
    
    "Class which is the whole RRT graph"
    def __init__(self, start, goal):
        
        self.nodeArray = np.array([])
        self.edgeArray = []
        self.start = Node(start[0], start[1], start[2])
        self.addNode(self.start)
        self.goal = goal
        self.goalReached = False
        
    
    def addNode(self, node):
        
        #self.nodeArray.append(node)
        self.nodeArray = np.append(self.nodeArray, node)
        #print(self.nodeArray)
    
    def addEdge(self, nodeInGraph, nodeToAdd):
        
        self.edgeArray.append([[nodeInGraph.x, nodeToAdd.x], [nodeInGraph.y, nodeToAdd.y], [nodeInGraph.z, nodeToAdd.z]])
        
    
    def addNodetoExistingNode(self, nodeInGraph, nodeToAdd):
        
        #tempEdge = Edge(nodeInGraph, nodeToAdd)
        
        nodeInGraph.children.append(nodeToAdd)
        nodeToAdd.parent = nodeInGraph
        
        
        # Making it rrt*
        
        nodeToAdd.cost += nodeInGraph.cost
        # calculate distance between them
        nodeToAdd.cost += np.sqrt((nodeToAdd.x - nodeInGraph.x)**2 + (nodeToAdd.y -nodeInGraph.y)**2 + (nodeToAdd.z - nodeInGraph.z)**2)
        
        
        self.addNode(nodeToAdd)
        self.addEdge(nodeInGraph, nodeToAdd)
        

        

    
    def findNearestNode(self, newNode):
        minDis = 10000000
        nearestNode = Node(0,0,0)
        
        for node in self.nodeArray:
            
            distance = np.sqrt((node.x - newNode.x)**2 + (node.y -newNode.y)**2 + (node.z - newNode.z)**2)
            
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
            print(i)
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
            print(obs.occ_grid.shape)
            
            world_voxels = scale_3d_matrix_values(obs.occ_grid, obs.resolution)
            print(world_voxels.shape)
            ax.voxels(world_voxels, edgecolor='k')
            print("done")
        plt.show()
        
        
        
        
    
    def chooseParent(self, radius, centerNode):
        
        node_list = []
        chosenNode = centerNode.parent
        xCeil = centerNode.x + radius
        xFloor = centerNode.x - radius
        yCeil = centerNode.y + radius
        yFloor = centerNode.y - radius
        zCeil = centerNode.z + radius
        zFloor = centerNode.z - radius
        print("test")
        
        counter = 0
        # choose parent
        for node in self.nodeArray:
            
            if (xFloor <= node.x <= xCeil) and (yFloor <= node.y <= yCeil) and (zFloor <= node.z <= zCeil):
                
                travelCost = np.sqrt((centerNode.x - node.x)**2 + (centerNode.y -node.y)**2 + (centerNode.z - node.z)**2)
                
                if (node.cost + travelCost) < centerNode.cost:
                    
                    centerNode.parent = node
                    centerNode.cost = node.cost + travelCost
                    self.nodeArray[-1] = centerNode
                    
                    
            counter += 1
        

        counter = 0

    
        #rewire graph
        
        for node in self.nodeArray:
            
            if (xFloor <= node.x <= xCeil) and (yFloor <= node.y <= yCeil) and (zFloor <= node.z <= zCeil):
            
                travelCost = np.sqrt((centerNode.x - node.x)**2 + (centerNode.y -node.y)**2 + (centerNode.z - node.z)**2)
            
                if  centerNode.cost + travelCost < node.cost:
                
                    node.parent = centerNode
                    node.cost = centerNode.cost + travelCost
                    self.nodeArray[counter] = node
                
            counter += 1
            
        
        

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
   
def rrt(graph, occ_grid, goal_threshold, step, points_interp=20):
    """
        Based on a graph, sample points withing the space of occ_grid and expand the graph
    """
    min_space = occ_grid.origin
    max_space = min_space + occ_grid.dimensions

    randX = np.random.uniform(min_space[0], max_space[0]-0.0001)
    randY = np.random.uniform(min_space[1], max_space[1]-0.0001)
    randZ = np.random.uniform(min_space[2], max_space[2]-0.0001)
    
    newNode = Node(randX, randY,randZ)
    
    nearestNode = graph.findNearestNode(newNode)
    

    
    # extend from nearestNode with a fixed step
    #nearest_vec = np.array([nearestNode.x, nearestNode.y, nearestNode.z])
    #norm_vec = np.array([randX, randY,randZ]) - nearest_vec

    #if np.linalg.norm(norm_vec) > step:
        #norm_vec = norm_vec / np.linalg.norm(norm_vec)

        #to_add_vec = nearest_vec + step * norm_vec
        #to_add_node = Node(to_add_vec[0], to_add_vec[1], to_add_vec[2])
    #else:
        #to_add_node = newNode
        

    if not graph.checkCollision(nearestNode, newNode, occ_grid, num_points=points_interp):
        
        graph.addNodetoExistingNode(nearestNode, newNode)
        
        graph.chooseParent(3, newNode)
        
        
        
        
        distanceToGoal = np.sqrt((newNode.x - graph.goal[0])**2 + (newNode.y - graph.goal[1])**2 + (newNode.z - graph.goal[2])**2)
        if (distanceToGoal < goal_threshold):
            
            graph.goalReached = True
            
    
    



def main():
    
    scene_ids, occ_grid = treeScenario(4, [0.,0.,0.], [80,80,80], size=10)
    
    start = [1,1,1]
    goal = [75, 75,75]
    min_space = occ_grid.origin
    max_space = min_space + occ_grid.dimensions
    
    graph = Graph(start, goal)

    

    #while graph.goalReached != True:
        
    #    rrt(graph, occ_grid, 10, 1)
    
    for i in range(0, 1000):
        
        rrt(graph, occ_grid, 10, 1)
          

    graph.draw(min_space, max_space, obs=occ_grid)
    optimalNodes = graph.getOptimalPath()
    
    return 0


if __name__ == '__main__':
    
    main()
