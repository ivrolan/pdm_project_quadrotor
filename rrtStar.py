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
GOAL_THRESHOLD = 3

class Node:
    
    def __init__(self, pos: np.array):
        self.pos = pos
        self.parent = None
        self.children = []
        self.cost = 0


class Graph:
    
    "Class which is the whole RRT graph"
    def __init__(self, start, goal):
        
        self.nodeArray = np.array([])
        self.edgeArray = []
        self.start = Node(start)
        self.addNode(self.start)
        self.goal = Node(goal)
        self.goalReached = False
        
    def calcDist(self, node1, node2):
        
        distance = np.sqrt((node1.pos[0] - node2.pos[0])**2 + (node1.pos[1] -node2.pos[1])**2)
        
        return distance
        
    
    def addNode(self, node):
        
        #self.nodeArray.append(node)
        self.nodeArray = np.append(self.nodeArray, node)
        #print(self.nodeArray)
    
    def addEdge(self, nodeInGraph, nodeToAdd):
        
        self.edgeArray.append([[nodeInGraph.pos[0], nodeToAdd.pos[0]], [nodeInGraph.pos[1], nodeToAdd.pos[1]]])
        
    
    def addNodetoExistingNode(self, nodeInGraph, nodeToAdd):
        
        #tempEdge = Edge(nodeInGraph, nodeToAdd)
        
        nodeInGraph.children.append(nodeToAdd)
        nodeToAdd.parent = nodeInGraph
        
        
        # Making it rrt*
        
        nodeToAdd.cost += nodeInGraph.cost
        # calculate distance between them
        nodeToAdd.cost += self.calcDist(nodeToAdd, nodeInGraph)
        
        
        self.addNode(nodeToAdd)
        self.addEdge(nodeInGraph, nodeToAdd)
    

    
    def findNearestNode(self, newNode):
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
            
        return False
    
    def findBestNode(self):
        
        bestDist = 1000000
        bestIdx = 0
        
        for idx, node in np.ndenumerate(self.nodeArray):
            
            goalDist = self.calcDist(node, self.goal)
            
            if goalDist < bestDist:
                
                bestDist = goalDist
                bestIdx  = idx
        
        return self.nodeArray[bestIdx]
            
            
    
    def getPath(self):
        
        flag = True
        goalNode = self.findBestNode()
       # print("test2")
        #print(goalNode)
        pathNodes = []
        
        parent1 = goalNode.parent
        parent2 = goalNode.parent
        #print("p1", parent1.pos)
        #print("p2", parent2.pos)
        # Return the path
        
        while flag != False:
            #print('test3')
            pathNodes.append(goalNode)
            print(goalNode.pos)
            if (goalNode.pos == self.start.pos):
                
                flag = False
                
            else:
                
                goalNode = goalNode.parent
        
            #print(goalNode.x)
            #print(self.start.x)
        
            # if (goalNode.pos[0] == self.start.pos[0]) and (goalNode.pos[1] == self.start.pos[1]):
            #     pathNodes.append(goalNode)
            #     flag = False
                
            # else:
            #     parent = goalNode.parent
            #     print('test3')
            #     pathNodes.append(goalNode)
            #     goalNode = parent
        
        return pathNodes
        
    
    
    def draw(self, min_bound: tuple, max_bound:tuple,obs=None):
        
        print("drawing")
        fig = plt.figure()
        ax = fig.add_subplot()#projection='3d')
        ax.set_xlim([min_bound[0], max_bound[0]])
        ax.set_ylim([min_bound[1], max_bound[1]])
        #ax.set_zlim([min_bound[2], max_bound[2]])

        for edge in self.edgeArray:
            
            ax.plot(edge[0], edge[1], color='black')
            
        pathNodes = self.getPath()
        #print("test1")
        
        for i in range(len(pathNodes)-1):
            
            ax.plot((pathNodes[i].pos[0], pathNodes[i+1].pos[0]), (pathNodes[i].pos[1], pathNodes[i+1].pos[1]), c='red')
        
        # if obs != None:
        #     print("drawing voxels")
        #     print(obs.occ_grid.shape)
            
        #     world_voxels = scale_3d_matrix_values(obs.occ_grid, obs.resolution)
        #     print(world_voxels.shape)
        #     ax.voxels(world_voxels, edgecolor='k')
        #     print("done")
        plt.show()
        
        
    
    def findCloseNodes(self, radius, centerNode):
        
        xCeil = centerNode.pos[0] + radius
        xFloor = centerNode.pos[0] - radius
        yCeil = centerNode.pos[1] + radius
        yFloor = centerNode.pos[1] - radius
        indexList = []

        # find closest nodes
        for idx, node in np.ndenumerate(self.nodeArray):
            
            if ((xFloor <= node.pos[0] <= xCeil) and (yFloor <= node.pos[1] <= yCeil) and (node.pos != centerNode.pos)):
                
                indexList.append(idx)
        
        #indexList = np.array(indexList)
        
        return indexList
    
    
    def updateChildCosts(self, parent):
        
        for node in self.nodeArray:
            
            if node.parent == parent:
                
                node.cost = parent.cost + self.calcDist(parent, node)
                
                self.updateChildCosts(node)
        
    
    def chooseParent(self, radius, centerNode):
        
        
        indexList = self.findCloseNodes(radius, centerNode)
        #print(indexList)
        cost = 100000000000
        bestIdx = 0
        
        for ind in indexList:
            
            node = self.nodeArray[ind]
            # TODO: Add collision here
            
            travelCost = self.calcDist(centerNode, node)
            
            if (travelCost + node.cost) < cost:
                cost = cost
                bestIdx = ind
            else:
                return centerNode
            
        
        centerNode.cost = cost
        centerNode.parent = self.nodeArray[bestIdx]
        #print(centerNode.parent.pos)
        
        self.nodeArray[-1] = centerNode
        
        return centerNode
            
    def rewire(self, radius, centerNode):
        
        indexList = self.findCloseNodes(radius, centerNode)
        
        for ind in indexList:
            
            node = self.nodeArray[ind]
            
            newCost = self.calcDist(centerNode, node) + centerNode.cost
            
            #TODO: Check collision
            
            if node.cost > newCost:
                
                print('testma')
                #print(node.parent.pos)
                node.parent = centerNode
                #print(node.parent.pos)
                node.cost = newCost
                
                self.nodeArray[ind] = node
                self.updateChildCosts(node)
        
        

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
    #min_space = occ_grid.origin
    #max_space = min_space + occ_grid.dimensions
    
    min_space = [0,0]
    max_space = [80, 80]

    randX = np.random.uniform(min_space[0], max_space[0]-0.0001)
    randY = np.random.uniform(min_space[1], max_space[1]-0.0001)
    #randZ = np.random.uniform(min_space[2], max_space[2]-0.0001)
    
    newNode = Node([randX, randY])
    
    nearestNode = graph.findNearestNode(newNode)

    if not graph.checkCollision(nearestNode, newNode, occ_grid, num_points=points_interp):
        
        graph.addNodetoExistingNode(nearestNode, newNode)
        
        updatedNode = graph.chooseParent(3, newNode)
        #print(newNode)
        graph.rewire(3, updatedNode)
        
        
        distanceToGoal = graph.calcDist(updatedNode, graph.goal)
        if (distanceToGoal < goal_threshold):
            
            graph.goalReached = True
            


def main():
    
    scene_ids, occ_grid = treeScenario(4, [0.,0.,0.], [80,80,80], size=10)
    
    start = [1,1]
    goal = [50, 75]
    min_space = occ_grid.origin
    max_space = min_space + occ_grid.dimensions
    
    graph = Graph(start, goal)

    

    #while graph.goalReached != True:
        
    #    rrt(graph, occ_grid, 10, 1)
    
    #while graph.goalReached != True:
        
    for i in range(0, 1500):
        
        rrt(graph, occ_grid, GOAL_THRESHOLD, 1)

    print(len(graph.nodeArray))
    graph.draw(min_space, max_space, obs=occ_grid)
    #optimalNodes = graph.getOptimalPath()
    
    return 0


if __name__ == '__main__':
    
    main()
