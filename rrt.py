# -*- coding: utf-8 -*-
"""
Created on Wed Dec 13 19:14:24 2023

@author: d-jw
"""

import math
import random
import matplotlib.pyplot as plt
import numpy as np

LENGTH = 100
HEIGHT = 100
WIDTH = 100
GOAL_THRESHOLD = 10

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
        
    
    
    def draw(self):
        
        
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.set_xlim([0, WIDTH])
        ax.set_ylim([0, LENGTH])
        ax.set_zlim([0, HEIGHT])

        for edge in self.edgeArray:
            
            ax.plot(edge[0], edge[1], edge[2], color='black')
            
        optimalNodes = self.getOptimalPath()
        
        for i in range(len(optimalNodes)-1):
            
            ax.plot((optimalNodes[i].x, optimalNodes[i+1].x), (optimalNodes[i].y, optimalNodes[i+1].y), (optimalNodes[i].z, optimalNodes[i+1].z), c='red')
        
        plt.show()
    
def rrt(graph):
    

    
    "Pick a random point"

    
    randX = np.random.uniform(0, LENGTH)
    randY = np.random.uniform(0, WIDTH)
    randZ = np.random.uniform(0, HEIGHT)
    
    newNode = Node(randX, randY, randZ)
    

            
    
    nearestNode = graph.findNearestNode(randX, randY, randZ)

    graph.addNodetoExistingNode(nearestNode, newNode)


    distanceToGoal = np.sqrt((randX - graph.goal[0])**2 + (randY - graph.goal[1])**2 + (randZ - graph.goal[2])**2)
    if (distanceToGoal < GOAL_THRESHOLD):
    
        graph.goalReached = True
    
    
    



def main():
    

    
    
    length = 100
    width = 100
    height = 100
    start = [1,1,1]
    goal = [75, 75,75]

    
    
    graph = Graph(start, goal)

    

    while graph.goalReached != True:
        
        rrt(graph)
          

    graph.draw()
    optimalNodes = graph.getOptimalPath()
    #goalNode = graph.nodeArray[-1]
    #print(goalNode)
    #optimalNodes = [goalNode]
    
    # Return the optimal path
    
    #while flag != False:
        #print(goalNode.x)
        #print(graph.start.x)
    
        #if (goalNode.x == graph.start.x) and (goalNode.y == graph.start.y) and (goalNode.z == graph.start.z):
            #optimalNodes.append(goalNode)
            #flag = False
            
        #else:
            #parent = goalNode.parent
            #optimalNodes.append(goalNode)
            #goalNode = parent
    
     # print some stuff here   
     
    #fig = plt.figure()
    #ax = fig.add_subplot(projection='3d')
    #ax.set_xlim([0, width])
    #ax.set_ylim([0, length])
    #ax.set_zlim([0, height])

   # for edge in graph.edgeArray:
        
        #ax.plot(edge[0], edge[1], edge[2])
        
    #for i in range(len(optimalNodes)-1):
        
        #ax.plot((optimalNodes[i].x, optimalNodes[i+1].x), (optimalNodes[i].y, optimalNodes[i+1].y), (optimalNodes[i].z, optimalNodes[i+1].z), c='black')
    
    #for i in range(len(obstacleList)):
        
        #ax.scatter(obstacleList[i].x, obstacleList[i].y,  c='red')
        #print(obstacleList[i].x, obstacleList[i].y)
    
        
    
    return 0


if __name__ == '__main__':
    
    main()
