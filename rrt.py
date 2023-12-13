# -*- coding: utf-8 -*-
"""
Created on Wed Dec 13 19:14:24 2023

@author: d-jw
"""

import math
import random
import matplotlib.pyplot as plt
import numpy as np

LEN = 100
HIGHT = 100
WIDTH = 100
GOAL_THRESHOLD = 2

class Node:
    
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.children = []
        
        


class Graph:
    
    "Class which is the whole RRT graph"
    def __init__(self, start, goal, maxIter=100):
        
        self.nodeArray = []
        self.edgeArray = []
        self.start = Node(start[0], start[1])
        self.addNode(self.start)
        self.goal = goal
        self.maxIter = maxIter
        self.goalReached = False
        
        
    
    def addNode(self, node):
        
        self.nodeArray.append(node)
    
    def addEdge(self, nodeInGraph, nodeToAdd):
        
        self.edgeArray.append([[nodeInGraph.x, nodeToAdd.x], [nodeInGraph.y, nodeToAdd.y]])
        
    
    def addNodetoExistingNode(self, nodeInGraph, nodeToAdd):
        
        #tempEdge = Edge(nodeInGraph, nodeToAdd)
        
        nodeInGraph.children.append(nodeToAdd)
        nodeToAdd.parent = nodeInGraph
        self.addNode(nodeToAdd)
        self.addEdge(nodeInGraph, nodeToAdd)
    
    def findNearestNode(self, x, y):
        minDis = 10000000;
        nearestNode = Node(0,0)
        
        for node in self.nodeArray:
            
            distance = np.sqrt((node.x - x)**2 + (node.y -y)**2)
            
            if distance < (minDis):
                minDis = distance
                nearestNode = node
            
            
        return nearestNode
        

    
def rrt(graph):
    
    "Pick a random point"
    
    randX = np.random.randint(0, high=LEN)
    #print(randX)
    randY = np.random.randint(0, high=WIDTH)
    #print(randY)
    
    newNode = Node(randX, randY)
    
    nearestNode = graph.findNearestNode(randX, randY)
    
    graph.addNodetoExistingNode(nearestNode, newNode)
    
    
    distanceToGoal = np.sqrt((randX - graph.goal[0])**2 + (randY - graph.goal[1])**2)
    if (distanceToGoal < GOAL_THRESHOLD):
        
        graph.goalReached = True
        
    
    



def main():
    
    
    length = 100
    width = 100
    start = [1,1]
    goal = [75, 75]
    
    graph = Graph(start, goal)
    node = Node(50, 50)
    flag = True

    while graph.goalReached != True:
        
        rrt(graph)
          

    
    goalNode = graph.nodeArray[-1]
    print(goalNode)
    optimalNodes = [goalNode]
    
    while flag != False:
        print(goalNode.x)
        print(graph.start.x)
    
        if (goalNode.x == graph.start.x) and (goalNode.y == graph.start.y):
            optimalNodes.append(goalNode)
            flag = False
            
        else:
            parent = goalNode.parent
            optimalNodes.append(goalNode)
            goalNode = parent
    
     # print some stuff here   
    fig, ax = plt.subplots()
    ax.set_xlim([0, width])
    ax.set_ylim([0, length])

    for edge in graph.edgeArray:
        
        ax.plot(edge[0], edge[1])
        
    for i in range(len(optimalNodes)-1):
        
        ax.plot((optimalNodes[i].x, optimalNodes[i+1].x), (optimalNodes[i].y, optimalNodes[i+1].y), c='black')
    
        
    print("TEST")
    
    return 0


if __name__ == '__main__':
    
    main()
