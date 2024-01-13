
import pybullet as p
import numpy as np
import random

import occupancy_grid

"""
    Creates n boxes and returns a list with all the bodyIDs related to them
"""
def randomScenario(n_boxes : int, min_bound = [-2., -2, 0], max_bound = [2., 2., 2.], size = 0.5) -> list:
    
    ids = []
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    # size = 0.5
    
    # for occ_grid generation
    origin = min_bound
    shape = max_bound - np.array(min_bound)

    my_occ_grid = occupancy_grid.OccGrid3D(shape, origin, size)

    for i in range(n_boxes):
        x = random.uniform(min_bound[0], max_bound[0])
        y = random.uniform(min_bound[1], max_bound[1])
        z = random.uniform(min_bound[2], max_bound[2])
        

        ids.append(p.loadURDF("cube.urdf", [x, y,z], startOrientation, globalScaling=size, useFixedBase = True))
        
        # mark as occupied
        my_occ_grid.occupyCoords([x,y,z])

    return ids, my_occ_grid

"""
    Creates pillars with smaller cubes and return the list with all the bodyIDs related to them
"""

def treeScenario(n_trees : int, min_bound = [-2., -2, 0], max_bound = [2., 2., 2.], size=0.5, using_sim=False) -> list:
    # TODO: personally, i'm not fully comfortable with the `using_sim` flag
    # how to decouple the creation of the occ_grid from the simulation?
    ids = []

    orient = p.getQuaternionFromEuler([0,0,0])
    # size = 0.5

    # for occ_grid generation
    origin = min_bound
    shape = np.array(max_bound) - np.array(min_bound)

    my_occ_grid = occupancy_grid.OccGrid3D(shape, origin, size)

    for i in range(n_trees):
        x = random.uniform(min_bound[0], max_bound[0])
        y = random.uniform(min_bound[1], max_bound[1])
        
        for j in range(int((max_bound[2] - min_bound[2])/size)):
            if using_sim:
                ids.append(p.loadURDF("cube.urdf", [x, y,j*size], orient, globalScaling=size, useFixedBase = True))
            # mark as occupied
            my_occ_grid.occupyCoords([x,y,j*size])
    return ids, my_occ_grid

"""
    Creates walls in X direction with smaller cubes and return the list with all the bodyIDs related to them.
    Each wall is displaced by the distances. The length of 'distances' will determine the number of walls
"""
def wallScenario(distances: list, fill_factor: list, width : float, height : float, size=0.5):
    
    ids = []
    orient = p.getQuaternionFromEuler([0,0,0])
    d = 0

    # for occ_grid generation
    origin = [0, -width/2, 0]
    x_length = np.sum(np.array(distances))
    shape = np.array([x_length, width/2, height]) - np.array(origin)

    my_occ_grid = occupancy_grid.OccGrid3D(shape, origin, size)

    for index in range(len(distances)):
        
        d += distances[index]
        prob = fill_factor[index]
        # print(d)
        for i in range(int(width / size)):
            for j in range(int(height / size)):
                if random.random() < prob:
                    ids.append(p.loadURDF("cube.urdf", [d, -width/2 + i*(size), j*(size)], orient, globalScaling=size, useFixedBase = True))
                    my_occ_grid.occupyCoords([[d, -width/2 + i*(size), j*(size)]])
    return ids, my_occ_grid


# def corridorScenario()

#     addCube
#     addCube

def addCube(pos, width, height, depth, occ_grid: occupancy_grid.OccGrid3D, using_sim=True):
     
    rgba=[0.,0.,0.,0.5]
    bodyId = None
    if using_sim:
        cubeHalfExtents = [width/2, height/2, depth/2]
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=cubeHalfExtents, rgbaColor=rgba)
        
        collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=cubeHalfExtents)
        

        pos[0] += cubeHalfExtents[0]
        pos[1] += cubeHalfExtents[1]
        pos[2] += cubeHalfExtents[2]

        bodyId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collisionShapeId, baseVisualShapeIndex=visualShapeId,
                                basePosition=pos, baseOrientation=[0,0,0,1])
        
        pos[0] -= cubeHalfExtents[0]
        pos[1] -= cubeHalfExtents[1]
        pos[2] -= cubeHalfExtents[2]

    xgoal = np.arange(pos[0], pos[0]+width, occ_grid.resolution)
    ygoal = np.arange(pos[1], pos[1]+height,occ_grid.resolution)
    zgoal = np.arange(pos[2], pos[2]+depth, occ_grid.resolution)

     # print(xgoal)
    for i in xgoal:
        for j in ygoal:
            for k in zgoal:
                # print(i,j,k)
                occ_grid.occupyCoords((i, j, k))

    return bodyId

def createWall(pos, width, height, depth, min_bound = [-2., -2, 0], max_bound = [2., 2., 2.], step=0.2):
    
    # wallsID = []
    
    origin = min_bound 
    shape = np.array(max_bound) - np.array(min_bound)

    my_occ_grid = occupancy_grid.OccGrid3D(shape, origin, step)
    
    bodyId = addCube(pos, width, height, depth, my_occ_grid)

    return bodyId, my_occ_grid

def createCubes(pos: list, width: list, height: list, depth: list, min_bound = [-2., -2, 0], max_bound = [2., 2., 2.], step=0.2, using_sim=False):

    origin = min_bound 
    shape = np.array(max_bound) - np.array(min_bound)

    my_occ_grid = occupancy_grid.OccGrid3D(shape, origin, step)
    
    cubesIds = []
    for i in range(len(pos)):
        cubesIds.append(addCube(pos[i], width[i], height[i], depth[i], my_occ_grid, using_sim=using_sim))
    
    return cubesIds, my_occ_grid