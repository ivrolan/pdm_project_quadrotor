
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

def treeScenario(n_trees : int, min_bound = [-2., -2, 0], max_bound = [2., 2., 2.]) -> list:

    ids = []

    orient = p.getQuaternionFromEuler([0,0,0])
    size = 0.5

    # for occ_grid generation
    origin = min_bound
    shape = np.array(max_bound) - np.array(min_bound)

    my_occ_grid = occupancy_grid.OccGrid3D(shape, origin, size)

    for i in range(n_trees):
        x = random.uniform(min_bound[0], max_bound[0])
        y = random.uniform(min_bound[1], max_bound[1])
        
        for j in range(int((max_bound[2] - min_bound[2])/size)):
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
