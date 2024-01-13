import time

import sys
import csv
from datetime import datetime

import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync, str2bool

from scenarios import randomScenario, treeScenario, wallScenario, createWall, createCubes

# import our own rrt library
import algorithms_rrt 

from pybullet_utils import plotGraph, inflate_obstacles_3d

ITERATIONS = 2000
N_TESTS = 100

def runTest(scenario, algorithm, cubesFlag=True):

    min_bound = [0, 0, 0]
    max_bound = [10, 10, 5]
    
    if (scenario == "Corridor"):
    
        "Corridor Scenario Creation "
        "Uncomment for corridor scenario"
    
        pos_list = [[3,0,0], [1,6,0]]
        width_list = [5, 8]
        height_list = [5, 4]
        depth_list = [5, 5]
        goal = [9, 5, 2]
        
    
    elif (scenario == "Wall"):
    
        "Wall Scenario Creation"
        "Uncomment for wall scenario"
    
        pos_list = [[3, 0, 0],[3, 3, 0]]
        width_list = [1, 1]
        height_list = [3, 3]
        depth_list = [5, 5]
        goal = [6, 1, 4]
    
    elif (scenario == "Bridge"):
    
        "Bridge Scenario Creation"
        "Uncomment for bridge scenario"
    
        pos_list = [[0,2,0], [3.5, 2, 2.5], [6.5, 2, 0]]
        width_list = [3.5, 3, 3.5]
        height_list = [5, 5, 5]
        depth_list = [5, 2.5, 5]
        goal = [8, 8, 2.5]
        
    if (cubesFlag == True):
        
        wallIds, occ_grid = createCubes(pos_list, width_list, height_list, depth_list, min_bound=min_bound, max_bound=max_bound)
    
    else:
        scene_ids, occ_grid = treeScenario(5, min_bound, max_bound, size=0.25, using_sim=False)
        goal = [9, 9, 9]
    
    
    # occ_grid.plot()
    # make the occ_grid bigger by 1 cell
    occ_grid.occ_grid = inflate_obstacles_3d(occ_grid.occ_grid, 3)
    
    
    #scene_ids, occ_grid = treeScenario(0, min_bound, max_bound, size=0.25, using_sim=True)
    
    start = [1.,1.,1.]
    # print("START:", start)
    #goal = [7., np.random.uniform(-2, 2), 1.]
    #goal = [2, 0, 2]
    # print("GOAL:", goal)
    # compute path with rrt 
    # step < threshold!
    threshold = 0.5
    step = 0.2
    rewire_radius = 0.8
    min_space = occ_grid.origin
    max_space = min_space + occ_grid.dimensions
    
    ## start of the path planning
    
    data = []
    for t in range(N_TESTS):
        print(f"TEST {t+1}")
        graph = algorithms_rrt.Graph(start, goal)
        ns_start = time.time_ns()
    
        for i in range(ITERATIONS):
            
            if (algorithm == "rrtstar"):
                
                algorithms_rrt.rrt_star(graph, occ_grid, threshold, step, rewire_radius,  points_interp=50)
            #algorithms_rrt.rrt_star_gaussian(graph, occ_grid, threshold, 0.2, rewire_radius, points_interp=10, covariance_type="varying")
            if (algorithm == "conv_cone") :
                algorithms_rrt.rrt_star_gaussian(graph, occ_grid, threshold, 0.2, rewire_radius, points_interp=50, covariance_type="converging_cone")
            if (algorithm == "divg_cone"):
                algorithms_rrt.rrt_star_gaussian(graph, occ_grid, threshold, 0.2, rewire_radius, points_interp=50, covariance_type="diverging_cone")
            if (algorithm == "line"):
                algorithms_rrt.rrt_star_gaussian(graph, occ_grid, threshold, 0.2, rewire_radius, points_interp=50, covariance_type="line")
            if (algorithm == "varying"):
                algorithms_rrt.rrt_star_gaussian(graph, occ_grid, threshold, 0.2, rewire_radius, points_interp=50, covariance_type="varying")
            if (algorithm == "informed"):
                algorithms_rrt.informed_rrt_star(graph, occ_grid, threshold, 0.2, rewire_radius, points_interp=50)
                
            if i %10 == 0:
                # print(f"iter {i}")
                pass
        ## end of path planning
        ns_ellapsed = time.time_ns() - ns_start
    
        # invert path so we start from the beginning
        path = graph.getPath(threshold)[::-1]
        print(path)
        # convert the nodes to coordinates
        # for i in range(len(path)):
        #     path[i] = np.array(path[i].pos)
        # compute the length of the path
            
        total_length = 0
        for i in range(len(path) - 1):
            total_length += np.sqrt(np.sum((path[i+1].pos - path[i].pos)**2))
    
        data.append([ns_ellapsed, ns_ellapsed * 1e-9, total_length])
    
    # Save the original standard output
    original_stdout = sys.stdout
    
    # Generate a date and time stamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Construct the filename with the timestamp - change path - scenario - search algorithm.
    

    filename = f"Data/wallscenario/{scenario}_{algorithm}_{timestamp}.csv"

    
    
    data_header = ["time_ns", "time_s", "Length"]
    
    # Redirect the standard output to the file
    with open(filename, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
    
        # Write the header and data rows
        csvwriter.writerow(data_header)
        csvwriter.writerows(data)
    
    print(f"CSV file saved to {filename}")
    
    # Restore the original standard output
    sys.stdout = original_stdout
    
    print("Output saved to", filename)
    
    
    
runTest("Wall", "rrtstar", cubesFlag=False)

#runTest("Wall", "rrtstar")
#runTest("Bridge", "rrtstar")

#runTest("Corridor", "conv_cone")
#runTest("Wall", "conv_cone")
#runTest("Bridge", "conv_cone")

#runTest("Corridor", "divg_cone")
# runTest("Wall", "divg_cone")
# runTest("Bridge", "divg_cone")

# runTest("Corridor", "line")
# runTest("Wall", "line")
# runTest("Bridge", "line")


# runTest("Corridor", "varying")
# runTest("Wall", "varying")
# runTest("Bridge", "varying")

# runTest("Corridor", "informed")
# runTest("Wall", "informed")
# runTest("Bridge", "informed")