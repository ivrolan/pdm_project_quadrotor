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
import rrt 

from pybullet_utils import plotGraph, inflate_obstacles_3d

GUI = True

env = CtrlAviary(gui=GUI, initial_xyzs=np.array([[1, 1, 1]]))

# only test with 1 drone
ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

duration_sec = 15
PYB_CLIENT = env.getPyBulletClient()
startOrientation = p.getQuaternionFromEuler([0,0,0])

min_bound = [0, 0, 0]
max_bound = [8, 8, 5]

# same pos for both
pos_list = [[3, 0, 0]]
width_list = [1]
height_list = [6]
depth_list = [5]


wallIds, occ_grid = createCubes(pos_list, width_list, height_list, depth_list, min_bound=min_bound, max_bound=max_bound)

occ_grid.plot()
# make the occ_grid bigger by 1 cell
occ_grid.occ_grid = inflate_obstacles_3d(occ_grid.occ_grid, 3)


#scene_ids, occ_grid = treeScenario(0, min_bound, max_bound, size=0.25, using_sim=True)
# print(occ_grid)
occ_grid.plot()
start = env.pos[0]
print("START:", start.tolist())
goal = [6, 1, 4]
#goal = [7., np.random.uniform(-2, 2), 1.]
#goal = [2, 0, 2]
print("GOAL:", goal)
# compute path with rrt 

graph = rrt.Graph(start, goal)
# step < threshold!
threshold = 0.5
step = 0.2
min_space = occ_grid.origin
max_space = min_space + occ_grid.dimensions

## start of the path planning
start = time.time_ns()
while graph.goalReached != True:
    rrt.rrt_star(graph, occ_grid, threshold, 0.2, points_interp=50)
    # rrt.rrt_gaussian(graph, occ_grid, threshold, 0.2, points_interp=10, covariance="varying")

## end of path planning
ns_ellapsed = time.time_ns() - start

if GUI:
    graph.draw(min_bound, max_bound)
    #graph.draw_line_samples()
# as the size is < 1.0 plotting with obs fails because of scaling 
# graph.draw(obs=occ_grid)

if GUI:
    plotGraph(graph)

# invert path so we start from the beginning
path = graph.getOptimalPath()[::-1]

# convert the nodes to coordinates
for i in range(len(path)):
    path[i] = np.array([path[i].x, path[i].y, path[i].z])

action = np.array([0.,0.,0.,0.]).reshape(1,4)
next_wp_index = 0

num_drones = 1
INIT_RPYS = np.array([[0, 0, 0] for i in range(num_drones)])

print("following")
START = time.time()
controller_start = time.time_ns()

# how to get the sim time without actually runnig the sim in gui?
# env.time

time_controller = -1
for i in range(0, int(duration_sec*env.CTRL_FREQ)):
    obs, reward, terminated, truncated, info = env.step(action)
    # print(obs)

    action[0], _, _ = ctrl.computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                state=obs[0],
                                                                # target_pos=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2]]),
                                                                target_pos=path[next_wp_index],
                                                                target_rpy=INIT_RPYS[0, :]
                                                                )
    env.render()
    if np.sqrt(np.sum((path[next_wp_index] - obs[0,:3])**2)) < 0.15 and next_wp_index < len(path):
        next_wp_index += 1
        if next_wp_index == len(path):
            time_controller = time.time_ns() - controller_start
            break
    if GUI:
        sync(i, START, env.CTRL_TIMESTEP)



env.close()

# Save the original standard output
original_stdout = sys.stdout

# Generate a date and time stamp
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

# Construct the filename with the timestamp - change path - scenario - search algorithm.
filename = f"Data/wallscenario/wallscenario_RRT_{timestamp}.csv"

# compute the length of the path
total_length = 0
for i in range(len(path) - 1):
    total_length += np.sqrt(np.sum((path[i+1] - path[i])**2))


data_header = ["time_ns", "time_s", "Length"]
data_content = [ns_ellapsed, ns_ellapsed * 1e-9, total_length]

# Redirect the standard output to the file
with open(filename, 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)

    # Write the header and data rows
    csvwriter.writerow(data_header)
    csvwriter.writerow(data_content)

print(f"CSV file saved to {filename}")

# Restore the original standard output
sys.stdout = original_stdout

print("Output saved to", filename)


