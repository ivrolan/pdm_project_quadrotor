import time
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync, str2bool

from scenarios import randomScenario, treeScenario, wallScenario

# import our own rrt library
import rrt 

from pybullet_utils import plotGraph, createWall


GUI = True

env = CtrlAviary(gui=GUI)

# only test with 1 drone
ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

duration_sec = 15
PYB_CLIENT = env.getPyBulletClient()
startOrientation = p.getQuaternionFromEuler([0,0,0])

min_bound = [0, 0, 0]
max_bound = [8, 3, 3]

wallIds, occ_grid = createWall([3,0,1], 2, 2, 2, min_bound=min_bound, max_bound=max_bound)

#scene_ids, occ_grid = treeScenario(0, min_bound, max_bound, size=0.25, using_sim=True)
# print(occ_grid)
occ_grid.plot()
start = env.pos[0]
print("START:", start.tolist())
goal = [7., np.random.uniform(-2, 2), 1.]
print("GOAL:", goal)
# compute path with rrt

graph = rrt.Graph(start, goal)
# step < threshold!
threshold = 0.8
step = 0.2
min_space = occ_grid.origin
max_space = min_space + occ_grid.dimensions

## start of the path planning
start = time.time_ns()
while graph.goalReached != True:

    rrt.rrt(graph, occ_grid, threshold, 0.2, points_interp=50)
## end of path planning
ns_ellapsed = time.time_ns() - start

if GUI:
    graph.draw(min_bound, max_bound)

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
    print(obs)

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

print("Planning_time nanoseconds:", ns_ellapsed)
print("Planning_time seconds:", ns_ellapsed * 1e-9)
if time_controller != -1:
    print("Controller_time nanoseconds:", time_controller)
    print("Controller_time seconds:", time_controller * 1e-9)

    # warning:  controller does not follow the path perfectly, 
    #           sometimes it collides with obstacles and fails

# compute the length of the path
total_length = 0
for i in range(len(path) - 1):
    total_length += np.sqrt(np.sum((path[i+1] - path[i])**2))

print("Total length of the path:", total_length)