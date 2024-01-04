import time
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from scenarios import randomScenario, treeScenario, wallScenario

# import our own rrt library
import rrt 

env = CtrlAviary(gui=True)
duration_sec = 3
PYB_CLIENT = env.getPyBulletClient()
startOrientation = p.getQuaternionFromEuler([0,0,0])

min_bound = [0, -2, 0]
max_bound = [8, 2, 2]



scene_ids, occ_grid = treeScenario(15, min_bound, max_bound, size=0.25, using_sim=True)
# print(occ_grid)

start = env.pos[0]
print("START:", start.tolist())
goal = [7., np.random.uniform(-2, 2), 1.]
print("GOAL:", goal)
# compute path with rrt

graph = rrt.Graph(start, goal)
threshold = 0.2

min_space = occ_grid.origin
max_space = min_space + occ_grid.dimensions


while graph.goalReached != True:

    rrt.rrt(graph, occ_grid, threshold, points_interp=100)

graph.draw(min_bound, max_bound)

# as the size is < 1.0 plotting with obs fails because of scaling 
# graph.draw(obs=occ_grid)
for i in range(0, int(duration_sec*env.CTRL_FREQ)):
    env.render()
    #env.step(15000*np.array([[1., 1., 1., 1.]]))
    time.sleep(1. / env.CTRL_FREQ)
env.close()
