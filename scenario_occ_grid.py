import time
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from scenarios import randomScenario, treeScenario, wallScenario

from scipy.ndimage import convolve

def inflate_obstacles_3d(grid, inflation_size):
    # Create a 3D structuring element (kernel)
    kernel = np.ones((inflation_size, inflation_size, inflation_size), dtype=int)

    # Use 3D convolution to inflate the obstacles
    print(grid.shape, kernel.shape)
    inflated_grid = convolve(grid, weights=kernel, mode='constant', cval=0)


    return inflated_grid


env = CtrlAviary(gui=True)
duration_sec = 10
PYB_CLIENT = env.getPyBulletClient()
startOrientation = p.getQuaternionFromEuler([0,0,0])

scene_ids, occ_grid = treeScenario(4, size=0.1)

# 3d plotting

occ_grid.plot()
occ_grid.occ_grid = inflate_obstacles_3d(occ_grid.occ_grid, 3)
print(occ_grid)

# 3d plotting

occ_grid.plot()

for i in range(0, int(duration_sec*env.CTRL_FREQ)):
    env.render()
    env.step(15000*np.array([[1., 1., 1., 1.]]))
    time.sleep(1. / env.CTRL_FREQ)
env.close()

