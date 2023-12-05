import time
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary


env = CtrlAviary(gui=True)
duration_sec = 5
PYB_CLIENT = env.getPyBulletClient()

for i in range(0, int(duration_sec*env.CTRL_FREQ)):
    env.render()
    env.step(15000*np.array([[0.9, 0.9, 1., 1.]]))
    time.sleep(1. / env.CTRL_FREQ)
env.close()

