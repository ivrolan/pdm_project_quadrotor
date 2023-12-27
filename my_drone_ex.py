import time
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
from rrt import rrt
from rrt import Graph

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl

# Change if we want multiple drones
num_drones = 1

env = CtrlAviary(gui=True)
duration_sec = 10
DRONE_MODEL = DroneModel("cf2x")

# Test RRT stuff

start = [1, 1, 1]

goal = [100, 100, 100]

graph = Graph(start, goal)

while graph.goalReached != True:
        
        rrt(graph)
        
optimalNodes = graph.getOptimalPath()


# The amount of waypoints in our trajectory, e.g. amount of RRT points
NUM_WAYPOINTS = len(optimalNodes)
TARGET_POS = np.zeros((NUM_WAYPOINTS,3)) # number of waypoints by x, y, z position
PYB_CLIENT = env.getPyBulletClient()






# Circle trajectory from the example, change this to our purpose
R = .3
H = 1.
H_STEP = .05
INIT_XYZS = np.array([[R*np.cos((i/6)*2*np.pi+np.pi/2), R*np.sin((i/6)*2*np.pi+np.pi/2)-R, H+i*H_STEP] for i in range(num_drones)])
INIT_RPYS = np.array([[0, 0,  i * (np.pi/2)/num_drones] for i in range(num_drones)])

for i in range(len(optimalNodes)):
    TARGET_POS[i, :] = optimalNodes[i].x, optimalNodes[i].y, optimalNodes[i].z
wp_counters = np.array([int((i*NUM_WAYPOINTS/6)%NUM_WAYPOINTS) for i in range(num_drones)]) # counter for where we are in the trajectory

# Initialize action vector
action = np.zeros((num_drones, 4)) # number of drones by the amount of actions, 1 RPM for each propellor

# Initalize PID controllers for each drone (swap out later for e.g. LQR or MPC)
if DRONE_MODEL in [DroneModel.CF2X, DroneModel.CF2P]:
    ctrl = [DSLPIDControl(drone_model=DRONE_MODEL) for i in range(num_drones)]

# Run simulation
for i in range(0, int(duration_sec*env.CTRL_FREQ)):
    env.render()
    # We can change the reward metric in the env that suits our purpose
    obs, reward, terminated, truncated, info = env.step(action)

    # For every drone calculate the action based on the current observation (control for current waypoint)
    # for drone in range(num_drones): 
        # action[drone,:] = 15000*np.array([[1., 1., 1., 1.]])
    for j in range(num_drones):
        position = np.array((optimalNodes[j].x, optimalNodes[j].y, optissssmalNodes[j].z))
        
        action[j, :], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                state=obs[j],
                                                                # target_pos=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2]]),
                                                                target_pos=TARGET_POS[wp_counters[j], :],
                                                                target_rpy=INIT_RPYS[j, :]
                                                                )

    # increment to the next waypoint
    for j in range(num_drones):
            wp_counters[j] = wp_counters[j] + 1 if wp_counters[j] < (NUM_WAYPOINTS-1) else 0

    time.sleep(1. / env.CTRL_FREQ)
env.close()

