import time
import argparse
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync, str2bool

from scenarios import randomScenario, treeScenario, wallScenario, createWall, createCubes

# import our own rrt library
#import rrt 
import algorithms_rrt

from pybullet_utils import plotGraph, plotPointsPath, inflate_obstacles_3d

GUI = True

# get the arguments
parser = argparse.ArgumentParser(description='Demo of the working algorithm in Pybullet.')

possible_algorithms = ["rrt_star", "informed-rrt_star", "rrt_star-n", "rrt_star-ni",  "cc-rrt_star-n", "dc-rrt_star-n"]
possible_scenarios = ["bridge", "near", "far", "mid", "wall"]

parser.add_argument('--alg', type=str, default="rrt_star", choices=possible_algorithms, help='Algorithm chosen')
parser.add_argument('--sce', type=str, default="bridge", choices=possible_scenarios, help='Scenario chosen')
parser.add_argument('--iter',type=int, default=2000, help='Number of iterations')
parser.add_argument('--video',action='store_true', help="Option to save the runnning test into test.mp4 file")

# parse args
args = parser.parse_args()


env = CtrlAviary(gui=GUI)

# only test with 1 drone
ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

duration_sec = 20
PYB_CLIENT = env.getPyBulletClient()

min_bound = [0, 0, 0]
max_bound = [10, 10, 5]


"Near obstacle Scenario Creation"
if args.sce == "near":
    pos_list = [[0.5, 0.5, 0],[0.5,0.5,2.5]]
    width_list = [1,1]
    height_list = [1,1]
    depth_list = [2.5,2.5]
    goal = [9, 9, 4]

"Far obstacle Scenario Creation"
if args.sce == "far":
    pos_list = [[7.5, 7.5, 0],[7.5,7.5,2.5]]
    width_list = [1,1]
    height_list = [1,1]
    depth_list = [2.5,2.5]
    goal = [9, 9, 4]
if args.sce == "mid":
    pos_list = [[4.5, 4.5, 0],[4.5,4.5,2.5]]
    width_list = [1,1]
    height_list = [1,1]
    depth_list = [2.5,2.5]
    goal = [9, 9, 4]

"Bridge Scenario Creation"
if args.sce == "bridge":
    pos_list = [[0,2,0], [3.5, 2, 2.5], [6.5, 2, 0]]
    width_list = [3.5, 3, 3.5]
    height_list = [5, 5, 5]
    depth_list = [5, 2.5, 5]
    goal = [8, 8, 2.5]

if args.sce == "wall":
    pos_list = [[3, 0, 0],[3, 3, 0]]
    width_list = [1, 1]
    height_list = [3, 3]
    depth_list = [5, 5]
    goal = [6, 1, 4]

wallIds, occ_grid = createCubes(pos_list, width_list, height_list, depth_list, min_bound=min_bound, max_bound=max_bound, using_sim=True)
# scene_ids, occ_grid = treeScenario(6, min_bound, max_bound, size=0.25, using_sim=True)

# occ_grid.plot()
# make the occ_grid bigger by 1 cell
occ_grid.occ_grid = inflate_obstacles_3d(occ_grid.occ_grid, 7)

# goal = [2,2,2]
# print(occ_grid)
# occ_grid.plot()
start = env.pos[0]
print("start type is", start)
print("START:", start.tolist())
# start = start.tolist()
# goal = [7., 1, 1.]
# goal = [2, 0, 2]
print("GOAL:", goal)
# compute path with rrt 

graph = algorithms_rrt.Graph(start, goal)
# step < threshold!
threshold = 0.7
step = 0.3
min_space = occ_grid.origin
max_space = min_space + occ_grid.dimensions
rewire_radius = 0.7

## start of the path plannings
start = time.time_ns()

iter = 0

# possible_algorithms = ["rrt_star", "informed-rrt_star", "rrt_star-n", "rrt_star-ni",  "cc-rrt_star-n", "dc-rrt_star-n"]

for i in range(args.iter):
    if (args.alg == "rrt_star"):
        algorithms_rrt.rrt_star(graph, occ_grid, threshold, step, rewire_radius,  points_interp=50)
    elif (args.alg == "cc-rrt_star-n") :
        algorithms_rrt.rrt_star_gaussian(graph, occ_grid, threshold, 0.2, rewire_radius, points_interp=50, covariance_type="converging_cone")
    elif (args.alg == "dc-rrt_star-n"):
        algorithms_rrt.rrt_star_gaussian(graph, occ_grid, threshold, 0.2, rewire_radius, points_interp=50, covariance_type="diverging_cone")
    elif (args.alg == "rrt_star-n"):
        algorithms_rrt.rrt_star_gaussian(graph, occ_grid, threshold, 0.2, rewire_radius, points_interp=50, covariance_type="line")
    elif (args.alg == "rrt_star-ni"):
        algorithms_rrt.rrt_star_gaussian(graph, occ_grid, threshold, 0.2, rewire_radius, points_interp=50, covariance_type="varying")
    elif (args.alg == "informed-rrt_star"):
        algorithms_rrt.informed_rrt_star(graph, occ_grid, threshold, 0.2, rewire_radius, points_interp=50)

    iter += 1

reached = False
if len(graph.getPath(threshold)) > 1: 
    print("GOAL:", graph.goal.pos[0], graph.goal.pos[1], graph.goal.pos[2], "reached")
    reached = True
else:
    print("NOT_REACHED")
ns_ellapsed = time.time_ns() - start

if GUI:
    graph.draw(min_bound, max_bound, threshold)
    # graph.draw_line_samples()


# invert path so we start from the beginning
path = graph.getPath(threshold)[::-1]   

# convert the nodes to coordinates
for i in range(len(path)):
    path[i] = np.array([path[i].pos[0], path[i].pos[1], path[i].pos[2]])

if GUI and reached:
    print("Path is:", path)    
    plotGraph(graph)
    plotPointsPath(path, rgba=[1.,0.,0., 0.8])
action = np.array([0.,0.,0.,0.]).reshape(1,4)
next_wp_index = 0

num_drones = 1
INIT_RPYS = np.array([[0, 0, 0] for i in range(num_drones)])

print("following")
START = time.time()
controller_start = time.time_ns()

# env.time

# camera_target_position = [0, 0, 0]
camera_distance = 3.0
camera_pitch = -30.0
camera_yaw = 30.0


time_controller = -1
# compute the length of the path
total_length = 0
for i in range(len(path) - 1):
    total_length += np.sqrt(np.sum((path[i+1] - path[i])**2))
if reached:
    # from here start recording
    if args.video and reached:
        logId = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName="test.mp4")


    duration_sec = total_length*2.4
    for i in range(0, int(duration_sec*env.CTRL_FREQ)):
        obs, reward, terminated, truncated, info = env.step(action)
        # print(obs)

        action[0], _, _ = ctrl.computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                    state=obs[0],
                                                                    # target_pos=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2]]),
                                                                    target_pos=path[next_wp_index],
                                                                    target_rpy=INIT_RPYS[0, :]
                                                                    )
        print(obs[0][:3])
        p.resetDebugVisualizerCamera(
            cameraTargetPosition=obs[0][:3],
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
        )
        env.render()
        if np.sqrt(np.sum((path[next_wp_index] - obs[0,:3])**2)) < 0.15 and next_wp_index < len(path):
            print("WAS GOING TO:", path[next_wp_index])
            next_wp_index += 1
            if next_wp_index == len(path):
                time_controller = time.time_ns() - controller_start
                break
            
        if GUI:
            sync(i, START, env.CTRL_TIMESTEP)

    if args.video:
        p.stopStateLogging(logId)
env.close()

print("Planning_time nanoseconds:", ns_ellapsed)
print("Planning_time seconds:", ns_ellapsed * 1e-9)
print("Iter", iter)
print("Nodes added", len(graph.nodeArray))
if time_controller != -1:
    print("Controller_time nanoseconds:", time_controller)
    print("Controller_time seconds:", time_controller * 1e-9)

    # warning:  controller does not follow the path perfectly, 
    #           sometimes it collides with obstacles and fails

print("Total length of the path:", total_length)
# print("Cost of bestNode:", graph.findBestNode(threshold).cost)