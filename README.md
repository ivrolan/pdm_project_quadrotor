# Planning & Decision Making Project: Comparison of Sampling Planning methods: RRT\*, RRT\*-N, RRT\*-Ni, CC-RRT\*-N, DC-RRT\*-N

## Install environment

Install the 'drones' environment specified in [gym_pybullet_drones](https://github.com/utiasDSL/gym-pybullet-drones). Then activate the environment before running any of the examples provided:
    
    conda activate drones

## Execute simple demo

This demo will showcase the different algorithms implemented and the different scenarios used for a fixed number of iterations (`--iter`).

Possible algorithms to test on, option `--alg algorithm_chosen`
- `rrt_star` (default)
- `informed-rrt_star`
- `rrt_star-n`
- `rrt_star-ni`
- `cc-rrt_star-n`
- `dc-rrt_star-n`

Possible scenarios `--sce scenario_chosen`:

- `bridge`
- `near`
- `far`
- `forest`
- `center`

The demo can be executed with:

    python3 demo.py --alg algorithm_chosen --sce scenario_chosen --iter n
    
### Video of execution 

    python3 demo.py --alg rrt_star --sce bridge --iter 2000

https://github.com/ivrolan/pdm_project_quadrotor/assets/22964725/b2e40f20-e4ab-46f9-935d-619ac5364d3b

## Algorithms:

### RRT*

Implements all the features of RRT with two additions, it searches near parent nodes in a certain radius and rewires the extended node to the parent which results in the lowest path cost. This allows RRT* to find the optimal solution (over infinite iterations), as the path cost is minimized.

### Informed-RRT*

Once the path is found by RRT*, an ellipsoid can be described using the start and the goal as focus points and the length of the path as the length of the major axis through them. When sampling, only nodes inside this defined ellipsoid are accepted and used for extending the tree. As the length of the path decreases, the ellipsoid becomes smaller and the sampling is more focused around the path which improves the computation time to arrive at an optimal solution. 

### RRT*N

A modification of RRT* that draws a straight line L that connects the `start` and `goal`. This line is divided into equal segments with nodes. Upon every iteration, one of these nodes is randomly sampled. Then, a node about this point is sampled using a Gaussian distribution. Multiple variants of this method are discussed below. 

### RRT*Ni - Increasing Covariance
 
Whenever a node is rejected, due to it being in collision with an obstacle, the covariance of the Gaussian distribution increases exponentially by a factor 1.1. This causes the sampling to adapt to the size of the obstacles along the line, while still being able to find the optimal path around it. The covariance starts at 0.01 and increases to an upper bound of 10% of L.


### DC-RRT*N

The sampling region can vary depending on how far along L the node is sampled. Starting with a small covariance, and increasing the covariance as we advance along the line to the goal. The lower bound for this method is $.01 and the upper bound is 10% of L.

### CC-RRT*N

As opposed to DC-RRT\*N, CC-RRT\*N starts with a large covariance and decreases as we advance along the line to the goal. The lower bound for this method is 0.01 and the upper bound is 10% of L.


## Structure of the code

- `demo.py`: implements a working example of the algorithms and cases implemented
- `algorithms_rrt.py`: implements the `Graph` structure that contains `Node`s and the different algorithms implemented.
- `scenarios.py`: implements the creation of obstacles in the simulation according to the chosen scenario. At the same time, an object `OccGrid3D` is created containing the occupancy information of the scenario.
- `occupancy_grid.py`: implements the `OccGrid3D` object.
- `pybullet_utils.py`: contains helper functions to draw objects in the simulation.
- `scenario_test.py`: script used to obtain meausures of performance for the different algorithms and scenarios

# Authors

- Iván López Broceño
- Quentin Missine
- Daniel Wright 
- Jack Zeng 
