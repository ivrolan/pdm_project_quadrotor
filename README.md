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
### Informed-RRT*
### RRT*N
### RRT*Ni
### CC-RRT*N
### DC-RRT*N

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
