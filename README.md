# Planning & Decision Making Project: Comparison of Sampling Planning methods: RRT\*, RRT\*-N, RRT\*-Ni, CC-RRT\*-N, DC-RRT\*-N

## Install environment

Install the 'drones' environment specified in [gym_pybullet_drones](https://github.com/utiasDSL/gym-pybullet-drones). Then activate the environment before running any of the examples provided:
    
    conda activate drones

## Execute simple demo

This demo will showcase the different algorithms implemented and the different scenarios used for a fixed number of iterations (`--iter`).

Possible algorithms to test on, option `--alg algorithm_chosen`
- `rrt*` (default)
- `informed-rrt*`
- `rrt*n`
- `rrt*ni`
- `cc-rrt*n`
- `dc-rrt*n`

Possible scenarios `--sce scenario_chosen`:

- `bridge`
- `near`
- `far`
- `forest`
- `center`

The demo can be executed with:

    python3 demo.py --alg algorithm_chosen --sce scenario_chosen --iter n
    
### Video of execution 

    python3 demo.py --alg rrt* --sce bridge --iter 2000

https://github.com/ivrolan/pdm_project_quadrotor/assets/22964725/b2e40f20-e4ab-46f9-935d-619ac5364d3b

## Algorithms:

### RRT*
### Informed-RRT*
### RRT*N
### RRT*Ni
### CC-RRT*N
### DC-RRT*N

# Authors

- Iván López Broceño
- Quentin Missine
- Daniel Wright 
- Jack Zeng 
