# pdm_project_quadrotor

# Install environment

Install the 'drones' environment specified in [gym_pybullet_drones](https://github.com/utiasDSL/gym-pybullet-drones).

Furthermore, in order to use the MPC controller we will use the [cvxpy](https://www.cvxpy.org/index.html) library. Make sure to install this package in the drones environment after activating it:
```
conda activate drones
conda install -c conda-forge cvxpy
```

# Execute simple test
```
conda activate drones
python3 my_drone_ex.py
```
