import math
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from gym_pybullet_drones.control.BaseControl import BaseControl
from gym_pybullet_drones.utils.enums import DroneModel

class MPC_controller(BaseControl):
    """
    MPC controller for tracking of the RRT plan.
    """
    def __init__(self,
                 drone_model: DroneModel,
                 g: float=9.8
                 ):
        
        super().__init__(drone_model=drone_model, g=g)