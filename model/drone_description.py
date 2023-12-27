import numpy as np

class DroneDescription():
    """
    Class used to store parameters of the drone model.
    """

    def __init__(self):

        # model parameters 
        self.mass = 0.027 # kg (very light)
        self.arm = 0.0397 # m
        self.kf = 3.16e-10 # thrust coefficient
        self.km = 7.94e-12 # torque coeefficient
        self.thrust2weight = 2.25
        self.max_speed_kmh = 30.0 # km/h
        self.gnd_eff_coeff = 11.36859
        self.prop_radius = 2.31348e-2 # m
        self.drag_coeff_xy = 9.1785e-7
        self.drag_coeff_z = 10.311e-7
        self.dw_coeff_1 = 2267.18
        self.dw_coeff_2 = .16
        self.dw_coeff_3 = -.11
        self.g = 9.81 #m/s^2
        self.I_xx = 2.3951e-5
        self.I_yy = 2.3951e-5
        self.I_zz = 3.2347e-5

        # linear state space model
        self.A_matrix = np.zeros((12,12))
        self.A_matrix[0,3] = self.A_matrix[1,4]= self.A_matrix[2,5] = 1
        self.A_matrix[3,7] = self.g
        self.A_matrix[4,6] = -self.g
        self.A_matrix[6,9] = self.A_matrix[7,10] = self.A_matrix[8,11] = 1

        self.B_matrix = np.zeros((12,4))
        self.B_matrix[5,0] = -1/self.mass
        self.B_matrix[9,1] = 1/self.I_xx
        self.B_matrix[10,2] = 1/self.I_yy
        self.B_matrix[11,3] = 1/self.I_zz

        # self.B_matrix = np.zeros((12,4))
        # self.B_matrix[5,:] = (2*self.kf)/self.mass
        # self.B_matrix[9,0] = self.B_matrix[9,1] = -np.sqrt(2)*self.arm*self.kf/self.I_xx
        # self.B_matrix[9,2] = self.B_matrix[9,3] = np.sqrt(2)*self.arm*self.kf/self.I_xx
        # self.B_matrix[10,0] = self.B_matrix[10,3] = -np.sqrt(2)*self.arm*self.kf/self.I_yy
        # self.B_matrix[10,1] = self.B_matrix[10,2] = np.sqrt(2)*self.arm*self.kf/self.I_yy
        # self.B_matrix[11,0] = self.B_matrix[11,2] = -2*self.km/self.I_zz
        # self.B_matrix[11,1] = self.B_matrix[11,3] = 2*self.km/self.I_zz
        # self.B_matrix = self.B_matrix*16073