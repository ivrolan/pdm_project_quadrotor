import math
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation
import cvxpy as cp

from gym_pybullet_drones.control.BaseControl import BaseControl
from gym_pybullet_drones.utils.enums import DroneModel
from model.drone_description import DroneDescription

class MPC_controller(BaseControl):
    """
    MPC controller for tracking of the given reference plan.
    """
    def __init__(self,
                 drone_model: DroneModel,
                 g: float=9.8
                 ):
        
        super().__init__(drone_model=drone_model, g=g)
        if self.DRONE_MODEL != DroneModel.CF2X and self.DRONE_MODEL != DroneModel.CF2P:
            print("[ERROR] in MPC_controller.__init__(), DSLPIDControl requires DroneModel.CF2X or DroneModel.CF2P")
            exit()
        self.drone_description = DroneDescription()
        self.Q_pos = 100*np.eye(9) # State tracking error for position and velocity
        self.Q_pos[3,3] = self.Q_pos[4,4] = self.Q_pos[5,5] = 0
        self.R_pos = np.eye(4) # Input cost matrix
        self.horizon = 10
        self.P_COEFF_FOR = np.array([.4, .4, 1.25])
        self.I_COEFF_FOR = np.array([.05, .05, .05])
        self.D_COEFF_FOR = np.array([.2, .2, .5])
        self.P_COEFF_TOR = np.array([70000., 70000., 60000.])
        self.I_COEFF_TOR = np.array([.0, .0, 500.])
        self.D_COEFF_TOR = np.array([20000., 20000., 12000.])
        self.PWM2RPM_SCALE = 0.2685
        self.PWM2RPM_CONST = 4070.3
        self.MIN_PWM = 20000
        self.MAX_PWM = 65535
        if self.DRONE_MODEL == DroneModel.CF2X:
            self.MIXER_MATRIX = np.array([ 
                                    [-.5, -.5, -1],
                                    [-.5,  .5,  1],
                                    [.5, .5, -1],
                                    [.5, -.5,  1]
                                    ])
        elif self.DRONE_MODEL == DroneModel.CF2P:
            self.MIXER_MATRIX = np.array([
                                    [0, -1,  -1],
                                    [+1, 0, 1],
                                    [0,  1,  -1],
                                    [-1, 0, 1]
                                    ])
        self.reset()

    ################################################################################

    def reset(self):
        """Resets the control classes.

        The previous step's and integral errors for both position and attitude are set to zero.

        """
        super().reset()
        #### Store the last roll, pitch, and yaw ###################
        self.last_rpy = np.zeros(3)
        #### Initialized PID control variables #####################
        self.last_pos_e = np.zeros(3)
        self.integral_pos_e = np.zeros(3)
        self.last_rpy_e = np.zeros(3)
        self.integral_rpy_e = np.zeros(3)

    ################################################################################

    def computeControl(self,
                       control_timestep,
                       cur_pos,
                       cur_quat,
                       cur_vel,
                       cur_ang_vel,
                       target_pos,
                       target_rpy=np.zeros(3),
                       target_vel=np.zeros(3),
                       target_rpy_rates=np.zeros(3)
                       ):
        """Computes the MPC control action (as RPMs) for a single drone.

        This methods sequentially calls `_dslPIDPositionControl()` and `_dslPIDAttitudeControl()`.
        Parameter `cur_ang_vel` is unused.

        Parameters
        ----------
        control_timestep : float
            The time step at which control is computed.
        cur_pos : ndarray
            (3,1)-shaped array of floats containing the current position.
        cur_quat : ndarray
            (4,1)-shaped array of floats containing the current orientation as a quaternion.
        cur_vel : ndarray
            (3,1)-shaped array of floats containing the current velocity.
        cur_ang_vel : ndarray
            (3,1)-shaped array of floats containing the current angular velocity.
        target_pos : ndarray
            (3,1)-shaped array of floats containing the desired position.
        target_rpy : ndarray, optional
            (3,1)-shaped array of floats containing the desired orientation as roll, pitch, yaw.
        target_vel : ndarray, optional
            (3,1)-shaped array of floats containing the desired velocity.
        target_rpy_rates : ndarray, optional
            (3,1)-shaped array of floats containing the desired roll, pitch, and yaw rates.

        Returns
        -------
        ndarray
            (4,1)-shaped array of integers containing the RPMs to apply to each of the 4 motors.
        ndarray
            (3,1)-shaped array of floats containing the current XYZ position error.
        float
            The current yaw error.

        """
        self.control_counter += 1
        thrust, computed_target_rpy, pos_e = self._dslPIDPositionControl(control_timestep,
                                                                         cur_pos,
                                                                         cur_quat,
                                                                         cur_vel,
                                                                         cur_ang_vel,
                                                                         target_pos,
                                                                         target_rpy,
                                                                         target_vel
                                                                         )
        rpm = self._dslPIDAttitudeControl(control_timestep,
                                          thrust,
                                          cur_quat,
                                          computed_target_rpy,
                                          target_rpy_rates
                                          )
        cur_rpy = p.getEulerFromQuaternion(cur_quat)
        print("RPMs are ", rpm)
        return rpm, pos_e, computed_target_rpy[2] - cur_rpy[2]
    
    ################################################################################

    def MPCTrajectoryControl(self,
                        control_timestep,
                        cur_pos,
                        cur_quat,
                        cur_vel,
                        cur_ang_vel,
                        target_pos,
                        target_rpy,
                        target_vel
                        ):
        """
        Calculate the RPMs based on the position and rotations based 
        on a linearized state space model of a quadrotor.
        """
        cur_rpy = np.array(p.getEulerFromQuaternion(cur_quat))
        x = cp.Variable((12, self.horizon + 1)) # cp.Variable((dim_1, dim_2))
        u = cp.Variable((4, self.horizon))
        cost = 0 
        constraints = []
        targets = np.concatenate((target_pos, target_vel, target_rpy))
        
        x_current = np.concatenate((cur_pos,cur_vel,cur_rpy,cur_ang_vel))
        constraints += [x[:, 0] == x_current]


        for k in range(self.horizon):
            cost += cp.quad_form(x[:9,k] - targets,self.Q_pos) + cp.quad_form(u[:,k], self.R_pos)
            constraints += [x[:, k+1] == self.drone_description.A_matrix @ x[:,k] + self.drone_description.B_matrix @ u[:,k]]
            # constraints += [x[3:6,k+1]] <= np.array([self.drone_description.max_speed_kmh/3.6])

        cost += cp.quad_form(x[:9,self.horizon] - targets,self.Q_pos)
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve(solver=cp.OSQP)

        rpm = u[:,0].value
        return rpm


    def _dslPIDPositionControl(self,
                                control_timestep,
                                cur_pos,
                                cur_quat,
                                cur_vel,
                                cur_ang_vel,
                                target_pos,
                                target_rpy,
                                target_vel
                                ):
            """DSL's CF2.x PID position control.

            Parameters
            ----------
            control_timestep : float
                The time step at which control is computed.
            cur_pos : ndarray
                (3,1)-shaped array of floats containing the current position.
            cur_quat : ndarray
                (4,1)-shaped array of floats containing the current orientation as a quaternion.
            cur_vel : ndarray
                (3,1)-shaped array of floats containing the current velocity.
            target_pos : ndarray
                (3,1)-shaped array of floats containing the desired position.
            target_rpy : ndarray
                (3,1)-shaped array of floats containing the desired orientation as roll, pitch, yaw.
            target_vel : ndarray
                (3,1)-shaped array of floats containing the desired velocity.

            Returns
            -------
            float
                The target thrust along the drone z-axis.
            ndarray
                (3,1)-shaped array of floats containing the target roll, pitch, and yaw.
            float
                The current position error.

            """
            cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3, 3)
            pos_e = target_pos - cur_pos
            vel_e = target_vel - cur_vel
            # cur_rpy = np.array(p.getEulerFromQuaternion(cur_quat))
            self.integral_pos_e = self.integral_pos_e + pos_e*control_timestep
            self.integral_pos_e = np.clip(self.integral_pos_e, -2., 2.)
            self.integral_pos_e[2] = np.clip(self.integral_pos_e[2], -0.15, .15)
            #change to MPC
            

            target_thrust = np.multiply(self.P_COEFF_FOR, pos_e) \
                            + np.multiply(self.I_COEFF_FOR, self.integral_pos_e) \
                            + np.multiply(self.D_COEFF_FOR, vel_e) + np.array([0, 0, self.GRAVITY])
            scalar_thrust = max(0., np.dot(target_thrust, cur_rotation[:,2]))
            # print(scalar_thrust)
            thrust = (math.sqrt(scalar_thrust / (4*self.KF)) - self.PWM2RPM_CONST) / self.PWM2RPM_SCALE
            target_z_ax = target_thrust / np.linalg.norm(target_thrust)
            target_x_c = np.array([math.cos(target_rpy[2]), math.sin(target_rpy[2]), 0])
            target_y_ax = np.cross(target_z_ax, target_x_c) / np.linalg.norm(np.cross(target_z_ax, target_x_c))
            target_x_ax = np.cross(target_y_ax, target_z_ax)
            target_rotation = (np.vstack([target_x_ax, target_y_ax, target_z_ax])).transpose()
            #### Target rotation #######################################
            target_euler = (Rotation.from_matrix(target_rotation)).as_euler('XYZ', degrees=False)

            mpc_rpm = self.MPCTrajectoryControl(control_timestep, cur_pos, cur_quat, cur_vel, cur_ang_vel, target_pos, target_euler, target_vel)
            print("calculated RPMs", mpc_rpm)

            if np.any(np.abs(target_euler) > math.pi): # change the name here in the print statements
                print("\n[ERROR] ctrl it", self.control_counter, "in Control._dslPIDPositionControl(), values outside range [-pi,pi]")
            return thrust, target_euler, pos_e

        ################################################################################

    def _dslPIDAttitudeControl(self,
                                control_timestep,
                                thrust,
                                cur_quat,
                                target_euler,
                                target_rpy_rates
                                ):
            """DSL's CF2.x PID attitude control.

            Parameters
            ----------
            control_timestep : float
                The time step at which control is computed.
            thrust : float
                The target thrust along the drone z-axis.
            cur_quat : ndarray
                (4,1)-shaped array of floats containing the current orientation as a quaternion.
            target_euler : ndarray
                (3,1)-shaped array of floats containing the computed target Euler angles.
            target_rpy_rates : ndarray
                (3,1)-shaped array of floats containing the desired roll, pitch, and yaw rates.

            Returns
            -------
            ndarray
                (4,1)-shaped array of integers containing the RPMs to apply to each of the 4 motors.

            """
            cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3, 3)
            cur_rpy = np.array(p.getEulerFromQuaternion(cur_quat))
            target_quat = (Rotation.from_euler('XYZ', target_euler, degrees=False)).as_quat()
            w,x,y,z = target_quat
            target_rotation = (Rotation.from_quat([w, x, y, z])).as_matrix()
            rot_matrix_e = np.dot((target_rotation.transpose()),cur_rotation) - np.dot(cur_rotation.transpose(),target_rotation)
            rot_e = np.array([rot_matrix_e[2, 1], rot_matrix_e[0, 2], rot_matrix_e[1, 0]]) 
            rpy_rates_e = target_rpy_rates - (cur_rpy - self.last_rpy)/control_timestep
            self.last_rpy = cur_rpy
            self.integral_rpy_e = self.integral_rpy_e - rot_e*control_timestep
            self.integral_rpy_e = np.clip(self.integral_rpy_e, -1500., 1500.)
            self.integral_rpy_e[0:2] = np.clip(self.integral_rpy_e[0:2], -1., 1.)
            #### PID target torques ####################################
            target_torques = - np.multiply(self.P_COEFF_TOR, rot_e) \
                            + np.multiply(self.D_COEFF_TOR, rpy_rates_e) \
                            + np.multiply(self.I_COEFF_TOR, self.integral_rpy_e)
            target_torques = np.clip(target_torques, -3200, 3200)
            # print("target_torques", target_torques)
            pwm = thrust + np.dot(self.MIXER_MATRIX, target_torques)
            pwm = np.clip(pwm, self.MIN_PWM, self.MAX_PWM)
            print("PWM Signal is",pwm)
            print(target_torques)
            return self.PWM2RPM_SCALE * pwm + self.PWM2RPM_CONST

    ################################################################################

    def _one23DInterface(self,
                         thrust
                         ):
        """Utility function interfacing 1, 2, or 3D thrust input use cases.

        Parameters
        ----------
        thrust : ndarray
            Array of floats of length 1, 2, or 4 containing a desired thrust input.

        Returns
        -------
        ndarray
            (4,1)-shaped array of integers containing the PWM (not RPMs) to apply to each of the 4 motors.

        """
        DIM = len(np.array(thrust))
        pwm = np.clip((np.sqrt(np.array(thrust)/(self.KF*(4/DIM)))-self.PWM2RPM_CONST)/self.PWM2RPM_SCALE, self.MIN_PWM, self.MAX_PWM)
        if DIM in [1, 4]:
            return np.repeat(pwm, 4/DIM)
        elif DIM==2:
            return np.hstack([pwm, np.flip(pwm)])
        else:
            print("[ERROR] in DSLPIDControl._one23DInterface()")
            exit()
