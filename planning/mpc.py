""""
MPC controller for a 6-DOF drone. The trajectory from start to finish is shown for x,y,z and phi, theta, psi and its derivatives. 

The state space matrices are based on https://arxiv.org/ftp/arxiv/papers/1908/1908.07401.pdf



"""

import numpy as np
import cvxpy as cp
from tqdm.notebook import tqdm
import matplotlib.pyplot as plt
# import control_matrices


class drone_properties:
    def __init__(self, dt, m, I_x, I_y, I_z):
        # Constants
        self.m = m # mass of the drone
        self.g = 9.81  # acceleration due to gravity

        # Moments of inertia (assumed symmetric for simplicity)
        self.I_x = I_x
        self.I_y = I_y
        self.I_z = I_z

        # State-space matrices
        self.A = np.array([
            [1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, -self.g, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, self.g, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ])

        self.B = np.array([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [1/self.m, 0, 0, 0],#Check this change it used to be one up
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 1/self.I_z, 0, 0],
            [0, 0, 1/self.I_y, 0],
            [0, 0, 0, 1/self.I_x]
        ])

        self.C = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
        ])
        self.D = np.array([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]])
        
    def next_x(self,x,u):
        return self.A.dot(x)+self.B.dot(u)
    
def dummy_control(vehicle, x_init, x_target):
    u_roll, u_pitch, u_yaw, u_thrust = 0.1, 0.1, 0.1, 0.1
    u = np.array([u_roll, u_pitch, u_yaw, u_thrust])
    
    return u, vehicle.next_x(x_init, u), x_init, None

def mpc_control(vehicle, N, x_init, x_target):
    # Initialize constraints and cost
    constraints = []
    cost = 0
    num_states = 12
    num_controls = 4
    x = cp.Variable(shape=(num_states, N+1))  # Assuming num_states is the number of states
    u = cp.Variable(shape=(num_controls, N))  # Assuming num_controls is the number of control inputs
    
    
    # Loop through each step
    for k in range(N):
        # Implement cost components and constraints for each step
        Q = np.diag([10, 10, 10, 1.0, 1.0, 1.0, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001])
        R = np.diag([1, 1., 1., 1.])
       
        x_max, x_min, xdot_max, xdot_min = 50, -50, 1.5, -0.5 #specify max and min points for velocity coordinates and angles, coordinates are needed for the range of ppossible solutions
        y_max, y_min, ydot_max, ydot_min = 50, -50, 1.5, -0.5
        z_max, z_min, zdot_max, zdot_min = 50, -50, 1.5, -0.5


        phi_max, phi_min, phidot_max, phidot_min = 15*180/np.pi,-15*180/np.pi, 0.1*180/np.pi,-0.5*180/np.pi
        theta_max, theta_min, thetadot_max, thetadot_min = 15*180/np.pi,-15*180/np.pi, 0.5*180/np.pi,-0.5*180/np.pi
        psi_max, psi_min, psidot_max, psidot_min = 15*180/np.pi,-15*180/np.pi, 0.5*180/np.pi,-0.5*180/np.pi

        constraints += [x[:, k] >= [x_min, y_min, z_min, xdot_min, ydot_min, zdot_min, phi_min, theta_min, psi_min, phidot_min, thetadot_min, psidot_min]]
        constraints += [x[:, k] <= [x_max, y_max, z_max, xdot_max, ydot_max, zdot_max, phi_max, theta_max, psi_max, phidot_max, thetadot_max, psidot_max]]
        # constraints += [u[:, k] <= [0.01,10,10,10]]


        cost += cp.quad_form(x_target - x[:, k], Q) + cp.quad_form(u[:, k], R)        

        constraints += [x[:, k+1] == vehicle.A @ x[:, k] + vehicle.B @ u[:, k]]

    # Implement additional cost components and constraints
    constraints += [x[:, 0] == x_init] 
    cost += cp.quad_form((x_target - x[:, N]), Q)
    # Formulate the optimization problem
    problem = cp.Problem(cp.Minimize(cost), constraints)

    # Solve the optimization problem
    problem.solve()
    
    return u[:, 0].value, x[:, 1].value, x[:, :].value, None

def simulate(vehicle, dt, T, x_init, x_target, plan_length, control_func, plot_trajectories=True):
    # Initialise the output arrays
    x_real = np.zeros((12, T+1))
    x_all = np.zeros((12, plan_length+1, T+1))
    u_real = np.zeros((4, T))
    x_real[:, 0] = x_init
    theta_all = np.zeros((T))
    timesteps = np.linspace(0, dt, T)

    for t in range(0, T):
        # Compute the control input (and apply it)
        
        u_out, x_out,x_all_out, theta_out = control_func(x_real[:, t]) #vehicle, x_real[:, t], x_target)
        
        # Next x is the x in the second state
        
        x_real[:, t+1] = x_out
        x_all[:, :, t] = x_all_out # Save the plan (for visualization)

        # Used input is the first input
        u_real[:, t] = u_out

        theta_all[t] = theta_out
        
    if plot_trajectories:
            plot_trajectorie('mpc_control.eps', x_real, u_real, T)

    # Function that plots the trajectories.
    # The plot is stored with the name of the first parameter
    # print(x_real.shape)
    # print(x_real.shape)
    return x_real, u_real, x_all, timesteps, theta_all


def plot_trajectorie(filename, x, u, T):
    f, axes = plt.subplots(3, 2, figsize=(12, 6))

    # Plot position x
    axes[0, 0].plot(x[0, :])
    axes[0, 0].set_ylabel(r"$x$ $(m)$", fontsize=14)
    axes[0, 0].set_yticks([np.min(x[0, :]), 0, np.max(x[0, :])])
    axes[0, 0].set_ylim([np.min(x[0, :]) - 0.1, np.max(x[0, :]) + 0.1])
    axes[0, 0].set_xlim([0, T])
    axes[0, 0].grid()

    # Plot x_dot
    axes[0, 1].plot(x[3, :])
    axes[0, 1].set_ylabel(r"$x_{dot}$ $(m)$", fontsize=14)
    axes[0, 1].set_yticks([np.min(x[3, :]), 0, np.max(x[3, :])])
    axes[0, 1].set_ylim([np.min(x[3, :]) - 0.1, np.max(x[3, :]) + 0.1])
    axes[0, 1].set_xlim([0, T])
    axes[0, 1].grid()

    # Plot position y
    axes[1, 0].plot(x[1, :])
    axes[1, 0].set_ylabel(r"$y$ $(m)$", fontsize=14)
    axes[1, 0].set_yticks([np.min(x[1, :]), 0, np.max(x[1, :])])
    axes[1, 0].set_ylim([np.min(x[1, :]) - 0.1, np.max(x[1, :]) + 0.1])
    axes[1, 0].set_xlim([0, T])
    axes[1, 0].grid()

    # Plot y_dot
    axes[1, 1].plot(x[4, :])
    axes[1, 1].set_ylabel(r"$y_{dot}$ $(m/s)$", fontsize=14)
    axes[1, 1].set_yticks([np.min(x[4, :]), 0, np.max(x[4, :])])
    axes[1, 1].set_ylim([np.min(x[4, :]) - 0.1, np.max(x[4, :]) + 0.1])
    axes[1, 1].set_xlim([0, T])
    axes[1, 1].grid()

    # Plot position z
    axes[2, 0].plot(x[2, :])
    axes[2, 0].set_ylabel(r"$z$ $(m)$", fontsize=14)
    axes[2, 0].set_yticks([np.min(x[2, :]), 0, np.max(x[2, :])])
    axes[2, 0].set_ylim([np.min(x[2, :]) - 0.1, np.max(x[2, :]) + 0.1])
    axes[2, 0].set_xlim([0, T])
    axes[2, 0].grid()

    # Plot z_dot
    axes[2, 1].plot(x[5, :])
    axes[2, 1].set_ylabel(r"$z_{dot}$ $(m/s)$", fontsize=14)
    axes[2, 1].set_yticks([np.min(x[5, :]), 0, np.max(x[5, :])])
    axes[2, 1].set_ylim([np.min(x[5, :]) - 0.1, np.max(x[5, :]) + 0.1])
    axes[2, 1].set_xlim([0, T])
    axes[2, 1].grid()

    plt.tight_layout()
    plt.show()

    f, axes = plt.subplots(3, 2, figsize=(12, 6))

    # Plot position phi
    axes[0, 0].plot(x[6, :])
    axes[0, 0].set_ylabel(r"$\phi$ $(m)$", fontsize=14)
    axes[0, 0].set_yticks([np.min(x[0, :]), 0, np.max(x[0, :])])
    axes[0, 0].set_ylim([np.min(x[0, :]) - 0.1, np.max(x[0, :]) + 0.1])
    axes[0, 0].set_xlim([0, T])
    axes[0, 0].grid()

    # Plot phi_dot
    axes[0, 1].plot(x[9, :])
    axes[0, 1].set_ylabel(r"$\phi_{dot}$ $(m)$", fontsize=14)
    axes[0, 1].set_yticks([np.min(x[3, :]), 0, np.max(x[3, :])])
    axes[0, 1].set_ylim([np.min(x[3, :]) - 0.1, np.max(x[3, :]) + 0.1])
    axes[0, 1].set_xlim([0, T])
    axes[0, 1].grid()

    # Plot theta
    axes[1, 0].plot(x[7, :])
    axes[1, 0].set_ylabel(r"$\theta$ $(m)$", fontsize=14)
    axes[1, 0].set_yticks([np.min(x[1, :]), 0, np.max(x[1, :])])
    axes[1, 0].set_ylim([np.min(x[1, :]) - 0.1, np.max(x[1, :]) + 0.1])
    axes[1, 0].set_xlim([0, T])
    axes[1, 0].grid()

    # Plot theta_dot
    axes[1, 1].plot(x[10, :])
    axes[1, 1].set_ylabel(r"$\theta_{dot}$ $(m/s)$", fontsize=14)
    axes[1, 1].set_yticks([np.min(x[4, :]), 0, np.max(x[4, :])])
    axes[1, 1].set_ylim([np.min(x[4, :]) - 0.1, np.max(x[4, :]) + 0.1])
    axes[1, 1].set_xlim([0, T])
    axes[1, 1].grid()

    # Plot psi
    axes[2, 0].plot(x[8, :])
    axes[2, 0].set_ylabel(r"$\psi$ $(m)$", fontsize=14)
    axes[2, 0].set_yticks([np.min(x[2, :]), 0, np.max(x[2, :])])
    axes[2, 0].set_ylim([np.min(x[2, :]) - 0.1, np.max(x[2, :]) + 0.1])
    axes[2, 0].set_xlim([0, T])
    axes[2, 0].grid()

    # Plot psi_dot
    axes[2, 1].plot(x[11, :])
    axes[2, 1].set_ylabel(r"$\psi_{dot}$ $(m/s)$", fontsize=14)
    axes[2, 1].set_yticks([np.min(x[5, :]), 0, np.max(x[5, :])])
    axes[2, 1].set_ylim([np.min(x[5, :]) - 0.1, np.max(x[5, :]) + 0.1])
    axes[2, 1].set_xlim([0, T])
    axes[2, 1].grid()

    plt.tight_layout()
    plt.show()

    f, axes = plt.subplots(2, 2, figsize=(12, 6))

    # Plot input
    axes[0, 0].plot(u[0, :])
    axes[0, 0].set_ylabel(r"$u1$ $(m)$", fontsize=14)
    axes[0, 0].set_yticks([np.min(x[0, :]), 0, np.max(x[0, :])])
    axes[0, 0].set_ylim([np.min(x[0, :]) - 0.1, np.max(x[0, :]) + 0.1])
    axes[0, 0].set_xlim([0, T])
    axes[0, 0].grid()

    # Plot input
    axes[1, 0].plot(u[1, :])
    axes[1, 0].set_ylabel(r"$u2$ $(m)$", fontsize=14)
    axes[1, 0].set_yticks([np.min(x[0, :]), 0, np.max(x[0, :])])
    axes[1, 0].set_ylim([np.min(x[0, :]) - 0.1, np.max(x[0, :]) + 0.1])
    axes[1, 0].set_xlim([0, T])
    axes[1, 0].grid()

    # Plot input
    axes[0, 1].plot(u[2, :])
    axes[0, 1].set_ylabel(r"$u3$ $(m)$", fontsize=14)
    axes[0, 1].set_yticks([np.min(x[0, :]), 0, np.max(x[0, :])])
    axes[0, 1].set_ylim([np.min(x[0, :]) - 0.1, np.max(x[0, :]) + 0.1])
    axes[0, 1].set_xlim([0, T])
    axes[0, 1].grid()

    # Plot input
    axes[1, 1].plot(u[3, :])
    axes[1, 1].set_ylabel(r"$u4$ $(m)$", fontsize=14)
    axes[1, 1].set_yticks([np.min(x[0, :]), 0, np.max(x[0, :])])
    axes[1, 1].set_ylim([np.min(x[0, :]) - 0.1, np.max(x[0, :]) + 0.1])
    axes[1, 1].set_xlim([0, T])
    axes[1, 1].grid()

    plt.tight_layout()
    plt.show()
    

if __name__ == "__main__":
    # Problem parameters
    dt =0.1                                  # Sampling period
    T = 75                                      # Steps to simulate
    N = 20

    x_init = np.array([0, 0, 0, 0, 0, 0,0,0,0,0,0,0])      #x,y,z, x_Dot, y_Dot, z_Dot, phi, theta, psi, phi_Dot, theta_dot, psi_dot
    x_end = np.array([3, 3, 3,  0, 0, 0,0,0,0,0,0,0])              # Dummy for this exercise

    # Get the vehicle dynamics
    m=1
    I_x,I_y,I_z = 1, 1, 1
    vehicle = drone_properties(dt, m, I_x, I_y, I_z)

    # Simulate the result
    controller = lambda x_init : mpc_control(vehicle, N, x_init, x_end)
    states, inputs, plans, timesteps, theta_Al = simulate(vehicle, dt, T, x_init, x_end, N, controller)

    states = states.T
    for i in range(len(states)):
        print(states[i])

    print("States:", states)#check if it reaches the final state
    
    # print("Inputs:", inputs)
    # print("Plans:", plans)
    # print("Timesteps:", timesteps)


    # plt.plot(timesteps, inputs.T)
    # plt.plot(timesteps, states.T[1:])
    # plt.show()
