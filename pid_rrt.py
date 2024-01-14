"Based on examle pid of the gym_pybullet_drones"

import os
import time
import argparse
import pdb
import math
import random
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
import pkg_resources
from datetime import datetime

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool
from planning.rrt3D import *
from Environments.Obstacle_creation import *

DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = False
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = False
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 30
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False

def run(case = 2,
        drone=DEFAULT_DRONES,
        num_drones=DEFAULT_NUM_DRONES,
        physics=DEFAULT_PHYSICS,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VISION,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        obstacles=DEFAULT_OBSTACLES,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        colab=DEFAULT_COLAB
        ):
    
    #### Initialize the simulation #############################
    H = .1
    R = .3
    startpos = (0., 0.,1.)
    if case == 1:
        iterations = 500
        endpos = (-5.,5.,2.) #env 1
    if case == 2:
        iterations = 3000
        endpos = (5.,-5.,2.) #env 2
    if case == 3:
        iterations = 1500
        endpos = (8.5, -8, 2.5) #env 3
    
    H_STEP = np.sqrt((np.linalg.norm(np.array(startpos)-np.array(endpos)))**2)/10 # setting RRT resolution

    INIT_XYZS = np.array([[0, 0,  0] for i in range(num_drones)])
    INIT_RPYS = np.array([[0, 0,  0] for i in range(num_drones)])
    END_XYZS = np.array([[1,1,1] for i in range(num_drones)])

    #### Create the environment ################################
    env = CtrlAviary(drone_model=drone,
                        num_drones=num_drones,
                        initial_xyzs=INIT_XYZS,
                        initial_rpys=INIT_RPYS,
                        physics=physics,
                        neighbourhood_radius=10,
                        pyb_freq=simulation_freq_hz,
                        ctrl_freq=control_freq_hz,
                        gui=gui,
                        record=record_video,
                        obstacles=obstacles,
                        user_debug_gui=user_debug_gui
                        )
    
    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()
    
    ### Obstacle environment ###
    if case == 1:
        obstacle_ids = create_boxes_1(env) # Storing id of obstacles
    if case == 2:
        obstacle_ids = create_boxes_2(env) # Storing id of obstacles
    if case == 3:
        obstacle_ids = create_boxes_3(env) # Storing id of obstacles
    print("obstacle_ids: ", obstacle_ids)
    obstacle_info = {}

    for i in range(len(obstacle_ids)):
        # Take into account that the relation beween the position and size of the box is the following: position is the coordinate of the 
        # center of the box (so half width, half height and half depth)
        shape = np.round(p.getCollisionShapeData(obstacle_ids[i], -1)[0][3], 6) # getting the information on the shape of the obstacles
        pos, orn = p.getBasePositionAndOrientation(obstacle_ids[i])
        obstacle_info[obstacle_ids[i]] = [obstacle_ids[i], pos, shape, orn] #INFO: Obstacle ID (float), position tuple(x, y, x, size tuple (x, y, x) and orientation tuple(quaternions)
        print(f"for obstacle {obstacle_ids[i]}, the shape is: {shape}\nand the global position is: {pos}, with orientiation {orn}\n")
    # print(obstacle_info)
    # Start and end visual indicators
    p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/waypoint.urdf'),
                startpos,
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/target.urdf'),
                    endpos,
                    p.getQuaternionFromEuler([0,0,0]),
                    useFixedBase=True,
                    physicsClientId=env.CLIENT
                    )
    
    positions = []
    dimensions=[] 
    for id in obstacle_info:
        positions.append(list(obstacle_info[id][1]))
        dimensions.append(list(obstacle_info[id][2]))

    #### Initialize a trajectory ######################
    """Using the 3D rrt to get the path, 
    path is split up using linspace based on the num_wp
    to match the number of positions in the TARGET_PATH to NUM_WP
    """
    positions = []
    dimensions = []
    for id in obstacle_info:
        positions.append(obstacle_info[id][1])
        dimensions.append(obstacle_info[id][2])
    G = RRT_star(startpos,endpos,positions, iterations, dimensions, H_STEP) # startpos,endpos, obstacles, n_iter, radius, stepSize

    if G.success:
        print("A path has been found")
        path = dijkstra(G)
        # print(path)
        # print(obstacles)
        plot_path(G, positions, dimensions, path)
    else:
        print("A path was not found")
        plot_path(G, positions, dimensions)

    PERIOD = G.length # equal to 1m/s speed
    NUM_WP = control_freq_hz*PERIOD
    NUM_iter = control_freq_hz*DEFAULT_DURATION_SEC
    path = np.array(dijkstra(G))
    num = int(NUM_WP/len(path))
    TARGET_POS = np.zeros((NUM_iter,3))
    
    count = 0
    for i in range(len(path)):
        try:
            target = np.linspace(path[i],path[i+1], num)
            for j in range(len(target)):
                TARGET_POS[count, :] = target[j]
                count+=1
        except:
            try:
                for k in range(NUM_iter-count):
                    TARGET_POS[count, :] = endpos #  + np.random.uniform(0, 0.1, size=(3,))
                    count+=1
            except:
                continue

    wp_counters = np.array([int((k*NUM_WP/6)%NUM_WP) for k in range(num_drones)])

    for i, tar in enumerate(TARGET_POS):
        # loading the target positions in the simulation as red dots
        if i%20 == 0:
            p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/target.urdf'),
                    tar,
                    p.getQuaternionFromEuler([0,0,0]),
                    useFixedBase=True,
                    physicsClientId=env.CLIENT
                    )

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )

    #### Initialize the controllers ############################
    if drone in [DroneModel.CF2X, DroneModel.CF2P]:
        ctrl = [DSLPIDControl(drone_model=drone) for i in range(num_drones)]

    #### Run the simulation ####################################
    action = np.zeros((num_drones,4))
    START = time.time()
    for i in range(0, int(duration_sec*env.CTRL_FREQ)):

        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)

        #### Compute control for the current way point #############
        for j in range(num_drones):
            action[j, :], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                    state=obs[j],
                                                                    #target_pos=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2]]),
                                                                    target_pos=INIT_XYZS[j, :] + TARGET_POS[wp_counters[j], :],
                                                                    target_rpy=INIT_RPYS[j, :]
                                                                    )

        #### Go to the next way point and loop and hold on the last one  #####################
        for j in range(num_drones):
            if wp_counters[j] < (NUM_WP-1):
                wp_counters[j] += 1


        #### Log the simulation ####################################
        for j in range(num_drones):
            try:
                logger.log(drone=j,
                        timestamp=i/env.CTRL_FREQ,
                        state=obs[j],
                        control=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2], INIT_RPYS[j, :], np.zeros(6)])
                        )
            except:
                # print("error in logger: TARGET_POS", TARGET_POS, wp_counters[j])
                # print("error in logger target: ", TARGET_POS[wp_counters[j], 0:2])
                # print("error in logger: INIT_XYZS",INIT_XYZS[j, 2])
                # print("error in logger: INIT_RPYS", INIT_RPYS[j, :])
                print("error in logger: ",)

        #### Printout ##############################################
        env.render()

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    logger.save()
    logger.save_as_csv("pid") # Optional CSV save

    #### Plot the simulation results ###########################
    if plot:
        logger.plot()

if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Helix flight script using CtrlAviary and DSLPIDControl')
    parser.add_argument('--drone',              default=DEFAULT_DRONES,             type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--num_drones',         default=DEFAULT_NUM_DRONES,         type=int,           help='Number of drones (default: 3)', metavar='')
    parser.add_argument('--physics',            default=DEFAULT_PHYSICS,            type=Physics,       help='Physics updates (default: PYB)', metavar='', choices=Physics)
    parser.add_argument('--gui',                default=DEFAULT_GUI,                type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VISION,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot',               default=DEFAULT_PLOT,               type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui',     default=DEFAULT_USER_DEBUG_GUI,     type=str2bool,      help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--obstacles',          default=DEFAULT_OBSTACLES,          type=str2bool,      help='Whether to add obstacles to the environment (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ, type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,    type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=DEFAULT_DURATION_SEC,       type=int,           help='Duration of the simulation in seconds (default: 5)', metavar='')
    parser.add_argument('--output_folder',      default=DEFAULT_OUTPUT_FOLDER,      type=str,           help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB,              type=bool,          help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    ################## CHANGE WAREHOUSE ENVIRONMENT HERE ##################
    case = 2
    ################## CHANGE WAREHOUSE ENVIRONMENT HERE ##################
    run(case = case, **vars(ARGS))
