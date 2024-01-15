import matplotlib.pyplot as plt
from itertools import islice
import numpy as np
from matplotlib.patches import Rectangle

def plot_box3d(ax, center, dimensions):
    extra = 0.20  # margin for collision of the physical drone with the boxes
    half_dimensions = np.array(dimensions) / 2.0 + np.array([extra, extra, extra])
    
    min_bound = np.array(center) - half_dimensions
    max_bound = np.array(center) + half_dimensions
    
    x, y, z = zip(min_bound, max_bound)
    ax.bar3d(x[0], y[0], z[0], x[1]-x[0], y[1]-y[0], z[1]-z[0], color='gray', alpha=0.3)



def plots3D(rrt, mpc, obstacles, dimensions ):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(rrt[:, 0], rrt[:, 1], rrt[:, 2], label='RRT* Path', linestyle='dashed', color="blue")
    ax.plot(mpc[:, 0], mpc[:, 1], mpc[:, 2], label='MPC Path', color="red")


    for obs, dims in zip(islice(obstacles, 1, None), islice(dimensions, 1, None)):
        plot_box3d(ax, obs, dims)
    # Set labels and title
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title('3D Paths - RRT* and MPC')

    # Adding legend
    ax.legend()

    # Show the plot
    plt.show()

def plot_box2d(ax, center, dimensions):
    extra = 0.20  # margin for collision of the physical drone with the boxes
    half_dimensions = np.array(dimensions) / 2.0 + np.array([extra, extra, extra])
    
    min_bound = np.array(center) - half_dimensions
    max_bound = np.array(center) + half_dimensions
    
    rect = Rectangle((min_bound[0], min_bound[1]), dimensions[0], dimensions[1], color='gray', alpha=0.3)
    ax.add_patch(rect)



def plot2d(rrt, mpc,obstacles,dimensions):
    fig, ax = plt.subplots()
    ax.plot(rrt[:, 0], rrt[:, 1], label='RRT* Path', linestyle='dashed', color="blue")
    ax.plot(mpc[:, 0], mpc[:, 1], label='MPC Path', color="red")

    for obs, dims in zip(islice(obstacles, 1, None), islice(dimensions, 1, None)):
        plot_box2d(ax, obs, dims)

    # Set labels and title
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_title('2D Paths - RRT* and MPC')

    # Adding legend
    ax.legend()

    # Show the plot
    plt.show()
