# Planning and Decision Making

The objective of this project is to develop the path planning algorithm of a quadrotor in a warehouse simulation environment. The algorithm finds a path between a start and an end point avoiding the obstacles present in the environment.

An RRT* algorithm in 3D was implemented as a global planner to find the path from start to end position. This included a collision avoidance strategy to consider the layout of the warehouse and find the optimal path in different scenarios. Additionally, MPC was used for path smoothing and to take the physics of the drone into account. The physical feasibility of the found path is ensured, for example, by removing excessively sharp corners
along the path. 

## Environments

The obstacles are known beforehand, and, for simplicity, the representation of the obstacles is limited to bounding boxes. Three environments of in-
creasing complexity were created:

1) A warehouse with two boxes, only one of which is between the start and end positions of the drone, therefore intersecting the direct path and needing to be avoided. 

2) A warehouse with two walls creating corridor and a corner. The corridor is narrow and the drone needs to perform two sharp corners to arrive to the end position.

3) A warehouse with a wall with a gap in it and extra
corners thereafter. In this environment, the drone needs to navigate through the hole in the wall and around a corner to find the end position.


