import pkg_resources
import pybullet as p

def create_warehouse_skeleton(env):
    ### OUR CREATED OBSTACLES ###

    floor = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/floor.urdf'),
                [0, 0, -.01],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    # ceiling = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/floor.urdf'),
    #             [0, 0, 10],
    #             p.getQuaternionFromEuler([0,0,0]),
    #             useFixedBase=True,
    #             physicsClientId=env.CLIENT
    #             )
    
    #pillars
    pillar_1 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/pillar.urdf'),
                [20, 20, 5],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    pillar_1 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/pillar.urdf'),
                [20, 20, 5],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    pillar_2 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/pillar.urdf'),
                [-20, 20, 5],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    pillar_3 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/pillar.urdf'),
                [20, -20, 5],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    pillar_4 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/pillar.urdf'),
                [-20, -20, 5],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    return [floor]


def create_boxes_1(env):
    ### Creates 2 boxes in the warehouse ###
    base_warehouse = create_warehouse_skeleton(env)
    # spheres = create_sphere(env)
    box_1 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box.urdf'),
                [-2, 2, 1.5],
                p.getQuaternionFromEuler([0,0,0]),
                physicsClientId=env.CLIENT
                )
    box_2 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box.urdf'),
                [3, 2, 1.5],
                p.getQuaternionFromEuler([0,0,0]),
                physicsClientId=env.CLIENT
                )
    return base_warehouse + [box_1, box_2]

def create_boxes_2(env):
    ### Creates 3 boxes in the warehouse ###
    base_warehouse = create_warehouse_skeleton(env)

    box_1 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box_20x2x9.urdf'),
                [0, 1.5, 4.5],
                p.getQuaternionFromEuler([0,0,0]),
                physicsClientId=env.CLIENT
                )
    box_2 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box_20x2x9.urdf'),
                [0, -1.5, 4.5],
                p.getQuaternionFromEuler([0,0,0]),
                physicsClientId=env.CLIENT
                )
    box_3 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box_1x2x9.urdf'),
                [9.5, -3.5, 4.5],
                p.getQuaternionFromEuler([0,0,0]),
                physicsClientId=env.CLIENT
                )
    return base_warehouse + [box_1, box_2, box_3]

def create_boxes_3(env):
    ### Creates 6 boxes in the warehouse ###
    base_warehouse = create_warehouse_skeleton(env)

    box_1 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box_20x2x2.urdf'),
                [2.5, -3, 1],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                #useFixedBase=True,
                )
    box_2 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box_20x2x2.urdf'),
                [2.5, -3, 5],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    box_3 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box_7x2x2.urdf'),
                [-4, -3, 3],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    box_4 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box_7x2x2.urdf'),
                [9, -3, 3],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    box_5 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box_5x1x5.urdf'),
                [9.5, -6, 2.5],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    box_6 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box_1x5x5.urdf'),
                [6.5, -8, 2.5],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    box_7 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box_6x2x1.urdf'),
                [2.5, -3, 4],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    box_8 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box_1x20x9.urdf'),
                [-1, -14, 4],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    box_9 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box_1x20x9.urdf'),
                [9, 8, 4],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    # box_10 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box_1x20x9.urdf'),
    #             [12, -12, 4],
    #             p.getQuaternionFromEuler([0,0,0]),
                
    #             physicsClientId=env.CLIENT
    #             )
    return base_warehouse + [box_1, box_2, box_3, box_4, box_5, box_6, box_7, box_8, box_9] # box_10

def create_sphere(env):
    ### Creates 2 boxes in the warehouse ###
    base_warehouse = create_warehouse_skeleton(env)
    sphere_1 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/sphere.urdf'),
                [0,0,2],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )

    return [sphere_1]



def get_object_collision_shape_type(object_id):
        dynamics_info = p.getDynamicsInfo(object_id, -1)  # -1 refers to the base link of the object

        # Extract the collision shape type
        collision_shape_type = dynamics_info['collision_shape_type']

        return collision_shape_type