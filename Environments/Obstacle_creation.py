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
    ceiling = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/floor.urdf'),
                [0, 0, 10],
                p.getQuaternionFromEuler([0,0,0]),
                useFixedBase=True,
                physicsClientId=env.CLIENT
                )
    
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
    return [floor, ceiling, pillar_1, pillar_2, pillar_3, pillar_4]


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