import pkg_resources

def create_warehouse_skeleton(p, env):
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
    # Ceiling


def create_boxes_1(p, env):
    create_warehouse_skeleton(p, env)
    p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box.urdf'),
                [-2, 2, 1.5],
                p.getQuaternionFromEuler([0,0,0]),
                physicsClientId=env.CLIENT
                )
    p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'planning-and-decision-making/assets/box.urdf'),
                [3, 2, 1.5],
                p.getQuaternionFromEuler([0,0,0]),
                physicsClientId=env.CLIENT
                )
    

