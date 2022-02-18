import numpy as np
import pybullet as p
import pybullet_planning as pp

import reorientbot


def init_simulation(camera_distance=1.5):
    pp.add_data_path()
    p.setGravity(0, 0, -9.8)

    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=90,
        cameraPitch=-60,
        cameraTargetPosition=(0, 0, 0),
    )

    plane = p.loadURDF("plane.urdf")
    return plane


def create_pile(class_ids, num_instances, random_state=None):
    if random_state is None:
        random_state = np.random.RandomState()

    x = (-0.2, 0.2)
    y = (-0.2, 0.2)
    z = 0.5

    bin_unique_id = reorientbot.pybullet.create_bin(
        X=x[1] - x[0],
        Y=y[1] - y[0],
        Z=z / 2,
    )

    unique_ids = []
    class_ids = random_state.choice(class_ids, num_instances).tolist()
    while class_ids:
        class_id = class_ids.pop()

        position = random_state.uniform([x[0], y[0], z], [x[1], y[1], z])
        quaternion = random_state.random((4,))
        quaternion /= np.linalg.norm(quaternion)

        coord = reorientbot.geometry.Coordinate(position, quaternion=quaternion)

        visual_file = reorientbot.datasets.ycb.get_visual_file(class_id=class_id)
        collision_file = reorientbot.pybullet.get_collision_file(visual_file)
        unique_id = reorientbot.pybullet.create_mesh_body(
            visual_file=visual_file,
            collision_file=collision_file,
            mass=reorientbot.datasets.ycb.masses[class_id],
            position=coord.position,
            quaternion=coord.quaternion,
        )
        p.addUserData(unique_id, "class_id", str(class_id))

        for _ in range(1000):
            p.stepSimulation()
            if np.linalg.norm(p.getBaseVelocity(unique_id)[0]) < 1e-12:
                break

        aabb_min, aabb_max = p.getAABB(bin_unique_id)

        position, _ = p.getBasePositionAndOrientation(unique_id)
        if not (
            (aabb_min[0] < position[0] < aabb_max[0])
            and (aabb_min[1] < position[1] < aabb_max[1])
        ):
            p.removeBody(unique_id)
            class_ids.append(class_id)
        else:
            unique_ids.append(unique_id)

    for _ in range(250):
        coord = reorientbot.geometry.Coordinate(*pp.get_pose(bin_unique_id))
        coord.translate([0, 0, -0.001], wrt="world")
        pp.set_pose(bin_unique_id, coord.pose)
        for _ in range(100):
            p.stepSimulation()
            if all(
                np.linalg.norm(p.getBaseVelocity(unique_id)[0]) < 1e-12
                for unique_id in reorientbot.pybullet.get_body_unique_ids()
            ):
                break
    p.removeBody(bin_unique_id)

    return unique_ids
