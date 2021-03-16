import itertools
import shlex
import subprocess
import time

import numpy as np
import path
import pybullet
import pybullet_data
import pybullet_planning


def init_world(*args, **kwargs):
    pybullet.connect(pybullet.GUI)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.loadURDF("plane.urdf")
    pybullet.setGravity(0, 0, -9.8)


def get_body_unique_ids():
    num_bodies = pybullet.getNumBodies()
    unique_ids = [pybullet.getBodyUniqueId(i) for i in range(num_bodies)]
    return unique_ids


def create_mesh_body(
    visual_file=None,
    collision_file=None,
    position=None,
    quaternion=None,
    mass=0,
    rgba_color=None,
):
    if rgba_color is not None and len(rgba_color) == 3:
        rgba_color = [rgba_color[0], rgba_color[1], rgba_color[2], 1]
    if visual_file is not None:
        visual_shape_id = pybullet.createVisualShape(
            shapeType=pybullet.GEOM_MESH,
            fileName=visual_file,
            visualFramePosition=[0, 0, 0],
            meshScale=[1, 1, 1],
            rgbaColor=rgba_color,
        )
    else:
        visual_shape_id = -1
    if collision_file is not None:
        collision_shape_id = pybullet.createCollisionShape(
            shapeType=pybullet.GEOM_MESH,
            fileName=collision_file,
            collisionFramePosition=[0, 0, 0],
            meshScale=[1, 1, 1],
        )
    else:
        collision_shape_id = -1
    unique_id = pybullet.createMultiBody(
        baseMass=mass,
        baseInertialFramePosition=[0, 0, 0],
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=position,
        baseOrientation=quaternion,
        useMaximalCoordinates=False,
    )
    return unique_id


def get_collision_file(visual_file):
    visual_file = path.Path(visual_file)
    collision_file = visual_file.stripext() + ".convex" + visual_file.ext
    if not collision_file.exists():
        cmd = (
            f"testVHACD --input {visual_file} --output {collision_file}"
            " --log /tmp/testVHACD.log --resolution 200000"
        )
        subprocess.check_output(shlex.split(cmd))
    return collision_file


def get_debug_visualizer_image():
    width, height, *_ = pybullet.getDebugVisualizerCamera()
    _, _, rgba, depth, segm = pybullet.getCameraImage(
        width=width, height=height
    )
    rgb = rgba[:, :, :3]
    depth[segm == -1] = np.nan
    return rgb, depth, segm


def get_aabb(unique_id):
    aabb_min, aabb_max = pybullet.getAABB(unique_id)
    return np.array(aabb_min), np.array(aabb_max)


def is_colliding(id1, ids2=None):
    if ids2 is None:
        ids2 = np.array(get_body_unique_ids())
        ids2 = ids2[ids2 != id1]
    is_colliding = False
    for id2 in ids2:
        points = pybullet.getClosestPoints(id1, id2, distance=0)
        if points:
            is_colliding = True
    return is_colliding


def get_pose(body_id, link_id=-1, parent_body_id=None, parent_link_id=-1):
    self_to_world = pybullet_planning.get_link_pose(body_id, link_id)

    if parent_body_id is None:
        self_to_parent = self_to_world
    else:
        parent_to_world = pybullet_planning.get_link_pose(
            parent_body_id, parent_link_id
        )
        world_to_parent = pybullet.invertTransform(
            parent_to_world[0], parent_to_world[1]
        )
        self_to_parent = pybullet.multiplyTransforms(
            world_to_parent[0],
            world_to_parent[1],
            self_to_world[0],
            self_to_world[1],
        )
    return self_to_parent


def step_and_sleep(seconds=np.inf):
    for i in itertools.count():
        pybullet.stepSimulation()
        time.sleep(1 / 240)
        if int(round(i / 240)) >= seconds:
            break