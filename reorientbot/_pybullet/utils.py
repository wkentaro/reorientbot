import contextlib
import itertools
import shlex
import subprocess
import time

import numpy as np
import path
import pybullet as p
import pybullet_planning as pp

from .. import geometry


def init_world(*args, **kwargs):
    p.connect(p.GUI)
    pp.add_data_path()
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)


def get_body_unique_ids():
    num_bodies = p.getNumBodies()
    unique_ids = [p.getBodyUniqueId(i) for i in range(num_bodies)]
    return unique_ids


def create_mesh_body(
    visual_file=None,
    collision_file=None,
    position=None,
    quaternion=None,
    mass=0,
    rgba_color=None,
    texture=True,
    mesh_scale=(1, 1, 1),
):
    assert position is None or len(position) == 3
    assert quaternion is None or len(quaternion) == 4
    if rgba_color is not None and len(rgba_color) == 3:
        rgba_color = [rgba_color[0], rgba_color[1], rgba_color[2], 1]
    if visual_file is None:
        visual_shape_id = -1
    else:
        if visual_file is True:
            # visual_file from collision_file
            visual_file = collision_file
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=visual_file,
            visualFramePosition=[0, 0, 0],
            meshScale=mesh_scale,
            rgbaColor=rgba_color,
        )
    if collision_file is None:
        collision_shape_id = -1
    else:
        if collision_file is True:
            # collision_file from visual_file
            collision_file = get_collision_file(visual_file)
        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=collision_file,
            collisionFramePosition=[0, 0, 0],
            meshScale=mesh_scale,
        )
    unique_id = p.createMultiBody(
        baseMass=mass,
        baseInertialFramePosition=[0, 0, 0],
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=position,
        baseOrientation=quaternion,
        useMaximalCoordinates=False,
    )
    if not texture:
        p.changeVisualShape(unique_id, -1, textureUniqueId=-1)
    if collision_file:
        p.addUserData(unique_id, "collision_file", collision_file)
    return unique_id


def get_collision_file(visual_file, resolution=200000):
    visual_file = path.Path(visual_file)
    collision_file = visual_file.stripext() + ".convex" + visual_file.ext
    if not collision_file.exists():
        cmd = (
            f"testVHACD --input {visual_file} --output {collision_file}"
            " --log /tmp/testVHACD.log --resolution {resolution}"
        )
        subprocess.check_output(shlex.split(cmd))
    return collision_file


def get_debug_visualizer_image():
    width, height, *_ = p.getDebugVisualizerCamera()
    _, _, rgba, depth, segm = p.getCameraImage(
        width=width,
        height=height,
        renderer=p.ER_BULLET_HARDWARE_OPENGL,
    )
    rgb = rgba[:, :, :3]
    depth[segm == -1] = np.nan
    return rgb, depth, segm


def get_aabb(unique_id):
    aabb_min, aabb_max = p.getAABB(unique_id)
    return np.array(aabb_min), np.array(aabb_max)


def is_colliding(id1, ids2=None, distance=0):
    if ids2 is None:
        ids2 = np.array(get_body_unique_ids())
        ids2 = ids2[ids2 != id1]
    is_colliding = False
    for id2 in ids2:
        points = p.getClosestPoints(id1, id2, distance=distance)
        if points:
            is_colliding = True
            break
    return is_colliding


def get_pose(obj, parent=None):
    obj_to_world = pp.get_pose(obj)

    if parent is None:
        obj_to_parent = obj_to_world
    else:
        parent_to_world = pp.get_pose(parent)
        world_to_parent = pp.invert(parent_to_world)
        obj_to_parent = pp.multiply(world_to_parent, obj_to_world)
    return obj_to_parent


def set_pose(obj, pose, parent=None):
    obj_to_parent = pose

    if parent is None:
        obj_to_world = obj_to_parent
    else:
        parent_to_world = pp.get_pose(parent)
        obj_to_world = pp.multiply(parent_to_world, obj_to_parent)
    pp.set_pose(obj, obj_to_world)


def step_and_sleep(seconds=np.inf):
    for i in itertools.count():
        p.stepSimulation()
        time.sleep(pp.get_time_step())
        if int(round(i * pp.get_time_step())) >= seconds:
            break


def get_camera_image(
    T_cam2world,
    fovy,
    height,
    width,
    far=1000,
    near=0.01,
):
    # T_cam2world -> view_matrix
    view_matrix = T_cam2world.copy()
    view_matrix[:3, 3] = 0
    view_matrix[3, :3] = np.linalg.inv(T_cam2world)[:3, 3]
    view_matrix[:, 1] *= -1
    view_matrix[:, 2] *= -1
    view_matrix = view_matrix.flatten()

    projection_matrix = p.computeProjectionMatrixFOV(
        fov=np.rad2deg(fovy),
        aspect=1.0 * width / height,
        farVal=far,
        nearVal=near,
    )
    _, _, rgba, depth, segm = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_TINY_RENDERER,
    )
    rgb = rgba[:, :, :3]
    depth = np.asarray(depth, dtype=np.float32).reshape(height, width)
    depth = far * near / (far - (far - near) * depth)
    depth[segm == -1] = np.nan
    return rgb, depth, segm


def draw_camera(
    fovy,
    height,
    width,
    pose=None,
    marker_height=0.1,
    marker_color=(0, 0.9, 0.9),
    marker_width=2,
    **kwargs,
):
    aspect_ratio = width / height
    fovx = 2 * np.arctan(np.tan(fovy * 0.5) * aspect_ratio)

    x = marker_height * np.tan(fovx * 0.5)
    y = marker_height * np.tan(fovy * 0.5)
    z = marker_height

    # combine the points into the vertices of an FOV visualization
    points = np.array(
        [(0, 0, 0), (-x, -y, z), (x, -y, z), (x, y, z), (-x, y, z)],
        dtype=float,
    )

    # create line segments for the FOV visualization
    # a segment from the origin to each bound of the FOV
    segments = np.column_stack((np.zeros_like(points), points)).reshape(
        (-1, 3)
    )

    # add a loop for the outside of the FOV then reshape
    # the whole thing into multiple line segments
    segments = np.vstack((segments, points[[1, 2, 2, 3, 3, 4, 4, 1]])).reshape(
        (-1, 2, 3)
    )

    if pose is not None:
        segments = segments.reshape(-1, 3)
        segments = geometry.transform_points(
            segments, geometry.transformation_matrix(*pose)
        )
        segments = segments.reshape(-1, 2, 3)

    lines = []
    for segment in segments:
        lines.append(
            pp.add_line(
                segment[0],
                segment[1],
                color=marker_color,
                width=marker_width,
                **kwargs,
            )
        )
    return lines


def duplicate(
    body_id,
    visual=True,
    collision=True,
    position=None,
    quaternion=None,
    mass=None,
    **kwargs,
):
    if visual:
        visual_data = p.getVisualShapeData(body_id)
        assert len(visual_data) == 1
        visual_file = visual_data[0][4].decode()
    else:
        visual_file = None

    if collision:
        collision_file = p.getUserData(
            p.getUserDataId(body_id, "collision_file")
        ).decode()
    else:
        collision_file = None

    if position is None:
        position = pp.get_pose(body_id)[0]

    if quaternion is None:
        quaternion = pp.get_pose(body_id)[1]

    if mass is None:
        mass = pp.get_dynamics_info(body_id).mass

    return create_mesh_body(
        visual_file=visual_file,
        collision_file=collision_file,
        position=position,
        quaternion=quaternion,
        mass=mass,
        **kwargs,
    )


@contextlib.contextmanager
def stash_objects(object_ids):
    try:
        with pp.LockRenderer(), pp.WorldSaver():
            for obj in object_ids:
                pp.set_pose(obj, ((0, 0, 1000), (0, 0, 0, 1)))
            yield
    finally:
        pass


def pause():
    print(
        """
Usage:
\tn: continue
\tc: print camera pose
"""
    )
    while True:
        events = p.getKeyboardEvents()
        if events.get(ord("n")) == p.KEY_WAS_RELEASED:
            break
        elif events.get(ord("c")) == p.KEY_WAS_RELEASED:
            camera = pp.get_camera()
            print(
                f"""
p.resetDebugVisualizerCamera(
    cameraYaw={camera.yaw},
    cameraPitch={camera.pitch},
    cameraDistance={camera.dist},
    cameraTargetPosition={camera.target},
)
"""
            )


def annotate_pose(obj, parent=None):
    print("Press keys to annotate pose of an object.")
    while True:
        events = p.getKeyboardEvents()

        dp = 0.0001
        dr = 0.001

        sign = 1
        if events.get(65306) == p.KEY_IS_DOWN:  # SHIFT
            sign = -1
        if events.get(65307) == p.KEY_IS_DOWN:  # CTRL
            dp *= 10
            dr *= 10

        c = geometry.Coordinate(*pp.get_pose(obj))
        if events.get(ord("k")) == p.KEY_IS_DOWN:
            c.translate([dp, 0, 0], wrt="world")
        elif events.get(ord("j")) == p.KEY_IS_DOWN:
            c.translate([-dp, 0, 0], wrt="world")
        elif events.get(ord("l")) == p.KEY_IS_DOWN:
            c.translate([0, -dp, 0], wrt="world")
        elif events.get(ord("h")) == p.KEY_IS_DOWN:
            c.translate([0, dp, 0], wrt="world")
        elif events.get(ord("i")) == p.KEY_IS_DOWN:
            c.translate([0, 0, dp], wrt="world")
        elif events.get(ord("m")) == p.KEY_IS_DOWN:
            c.translate([0, 0, -dp], wrt="world")
        elif events.get(ord("1")) == p.KEY_IS_DOWN:
            c.rotate([dr * sign, 0, 0], wrt="world")
        elif events.get(ord("2")) == p.KEY_IS_DOWN:
            c.rotate([0, dr * sign, 0], wrt="world")
        elif events.get(ord("3")) == p.KEY_IS_DOWN:
            c.rotate([0, 0, dr * sign], wrt="world")
        elif events.get(ord("c")) == p.KEY_WAS_RELEASED:
            camera = pp.get_camera()
            print(
                f"""
p.resetDebugVisualizerCamera(
    cameraYaw={camera.yaw},
    cameraPitch={camera.pitch},
    cameraDistance={camera.dist},
    cameraTargetPosition={camera.target},
)
"""
            )
        elif events.get(ord("p")) == p.KEY_WAS_RELEASED:
            pose = get_pose(obj, parent=parent)
            if parent is None:
                print(f"mercury.pybullet.set_pose(obj, {pose})")
            else:
                print(f"mercury.pybullet.set_pose(obj, {pose}, parent=parent)")
        elif events.get(ord("q")) == p.KEY_WAS_RELEASED:
            break
        pp.set_pose(obj, c.pose)

        time.sleep(1 / 240)


def draw_points(points, colors=None, size=1):
    points = np.asarray(points)

    if colors is None:
        colors = np.full_like(points, (0.3, 0.7, 0.3), dtype=float)
    else:
        colors = np.asarray(colors)

    if colors.ndim == 1:
        colors = np.repeat(colors[None, :], points.shape[0], axis=0)

    if colors.dtype == np.uint8:
        colors = colors.astype(np.float32) / 255

    assert points.shape[-1] == 3
    assert colors.shape[-1] == 3

    points = points.reshape(-1, 3)
    colors = colors.reshape(-1, 3)

    mask = ~np.isnan(points).any(axis=1)
    points = points[mask]
    colors = colors[mask]

    N = len(points)
    assert len(colors) == N

    MAX_NUM_POINTS = 130000
    if N > MAX_NUM_POINTS:
        i = np.random.permutation(N)[:MAX_NUM_POINTS]
    else:
        i = Ellipsis
    return p.addUserDebugPoints(points[i], colors[i], pointSize=size)
