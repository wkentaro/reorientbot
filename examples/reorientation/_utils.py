import itertools

import numpy as np
import pybullet as p
import pybullet_planning as pp
import trimesh

import reorientbot


def get_class_id(object_id):
    visual_shape_data = p.getVisualShapeData(object_id)
    class_name = visual_shape_data[0][4].decode().split("/")[-2]
    class_id = reorientbot.datasets.ycb.class_names.tolist().index(class_name)
    return class_id


def get_canonical_quaternion(class_id):
    c = reorientbot.geometry.Coordinate()
    if class_id == 2:
        c.rotate([0, 0, np.deg2rad(0)])
    elif class_id == 3:
        c.rotate([0, 0, np.deg2rad(5)])
    elif class_id == 5:
        c.rotate([0, 0, np.deg2rad(-65)])
    elif class_id == 11:
        c.rotate([0, 0, np.deg2rad(47)])
    elif class_id == 12:
        c.rotate([0, 0, np.deg2rad(90)])
    elif class_id == 15:
        c.rotate([0, np.deg2rad(90), np.deg2rad(90)])
    else:
        pass
    return c.quaternion


cached_mesh = {}


def get_aabb(obj):
    class_id = get_class_id(obj)
    if class_id not in cached_mesh:
        visual_file = reorientbot.datasets.ycb.get_visual_file(class_id=class_id)
        cached_mesh[class_id] = trimesh.load_mesh(visual_file)

    visual = cached_mesh[class_id].copy()
    obj_to_world = pp.get_pose(obj)
    visual.apply_transform(
        reorientbot.geometry.transformation_matrix(*obj_to_world)
    )
    return visual.bounds


def create_shelf(X, Y, Z, N=3):
    T = 0.01
    color = (0.8, 0.8, 0.8, 1)

    def get_parts(origin, X, Y, Z, T):
        extents = np.array(
            [
                [X, Y, T],
                [X, Y, T],
                [X, T, Z],
                [X, T, Z],
                # [T, Y, Z],
                [T, Y, Z],
            ]
        )
        positions = (
            np.array(
                [
                    [0, 0, Z / 2],
                    [0, 0, -Z / 2],
                    [0, Y / 2, 0],
                    [0, -Y / 2, 0],
                    # [X / 2, 0, 0],
                    [-X / 2, 0, 0],
                ]
            )
            + origin
        )
        return extents, positions

    extents = []
    positions = []
    for i in range(N):
        origin = [0, 0, T + Z / 2 + i * (T + Z)]
        parts = get_parts(origin, X, Y, Z, T)
        extents.extend(parts[0])
        positions.extend(parts[1])

    halfExtents = np.array(extents) / 2
    shapeTypes = [p.GEOM_BOX] * len(extents)
    rgbaColors = [color] * len(extents)
    visual_shape_id = p.createVisualShapeArray(
        shapeTypes=shapeTypes,
        halfExtents=halfExtents,
        visualFramePositions=positions,
        rgbaColors=rgbaColors,
    )
    collision_shape_id = p.createCollisionShapeArray(
        shapeTypes=shapeTypes,
        halfExtents=halfExtents,
        collisionFramePositions=positions,
    )

    position = [0, 0, 0]
    quaternion = [0, 0, 0, 1]
    unique_id = p.createMultiBody(
        baseMass=0,
        basePosition=position,
        baseOrientation=quaternion,
        baseVisualShapeIndex=visual_shape_id,
        baseCollisionShapeIndex=collision_shape_id,
        baseInertialFramePosition=[0, 0, 0],
        baseInertialFrameOrientation=[0, 0, 0, 1],
    )
    return unique_id


def init_place_scene(env, class_id, random_state, face="front"):
    lock_renderer = pp.LockRenderer()

    place_aabb_extents = [0.25, 0.6, 0.3]

    shelf = create_shelf(*place_aabb_extents)

    place_aabb = (
        np.array([[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]]) * place_aabb_extents
    )
    place_aabb_offset = np.array([0, 0, 2.5]) * place_aabb_extents
    place_aabb_offset += [0, 0, 0.01 * 3]
    place_aabb += place_aabb_offset
    pp.draw_aabb(place_aabb, width=5, color=(1, 0, 0, 1), parent=shelf)

    visual_file = reorientbot.datasets.ycb.get_visual_file(class_id=class_id)

    c = reorientbot.geometry.Coordinate(
        quaternion=get_canonical_quaternion(class_id=class_id)
    )
    if face == "front":
        c.rotate([0, 0, 0], wrt="world")
    elif face == "right":
        c.rotate([0, 0, np.deg2rad(90)], wrt="world")
    elif face == "left":
        c.rotate([0, 0, np.deg2rad(-90)], wrt="world")
    elif face == "back":
        c.rotate([0, 0, np.deg2rad(180)], wrt="world")
    else:
        raise ValueError
    canonical_quaternion = c.quaternion

    # find the initial corner
    obj = reorientbot.pybullet.create_mesh_body(
        visual_file=visual_file,
        collision_file=reorientbot.pybullet.get_collision_file(visual_file),
        quaternion=canonical_quaternion,
    )
    aabb_min, aabb_max = get_aabb(obj)
    for x, y, z in itertools.product(
        np.linspace(-0.5, 0.5, num=50),
        np.linspace(-0.5, 0.5, num=50),
        np.linspace(-0.5, -0.4, num=5),
    ):
        position = np.array([x, y, z]) * place_aabb_extents + place_aabb_offset
        obj_to_world = (position - [0, 0, aabb_min[2]], canonical_quaternion)
        pp.set_pose(obj, obj_to_world)
        if not reorientbot.pybullet.is_colliding(
            obj, [shelf]
        ) and pp.aabb_contains_aabb(get_aabb(obj), place_aabb):
            pp.remove_body(obj)
            break

    # spawn all objects
    aabb_extents = aabb_max - aabb_min + 0.01
    ixs = []
    objects = []
    for iy in itertools.count():
        for ix in itertools.count():
            obj = reorientbot.pybullet.create_mesh_body(
                visual_file=visual_file,
                collision_file=reorientbot.pybullet.get_collision_file(
                    visual_file
                ),
            )
            c = reorientbot.geometry.Coordinate(*obj_to_world)
            c.translate(
                [aabb_extents[0] * ix, aabb_extents[1] * iy, 0], wrt="world"
            )
            pp.set_pose(obj, c.pose)
            if pp.aabb_contains_aabb(get_aabb(obj), place_aabb):
                ixs.append(ix)
                objects.append(obj)
            else:
                pp.remove_body(obj)
                break
        if ix == 0:
            break
    ixs = np.array(ixs)

    stop_index = random_state.choice(np.where(ixs == ixs.max())[0])
    for obj in objects[stop_index + 1 :]:
        pp.remove_body(obj)
    objects = objects[: stop_index + 1]

    # apply transform
    c = reorientbot.geometry.Coordinate()
    c.rotate([0, 0, np.deg2rad(-90)])
    c.translate([0, 0.7, env.TABLE_OFFSET], wrt="world")
    shelf_to_world = c.pose
    for obj in [shelf] + objects:
        obj_to_shelf = pp.get_pose(obj)
        obj_to_world = pp.multiply(shelf_to_world, obj_to_shelf)
        pp.set_pose(obj, obj_to_world)

    place_pose = pp.get_pose(objects[-1])
    pp.remove_body(objects[-1])
    reorientbot.pybullet.create_mesh_body(
        visual_file=visual_file,
        position=place_pose[0],
        quaternion=place_pose[1],
        rgba_color=[1, 1, 1, 0.5],
        # for virtual rendering, it must be smaller than env.fg_object_id
        mesh_scale=[0.99, 0.99, 0.99],
    )
    mesh = trimesh.load_mesh(visual_file)
    mesh.apply_transform(reorientbot.geometry.transformation_matrix(*place_pose))
    aabb = (mesh.vertices.min(axis=0), mesh.vertices.max(axis=0))
    pp.draw_aabb(aabb, width=2, color=(0, 1, 0, 1))

    lock_renderer.restore()

    return shelf, place_pose
