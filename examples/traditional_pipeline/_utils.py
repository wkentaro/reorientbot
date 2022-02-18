import itertools
import pickle
import time

import imgviz
from loguru import logger
import numpy as np
import pybullet as p
import pybullet_planning as pp

import reorientbot


def get_camera_pose(camera_config):
    c_cam_to_ee = reorientbot.geometry.Coordinate()

    if camera_config == 0:
        c_cam_to_ee.translate([0, -0.05, -0.1])
    elif camera_config == 1:
        c_cam_to_ee.rotate([np.deg2rad(-15), 0, 0])
        c_cam_to_ee.translate([0, -0.08, -0.2])
    elif camera_config == 2:
        c_cam_to_ee.rotate([np.deg2rad(-15), 0, 0])
        c_cam_to_ee.translate([0, -0.08, -0.35])
    else:
        raise ValueError

    return c_cam_to_ee.pose


class StepSimulation:
    def __init__(self, ri, imshow=False, retime=1, video_dir=None):
        self.ri = ri
        self._imshow = imshow
        self._retime = retime
        self._video_dir = video_dir

        self.i = 0

    def __call__(self):
        p.stepSimulation()
        self.ri.step_simulation()
        if self._video_dir and self.i % (8 * self._retime) == 0:
            debug_rgb, _, _ = reorientbot.pybullet.get_debug_visualizer_image()

            rgb, depth, _ = self.ri.get_camera_image()
            depth[(depth < 0.3) | (depth > 2)] = np.nan
            tiled = imgviz.tile(
                [
                    rgb,
                    imgviz.depth2rgb(depth, min_value=0.3, max_value=0.6),
                ],
                border=(255, 255, 255),
            )
            if self._imshow:
                imgviz.io.cv_imshow(tiled, "wrist_camera")
                imgviz.io.cv_waitkey(10)

            tiled = imgviz.resize(tiled, height=debug_rgb.shape[0])
            tiled = np.hstack(
                [
                    debug_rgb,
                    np.full((debug_rgb.shape[0], 10, 3), 255, dtype=np.uint8),
                    tiled,
                ]
            )
            imgviz.io.imsave(self._video_dir / f"{self.i:08d}.jpg", tiled)
        else:
            time.sleep(pp.get_time_step() / self._retime)
        self.i += 1


def place_to_regrasp(
    ri, regrasp_aabb, bg_object_ids, object_ids, step_simulation
):
    n_trial = 5

    object_id = ri.attachments[0].child
    class_id = get_class_id(object_id)

    t_start = time.time()
    for i in itertools.count():
        with pp.LockRenderer(), pp.WorldSaver():
            quaternion = get_canonical_quaternion(class_id=class_id)
            if i >= n_trial:
                c = reorientbot.geometry.Coordinate(quaternion=quaternion)
                euler = [
                    [np.deg2rad(90), 0, 0],
                    [np.deg2rad(-90), 0, 0],
                    [0, np.deg2rad(90), 0],
                    [0, np.deg2rad(-90), 0],
                ][np.random.randint(0, 4)]
                c.rotate(euler, wrt="world")
                quaternion = c.quaternion

            pp.set_pose(object_id, ([0, 0, 0], quaternion))
            aabb = pp.get_aabb(object_id)
            position = np.random.uniform(*regrasp_aabb)
            position[2] -= aabb[0][2]
            c = reorientbot.geometry.Coordinate(position, quaternion)

        regrasp_pose = c.pose  # obj_af_to_world

        with ri.enabling_attachments():
            obj_to_world = ri.get_pose("attachment_link0")

            move_target_to_world = reorientbot.geometry.Coordinate(*obj_to_world)
            move_target_to_world.transform(
                np.linalg.inv(
                    reorientbot.geometry.quaternion_matrix(regrasp_pose[1])
                ),
                wrt="local",
            )
            move_target_to_world = move_target_to_world.pose
            regrasp_pose = regrasp_pose[0], [0, 0, 0, 1]

            ee_to_world = ri.get_pose("tipLink")
            move_target_to_ee = pp.multiply(
                pp.invert(ee_to_world), move_target_to_world
            )
            ri.add_link("move_target", pose=move_target_to_ee)

            j = ri.solve_ik(
                regrasp_pose,
                move_target=ri.robot_model.move_target,
                rotation_axis="z",
            )
        if j is None:
            logger.warning("j is None")
            continue

        obstacles = bg_object_ids + object_ids
        obstacles.remove(object_id)
        path = ri.planj(j, obstacles=obstacles)
        if path is None:
            logger.warning("path is None")
            continue

        with pp.LockRenderer():
            with pp.WorldSaver():
                ri.setj(path[-1])
                c = reorientbot.geometry.Coordinate(*ri.get_pose("tipLink"))
                c.translate([0, 0, -0.05])
                j = ri.solve_ik(c.pose, rotation_axis=None)
                path2 = ri.planj(j)
        if path2 is None:
            logger.warning("path2 is None")
            continue

        break
    planning_time = time.time() - t_start
    for _ in (_ for j in path for _ in ri.movej(j, speed=0.005)):
        step_simulation()

    for _ in range(240):
        step_simulation()

    ri.ungrasp()

    for _ in range(240):
        step_simulation()

    for _ in (_ for j in path2 for _ in ri.movej(j, speed=0.005)):
        step_simulation()

    for _ in ri.move_to_homej(bg_object_ids, object_ids):
        step_simulation()

    return regrasp_pose, i < n_trial, planning_time


def get_place_pose(object_id, bin_aabb_min, bin_aabb_max):
    position_org, quaternion_org = p.getBasePositionAndOrientation(object_id)

    class_id = get_class_id(object_id)
    quaternion = get_canonical_quaternion(class_id)

    with pp.LockRenderer():
        with pp.WorldSaver():
            p.resetBasePositionAndOrientation(
                object_id, position_org, quaternion
            )

            aabb_min, aabb_max = np.array(p.getAABB(object_id))
            position_lt = bin_aabb_min - (aabb_min - position_org)
            position_rb = bin_aabb_max + (aabb_min - position_org)

            with pp.LockRenderer():
                z = position_lt[2]
                for x in np.linspace(position_lt[0], position_rb[0]):
                    for y in np.linspace(position_lt[1], position_rb[1]):
                        position = (x, y, z)
                        pp.set_pose(object_id, (position, quaternion))
                        if not reorientbot.pybullet.is_colliding(object_id):
                            break
                    else:
                        continue
                    break
                else:
                    position, quaternion = None, None

    return position, quaternion


def plan_placement(ri, place_aabb, bg_object_ids, object_ids):
    object_id = ri.attachments[0].child
    place_pose = get_place_pose(
        object_id=object_id,
        bin_aabb_min=place_aabb[0],
        bin_aabb_max=place_aabb[1],
    )
    with ri.enabling_attachments():
        j = ri.solve_ik(
            place_pose,
            move_target=ri.robot_model.attachment_link0,
        )
    if j is None:
        logger.warning("j is None")
        return place_pose, None

    obstacles = bg_object_ids + object_ids
    obstacles.remove(object_id)
    path = ri.planj(j, obstacles=obstacles)
    if path is None:
        logger.warning("path is None")
        return place_pose, None

    return place_pose, path


virtual_objects = []


def place(
    ri, object_id, place_pose, path, bg_object_ids, object_ids, step_simulation
):
    obj_v = reorientbot.pybullet.duplicate(
        object_id,
        collision=False,
        rgba_color=[0, 1, 0, 0.5],
        position=place_pose[0],
        quaternion=place_pose[1],
        texture=False,
        mass=0,
    )
    virtual_objects.append(obj_v)

    for _ in (_ for j in path for _ in ri.movej(j, speed=0.005)):
        step_simulation()

    for _ in range(240):
        step_simulation()

    ri.ungrasp()

    for _ in range(240):
        step_simulation()

    c = reorientbot.geometry.Coordinate(*ri.get_pose("tipLink"))
    c.translate([0, 0, -0.05])
    j = ri.solve_ik(c.pose, rotation_axis=None)
    for _ in ri.movej(j, speed=0.005):
        step_simulation()

    min_distance = 0
    path = None
    while path is None:
        min_distances = {}
        for attachment in ri.attachments:
            min_distances[(attachment.child, -1)] = min_distance

        path = ri.planj(
            ri.homej,
            obstacles=bg_object_ids + object_ids,
            min_distances=min_distances,
        )
        min_distance -= 0.01
    for _ in (_ for j in path for _ in ri.movej(j)):
        step_simulation()


def get_class_id(object_id):
    visual_shape_data = p.getVisualShapeData(object_id)
    class_name = visual_shape_data[0][4].decode().split("/")[-2]
    class_id = reorientbot.datasets.ycb.class_names.tolist().index(class_name)
    return class_id


def correct(
    ri,
    object_id,
    place_pose,
    bg_object_ids,
    object_ids,
    step_simulation,
    auc_threshold=0.5,
):
    c = reorientbot.geometry.Coordinate(*ri.get_pose("tipLink"))
    c.position = place_pose[0]
    c.position[2] = 0.7
    j = ri.solve_ik(
        c.pose,
        move_target=ri.robot_model.camera_link,
        rotation_axis="z",
    )
    if j is None:
        logger.warning("j is None")
        return
    for _ in ri.movej(j):
        step_simulation()
    j_camera = j

    while True:
        obj_to_world = pp.get_pose(object_id)

        class_id = get_class_id(object_id)
        pcd_file = reorientbot.datasets.ycb.get_pcd_file(class_id=class_id)
        pcd = np.loadtxt(pcd_file)
        pcd_target = reorientbot.geometry.transform_points(
            pcd, reorientbot.geometry.transformation_matrix(*place_pose)
        )
        pcd_source = reorientbot.geometry.transform_points(
            pcd, reorientbot.geometry.transformation_matrix(*obj_to_world)
        )
        auc = reorientbot.geometry.average_distance_auc(pcd_target, pcd_source)
        if auc >= auc_threshold:
            logger.success(f"auc: {auc:.3f} >= {auc_threshold:.3f}")
            break

        while True:
            with reorientbot.pybullet.stash_objects(virtual_objects):
                _, depth, segm = ri.get_camera_image()
            for _ in ri.random_grasp(
                depth=depth,
                segm=segm,
                mask=segm == object_id,
                bg_object_ids=bg_object_ids,
                object_ids=object_ids,
                max_angle=np.deg2rad(10),
            ):
                step_simulation()
            if not ri.gripper.check_grasp():
                ri.ungrasp()
                for _ in ri.movej(j):
                    step_simulation()
                continue
            break

        with ri.enabling_attachments():
            j = ri.solve_ik(
                place_pose, move_target=ri.robot_model.attachment_link0
            )
        obstacles = bg_object_ids + object_ids
        obstacles.remove(object_id)
        path = ri.planj(j, obstacles=obstacles)
        if path is None:
            logger.warning("path is None")
            path = [j]
        for _ in (_ for j in path for _ in ri.movej(j, speed=0.001)):
            step_simulation()

        for _ in range(240):
            step_simulation()

        ri.ungrasp()

        for _ in range(240):
            step_simulation()

        c = reorientbot.geometry.Coordinate(*ri.get_pose("tipLink"))
        c.translate([0, 0, -0.05])
        j = ri.solve_ik(c.pose, rotation_axis=None)
        for _ in ri.movej(j):
            step_simulation()

        for _ in ri.movej(j_camera):
            step_simulation()

        for _ in range(240):
            step_simulation()

    for _ in ri.move_to_homej(bg_object_ids, object_ids):
        step_simulation()


def plot_time_table(time_table):
    titles = []
    for section in time_table:
        for title, _ in section:
            titles.append(title)

    rows = []
    for section in time_table:
        header = np.zeros((200, 100, 3), dtype=np.uint8)
        header[...] = 255
        header = header.transpose(1, 0, 2)
        header = imgviz.draw.text_in_rectangle(
            header,
            loc="lt",
            text=f"object_{len(rows)}",
            color=(0, 0, 0),
            size=40,
            background=(255, 255, 255),
        )
        header = header.transpose(1, 0, 2)[:, ::-1, :]
        row = [header]
        for title, seconds in section:
            width = int(round(seconds * 100))
            if width == 0:
                continue
            viz_i = np.zeros((200, width, 3), dtype=np.uint8)
            color = imgviz.label_colormap()[titles.index(title) + 1]
            viz_i[...] = color
            viz_i = viz_i.transpose(1, 0, 2)
            viz_i = imgviz.draw.text_in_rectangle(
                viz_i,
                loc="lt",
                text=f"{title}\n({seconds:.1f} [s])",
                color=(255, 255, 255),
                size=25,
                background=color,
            )
            viz_i = viz_i.transpose(1, 0, 2)
            viz_i = viz_i[:, ::-1, :]
            row.append(viz_i)
        rows.append(np.hstack(row))

    max_width = max(row.shape[1] for row in rows)
    rows = [
        imgviz.centerize(row, shape=(row.shape[0], max_width), loc="lt")
        for row in rows
    ]
    rows = imgviz.tile(rows, shape=(-1, 1), border=(0, 0, 0))

    return rows


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


def load_pile(base_pose, pkl_file, mass=None):
    with open(pkl_file, "rb") as f:
        data = pickle.load(f)
    object_ids = []
    for class_id, position, quaternion in zip(
        data["class_id"], data["position"], data["quaternion"]
    ):
        coord = reorientbot.geometry.Coordinate(
            position=position,
            quaternion=quaternion,
        )
        coord.transform(
            reorientbot.geometry.transformation_matrix(*base_pose), wrt="world"
        )

        visual_file = reorientbot.datasets.ycb.get_visual_file(class_id)
        collision_file = reorientbot.pybullet.get_collision_file(visual_file)
        mass_actual = reorientbot.datasets.ycb.masses[class_id]
        with pp.LockRenderer():
            object_id = reorientbot.pybullet.create_mesh_body(
                visual_file=visual_file,
                collision_file=collision_file,
                mass=mass_actual if mass is None else mass,
                position=coord.position,
                quaternion=coord.quaternion,
                rgba_color=imgviz.label_colormap()[class_id] / 255,
                texture=False,
            )
        object_ids.append(object_id)
    return object_ids
