#!/usr/bin/env python

import numpy as np
import pybullet as p
import pybullet_planning
import trimesh

import reorientbot


def main():
    pybullet_planning.connect()
    pybullet_planning.add_data_path()

    p.loadURDF("plane.urdf")

    p.resetDebugVisualizerCamera(
        cameraDistance=0.8,
        cameraYaw=-60,
        cameraPitch=-60,
        cameraTargetPosition=(0, 0, 0),
    )

    data = np.load("assets/pile_001.npz")
    class_ids = data["class_ids"]
    positions = data["positions"]
    quaternions = data["quaternions"]

    scene = trimesh.Scene()

    with pybullet_planning.LockRenderer():
        for (
            class_id,
            position,
            quaternion,
        ) in zip(class_ids, positions, quaternions):
            visual_file = reorientbot.datasets.ycb.get_visual_file(class_id)
            collision_file = reorientbot.pybullet.get_collision_file(visual_file)
            reorientbot.pybullet.create_mesh_body(
                visual_file=visual_file,
                collision_file=collision_file,
                position=position,
                quaternion=quaternion,
            )
            cad = trimesh.load_mesh(visual_file)
            cad.visual = cad.visual.to_color()
            scene.add_geometry(
                cad,
                transform=reorientbot.geometry.transformation_matrix(
                    position, quaternion
                ),
            )

    cam_to_world = reorientbot.geometry.look_at(
        eye=[-0.3, -0.3, 1], target=[0, 0, 0]
    )
    fovy = np.deg2rad(45)
    height = 480
    width = 640
    rgb, depth, _ = reorientbot.pybullet.get_camera_image(
        cam_to_world, fovy=fovy, height=height, width=width
    )

    K = reorientbot.geometry.opengl_intrinsic_matrix(fovy, height, width)
    pcd = reorientbot.geometry.pointcloud_from_depth(
        depth, fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2]
    )
    scene.add_geometry(
        trimesh.PointCloud(
            vertices=pcd[~np.isnan(depth)], colors=rgb[~np.isnan(depth)]
        ),
        transform=cam_to_world,
    )
    scene.camera_transform = reorientbot.geometry.to_opengl_transform(cam_to_world)
    scene.show(resolution=(np.array([640, 480]) * 1.5).astype(int))


if __name__ == "__main__":
    main()
