#!/usr/bin/env python

import argparse
import time

import imgviz
import numpy as np
import pybullet as p
import pybullet_planning

import reorientbot


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--pause", action="store_true", help="pause")
    args = parser.parse_args()

    pybullet_planning.connect()
    pybullet_planning.add_data_path()
    p.setGravity(0, 0, -9.8)

    with pybullet_planning.LockRenderer():
        p.loadURDF("plane.urdf")

    ri = reorientbot.pybullet.PandaRobotInterface()

    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=-45,
        cameraPitch=-45,
        cameraTargetPosition=(0.5, 0, 0),
    )

    data = np.load("assets/pile_001.npz")
    class_ids = data["class_ids"]
    positions = data["positions"]
    quaternions = data["quaternions"]

    for (
        class_id,
        position,
        quaternion,
    ) in zip(class_ids, positions, quaternions):
        visual_file = reorientbot.datasets.ycb.get_visual_file(class_id=class_id)
        collision_file = reorientbot.pybullet.get_collision_file(visual_file)
        c_obj_to_world = reorientbot.geometry.Coordinate(position, quaternion)
        c_obj_to_world.translate([0.5, 0, 0], wrt="world")
        reorientbot.pybullet.create_mesh_body(
            visual_file=collision_file,
            collision_file=collision_file,
            position=c_obj_to_world.position,
            quaternion=c_obj_to_world.quaternion,
            mass=0.1,
            rgba_color=list(imgviz.label_colormap()[class_id] / 255),
        )

    if args.pause:
        print("Please press 'n' to start")
        while True:
            if ord("n") in p.getKeyboardEvents():
                break

    c_camera_to_world = reorientbot.geometry.Coordinate()
    c_camera_to_world.rotate([0, 0, np.deg2rad(-90)])
    c_camera_to_world.rotate([np.deg2rad(-180), 0, 0])
    c_camera_to_world.translate([0.5, 0, 0.7], wrt="world")

    fovy = np.deg2rad(60)
    height = 480
    width = 640
    pybullet_planning.draw_pose(c_camera_to_world.pose)
    reorientbot.pybullet.draw_camera(
        fovy, height=height, width=width, pose=c_camera_to_world.pose
    )

    # rgb, _, _ = reorientbot.pybullet.get_camera_image(
    #     c_camera_to_world.matrix, fovy=np.deg2rad(45), height=480, width=640
    # )
    # imgviz.io.pyglet_imshow(rgb)
    # imgviz.io.pyglet_run()

    np.random.seed(0)

    while True:
        c_ee_to_world = c_camera_to_world.copy()
        c_ee_to_world.translate(
            np.random.uniform([-0.2, -0.2, 0.4], [0.2, 0.2, 0.4])
        )
        j = ri.solve_ik(c_ee_to_world.pose, rotation_axis=True)
        for _ in ri.movej(j):
            p.stepSimulation()
            time.sleep(1 / 240)

        for i in ri.grasp():
            p.stepSimulation()
            time.sleep(1 / 240)
            if i > 5 * 240:  # 5s
                print("Warning: timeout while trying to grasp")
                break

        reorientbot.pybullet.step_and_sleep(1)

        if ri.gripper.check_grasp():
            c = reorientbot.geometry.Coordinate(
                *pybullet_planning.get_link_pose(ri.robot, ri.ee)
            )
            c.translate([0, 0, 0.5], wrt="world")
            j = ri.solve_ik(c.pose, rotation_axis="z")
            if j is False:
                j = ri.solve_ik(c.pose, rotation_axis=None)
            for _ in ri.movej(j):
                p.stepSimulation()
                time.sleep(1 / 240)

            reorientbot.pybullet.step_and_sleep(3)

        ri.ungrasp()

    for _ in ri.movej(ri.homej):
        p.stepSimulation()
        time.sleep(1 / 240)

    while True:
        p.stepSimulation()
        time.sleep(1 / 240)


if __name__ == "__main__":
    main()
