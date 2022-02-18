#!/usr/bin/env python

import time

import imgviz
import numpy as np
import pybullet as p
import pybullet_planning

import reorientbot


def main():
    pybullet_planning.connect()
    pybullet_planning.add_data_path()
    p.setGravity(0, 0, -9.8)

    p.resetDebugVisualizerCamera(
        cameraDistance=1,
        cameraYaw=120,
        cameraPitch=-30,
        cameraTargetPosition=(0, 0, 0.3),
    )

    p.loadURDF("plane.urdf")

    ri = reorientbot.pybullet.PandaRobotInterface()

    class_id = 16
    visual_file = reorientbot.datasets.ycb.get_visual_file(class_id=class_id)
    collision_file = reorientbot.pybullet.get_collision_file(visual_file)
    with pybullet_planning.LockRenderer():
        obj = reorientbot.pybullet.create_mesh_body(
            visual_file=collision_file,
            collision_file=collision_file,
            position=(0, 0, 0),
            quaternion=(0, 0, 0, 1),
            mass=reorientbot.datasets.ycb.masses[class_id],
            rgba_color=imgviz.label_colormap()[class_id] / 255,
        )
    aabb_min, _ = reorientbot.pybullet.get_aabb(obj)

    spawn_aabb = [0.3, -0.3, 0], [0.5, 0.3, 0]
    pybullet_planning.draw_aabb(spawn_aabb)

    while True:
        p.resetBasePositionAndOrientation(
            obj,
            -aabb_min + np.random.uniform(*spawn_aabb),
            reorientbot.geometry.quaternion_from_euler(
                [0, 0, np.random.uniform(-np.pi, np.pi)]
            ),
        )

        c = reorientbot.geometry.Coordinate(
            *pybullet_planning.get_link_pose(ri.robot, ri.ee),
        )
        c.position = pybullet_planning.get_pose(obj)[0]
        c.position[2] = pybullet_planning.get_aabb(obj)[1][2] + 0.1

        path = ri.planj(ri.solve_ik(c.pose, rotation_axis="z"))
        for j in path:
            for _ in ri.movej(j):
                p.stepSimulation()
                ri.step_simulation()
                time.sleep(1 / 240)

        for _ in ri.grasp(min_dz=0.08, max_dz=0.12):
            p.stepSimulation()
            ri.step_simulation()
            time.sleep(1 / 240)

        c = reorientbot.geometry.Coordinate(
            *pybullet_planning.get_link_pose(ri.robot, ri.ee),
        )

        speed = np.random.choice([0.01, 0.002])

        ri.planj(ri.homej)
        for j in path:
            for _ in ri.movej(j, speed=speed):
                p.stepSimulation()
                ri.step_simulation()
                time.sleep(1 / 240)

        path = ri.planj(ri.solve_ik(c.pose))
        for j in path:
            for _ in ri.movej(j, speed=speed):
                p.stepSimulation()
                ri.step_simulation()
                time.sleep(1 / 240)

        # reorientbot.pybullet.step_and_sleep(0.5)

        ri.ungrasp()

        path = ri.planj(ri.homej)
        for j in path:
            for _ in ri.movej(j):
                p.stepSimulation()
                ri.step_simulation()
                time.sleep(1 / 240)


if __name__ == "__main__":
    main()
