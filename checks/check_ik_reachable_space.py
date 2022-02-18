#!/usr/bin/env python

import argparse
import time

import imgviz
import numpy as np
import pybullet_planning as pp

import reorientbot


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--type",
        choices=["topdown", "backfront", "leftright", "rightleft"],
        help="type",
        required=True,
    )
    args = parser.parse_args()

    pp.connect()
    pp.add_data_path()

    pp.load_pybullet("plane.urdf")
    pp.enable_gravity()

    ri = reorientbot.pybullet.PandaRobotInterface()
    homej_quaternion = ri.get_pose("tipLink")[1]

    if args.type == "topdown":
        eulers = [[0, 0, 0]]
        sphere = pp.create_sphere(0.8, color=(1, 0, 0, 0.2), collision=False)
        pp.set_pose(sphere, ([0, 0, 0.1], [0, 0, 0, 1]))
    elif args.type == "backfront":
        eulers = [[np.deg2rad(90), 0, 0]]
        sphere = pp.create_sphere(0.8, color=(1, 0, 0, 0.2), collision=False)
        pp.set_pose(sphere, ([0.3, 0, 0.3], [0, 0, 0, 1]))
    elif args.type == "leftright":
        eulers = [[0, np.deg2rad(90), 0]]
        sphere = pp.create_sphere(0.8, color=(1, 0, 0, 0.2), collision=False)
        pp.set_pose(sphere, ([0, -0.3, 0.3], [0, 0, 0, 1]))
    elif args.type == "rightleft":
        eulers = [[0, np.deg2rad(-90), 0]]
        sphere = pp.create_sphere(0.8, color=(1, 0, 0, 0.2), collision=False)
        pp.set_pose(sphere, ([0, 0.3, 0.3], [0, 0, 0, 1]))

    if 1:
        c_homej = reorientbot.geometry.Coordinate(*ri.get_pose("tipLink"))
        for euler in eulers:
            c = c_homej.copy()
            c.rotate(euler)
            j = ri.solve_ik(c.pose)
            ri.setj(j)
            time.sleep(1)

    aabb = ((-0.8, -0.8, 0), (0.8, 0.8, 1.2))
    pp.draw_aabb(aabb)

    x, y, z = np.meshgrid(
        np.linspace(aabb[0][0], aabb[1][0], num=9),
        np.linspace(aabb[0][1], aabb[1][1], num=9),
        np.linspace(aabb[0][2], aabb[1][2], num=7),
    )
    xyz = np.stack([x, y, z], axis=3)

    points = xyz.reshape(-1, 3)
    for point in np.random.permutation(points):
        c_cano = reorientbot.geometry.Coordinate(
            point, quaternion=homej_quaternion
        )

        num_solutions = 0
        for euler in eulers:
            c = c_cano.copy()
            c.rotate(euler)
            j = ri.solve_ik(c.pose, n_init=1, rotation_axis="z")
            if j is not None:
                num_solutions += 1
        num_solutions /= len(eulers)

        color = (
            imgviz.depth2rgb(
                np.array([num_solutions]).reshape(1, 1),
                min_value=0,
                max_value=1,
            )[0, 0]
            / 255
        )

        box = pp.create_box(0.05, 0.05, 0.05, color=color.tolist() + [0.8])
        pp.set_pose(box, (point, (0, 0, 0, 1)))

    while True:
        pass


if __name__ == "__main__":
    main()
