#!/usr/bin/env python

import argparse

import numpy as np
import pybullet_planning as pp

import reorientbot

from reorientbot.examples.reorientation._env import Env
from reorientbot.examples.reorientation import _reorient


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--seed", type=int, help="seed", required=True)
    parser.add_argument(
        "--face",
        choices=["front", "right", "left"],
        default="front",
        help="face",
    )
    parser.add_argument("--mp4", help="mp4")
    parser.add_argument("--nogui", action="store_true", help="no gui")
    args = parser.parse_args()

    env = Env(
        class_ids=[2, 3, 5, 11, 12, 15],
        gui=not args.nogui,
        mp4=args.mp4,
        face=args.face,
    )
    env.random_state = np.random.RandomState(args.seed)
    env.eval = True
    env.reset()

    pcd_in_obj, normals_in_obj = _reorient.get_query_ocs(env)
    indices = np.random.permutation(pcd_in_obj.shape[0])[:20]
    pcd_in_obj = pcd_in_obj[indices]
    normals_in_obj = normals_in_obj[indices]
    quaternion_in_obj = reorientbot.geometry.quaternion_from_vec2vec(
        [0, 0, -1], normals_in_obj
    )
    grasp_poses = np.hstack([pcd_in_obj, quaternion_in_obj])  # in obj

    for grasp_pose in grasp_poses:
        pp.draw_pose(np.hsplit(grasp_pose, [3]), parent=env.fg_object_id)

    reorientbot.pybullet.pause()


if __name__ == "__main__":
    main()
