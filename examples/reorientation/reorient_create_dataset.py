#!/usr/bin/env python

import numpy as np
import path

import reorientbot

from reorientbot.examples.reorientation._env import Env
from reorientbot.examples.reorientation import _reorient


here = path.Path(__file__).abspath().parent


def main():
    faces = ["front", "right", "left"]

    log_dir = here / "logs/reorient_dataset"
    log_dir.makedirs_p()

    for seed in range(0, 200):
        for face in faces:
            env = Env(
                class_ids=[2, 3, 5, 11, 12, 15],
                gui=False,
                face=face,
            )
            env.random_state = np.random.RandomState(seed)
            env.eval = True
            env.reset()

            pcd_in_obj, normals_in_obj = _reorient.get_query_ocs(env)
            indices = np.random.permutation(pcd_in_obj.shape[0])[:20]
            pcd_in_obj = pcd_in_obj[indices]
            normals_in_obj = normals_in_obj[indices]
            quaternion_in_obj = reorientbot.geometry.quaternion_from_vec2vec(
                [0, 0, -1], normals_in_obj
            )
            target_grasp_poses = np.hstack([pcd_in_obj, quaternion_in_obj])
            result = _reorient.plan_place(env, target_grasp_poses)
            pick_and_placable = "js_place" not in result

            with open(log_dir / f"{seed:08d}-{face}.txt", "w") as f:
                f.write(f"{int(pick_and_placable)}")


if __name__ == "__main__":
    main()
