#!/usr/bin/env python

import argparse
import pickle
import time

from loguru import logger
import numpy as np
import path
import pybullet_planning as pp
import trimesh

import reorientbot

from reorientbot.examples.reorientation._env import Env
from reorientbot.examples.reorientation import _utils
from reorientbot.examples.reorientation.pickable_reorient_poses import (
    get_reorient_poses,  # NOQA
)


home = path.Path("~").expanduser()


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    choices = ["franka_panda/panda_suction", "franka_panda/panda_drl"]
    parser.add_argument(
        "--robot-model",
        default=choices[0],
        choices=choices,
        help=" ",
    )
    parser.add_argument("--seed", type=int, required=True, help=" ")
    parser.add_argument("--gui", action="store_true", help=" ")
    args = parser.parse_args()

    root_dir = (
        home / f".cache/reorientbot/reorientation/pickable/{args.robot_model}"
    )

    if (root_dir / f"s-{args.seed:08d}/00000099.pkl").exists():
        return

    env = Env(
        class_ids=[2, 3, 5, 11, 12, 15],
        gui=args.gui,
        robot_model=args.robot_model,
    )
    env.random_state = np.random.RandomState(args.seed)
    env.launch()

    with pp.LockRenderer():
        env.reset()
        for obj in reorientbot.pybullet.get_body_unique_ids():
            if obj in [env.plane, env.pi.robot] + env.object_ids:
                continue
            pp.remove_body(obj)

    object_fg_flags = []
    object_classes = []
    object_poses = []
    for object_id in env.object_ids:
        object_fg_flags.append(object_id == env.fg_object_id)
        object_classes.append(_utils.get_class_id(object_id))
        object_poses.append(np.hstack(pp.get_pose(object_id)))
    object_fg_flags = np.array(object_fg_flags, dtype=bool)
    object_classes = np.array(object_classes, dtype=np.int32)
    object_poses = np.array(object_poses, dtype=np.float32)

    reorient_poses = get_reorient_poses(env)

    class_id = _utils.get_class_id(env.fg_object_id)
    visual_file = reorientbot.datasets.ycb.get_visual_file(class_id=class_id)
    mesh = trimesh.load_mesh(visual_file)
    pcd_in_obj = mesh.vertices
    normals_in_obj = mesh.vertex_normals

    n_saved = 0
    for reorient_pose in reorient_poses[
        np.random.permutation(reorient_poses.shape[0])
    ]:
        world_saver = pp.WorldSaver()

        pp.set_pose(env.fg_object_id, (reorient_pose[:3], reorient_pose[3:]))

        for _ in range(480):
            pp.step_simulation()

        obj_to_world = pp.get_pose(env.fg_object_id)
        T_obj_to_world = reorientbot.geometry.transformation_matrix(
            *obj_to_world
        )

        pcd_in_world = reorientbot.geometry.transform_points(
            pcd_in_obj, T_obj_to_world
        )
        normals_in_world = (
            reorientbot.geometry.transform_points(
                pcd_in_obj + normals_in_obj, T_obj_to_world
            )
            - pcd_in_world
        )

        quaternions = reorientbot.geometry.quaternion_from_vec2vec(
            [0, 0, -1], normals_in_world
        )

        indices = np.random.permutation(quaternions.shape[0])
        for index in indices[:10]:
            ee_af_to_world = pcd_in_world[index], quaternions[index]

            c = reorientbot.geometry.Coordinate(*ee_af_to_world)
            c.translate([0, 0, -0.1])

            pickable = True

            j = env.pi.solve_ik(c.pose, rotation_axis="z")
            if j is not None:
                env.pi.setj(j)
                if args.gui:
                    time.sleep(0.1)

                obstacles = env.bg_objects + env.object_ids
                obstacles.remove(env.fg_object_id)
                if not env.pi.validatej(
                    j,
                    obstacles=obstacles,
                    min_distances={(env.fg_object_id, -1): -0.01},
                ):
                    j = None
            pickable &= j is not None

            for _ in range(5):
                if not pickable:
                    break
                c.translate([0, 0, 0.02])
                j = env.pi.solve_ik(c.pose, rotation_axis=True)
                if j is not None:
                    env.pi.setj(j)
                    if args.gui:
                        time.sleep(0.1)

                    obstacles = env.bg_objects + env.object_ids
                    obstacles.remove(env.fg_object_id)
                    if not env.pi.validatej(j, obstacles=obstacles):
                        j = None
                pickable &= j is not None

            ee_af_to_obj = pp.multiply(pp.invert(obj_to_world), ee_af_to_world)

            if args.gui:
                continue

            data = dict(
                pointmap=env.obs["pointmap"],
                maskmap=env.obs["segmmap"] == env.fg_object_id,
                object_fg_flags=object_fg_flags,
                object_classes=object_classes,
                object_poses=object_poses,
                reorient_pose=reorient_pose,
                final_pose=np.hstack(obj_to_world),
                grasp_pose_wrt_obj=np.hstack(ee_af_to_obj),
                pickable=pickable,
            )

            while True:
                pkl_file = root_dir / f"s-{args.seed:08d}/{n_saved:08d}.pkl"
                if not pkl_file.exists():
                    break
                n_saved += 1

            if n_saved > 99:
                return

            pkl_file.parent.makedirs_p()
            with open(pkl_file, "wb") as f:
                pickle.dump(data, f)
            logger.info(f"Saved to: {pkl_file}")

            n_saved += 1

        world_saver.restore()


if __name__ == "__main__":
    main()
