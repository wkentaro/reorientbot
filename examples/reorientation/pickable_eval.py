#!/usr/bin/env python

import argparse
import time

import gdown
import numpy as np
import path
import pybullet_planning as pp
import torch

import reorientbot

from reorientbot.examples.reorientation._env import Env
from reorientbot.examples.reorientation import _reorient
from reorientbot.examples.reorientation import _utils
from reorientbot.examples.reorientation.pickable_reorient_poses import (
    get_reorient_poses,  # NOQA
)
from reorientbot.examples.reorientation.pickable_train import Model


here = path.Path(__file__).abspath().parent


models = {}

models["franka_panda/panda_drl"] = Model()
model_file = gdown.cached_download(
    id="1FeIHeQl_nSQgutdALEgjICrEmwe5DSk3",
    path=here
    / "logs/pickable/20210811_174549.504802-panda_drl/models/model_best-epoch_0044.pt",  # NOQA
    md5="47cf65986f5cea24dceb5eb92c5308be",
)
models["franka_panda/panda_drl"].load_state_dict(torch.load(model_file))
models["franka_panda/panda_drl"].eval()

models["franka_panda/panda_suction"] = Model()
model_file = gdown.cached_download(
    id="11Z1wTicF7i8pcE6nARiJQfkq6VW7dpJH",
    path=here
    / "logs/pickable/20210819_035214.799043-panda_suction/models/model_best-epoch_0050.pt",  # NOQA
    md5="748f277c7b50b507d943a8aa9c9e1476",
)
models["franka_panda/panda_suction"].load_state_dict(torch.load(model_file))
models["franka_panda/panda_suction"].eval()


def get_goal_oriented_reorient_poses(env):
    model = models[env._robot_model]
    model.cuda()

    class_ids = [2, 3, 5, 11, 12, 15, 16]

    heightmap = env.obs["pointmap"][:, :, 2]

    object_fg_flags = []
    object_labels = []
    object_poses = []
    for object_id in env.object_ids:
        class_id = _utils.get_class_id(object_id)
        if class_id not in class_ids:
            continue
        object_fg_flags.append(object_id == env.fg_object_id)
        object_label = np.zeros(7)
        object_label[class_ids.index(class_id)] = 1
        object_labels.append(object_label)
        object_poses.append(np.hstack(pp.get_pose(object_id)))
    object_fg_flags = np.array(object_fg_flags, dtype=bool)
    object_labels = np.array(object_labels, dtype=np.float32)
    object_poses = np.array(object_poses, dtype=np.float32)

    reorient_poses = get_reorient_poses(env)

    # target grasp_pose
    pcd_in_obj, normals_in_obj = _reorient.get_query_ocs(env)
    indices = np.random.permutation(pcd_in_obj.shape[0])[:32]

    pcd_in_obj = pcd_in_obj[indices]
    normals_in_obj = normals_in_obj[indices]
    quaternion_in_obj = reorientbot.geometry.quaternion_from_vec2vec(
        [0, 0, -1], normals_in_obj
    )
    grasp_poses = np.hstack([pcd_in_obj, quaternion_in_obj])

    # pose representation -> point-normal representation
    grasp_points = []
    for grasp_pose in grasp_poses:
        ee_to_obj = np.hsplit(grasp_pose, [3])
        grasp_point_start = ee_to_obj[0]
        grasp_point_end = reorientbot.geometry.transform_points(
            [[0, 0, 1]], reorientbot.geometry.transformation_matrix(*ee_to_obj)
        )[0]
        grasp_points.append(np.hstack([grasp_point_start, grasp_point_end]))

        if 0:
            pp.draw_pose(
                np.hsplit(grasp_pose, [3]),
                parent=env.fg_object_id,
                length=0.05,
                width=3,
            )
    grasp_points = np.array(grasp_points)

    object_label = object_labels[object_fg_flags == 1][0]
    object_pose = object_poses[object_fg_flags == 1][0]

    R = reorient_poses.shape[0]
    G = grasp_points.shape[0]

    reorient_poses = reorient_poses[:, None].repeat(G, axis=1)
    grasp_poses = grasp_poses[None].repeat(R, axis=0)
    grasp_points = grasp_points[None].repeat(R, axis=0)

    reorient_poses = reorient_poses.reshape(R * G, 7)
    grasp_poses = grasp_poses.reshape(R * G, 7)
    grasp_points = grasp_points.reshape(R * G, 6)

    with torch.no_grad():
        pickable_pred = model(
            heightmap=torch.as_tensor(heightmap[None, None]).float().cuda(),
            object_label=torch.as_tensor(object_label[None]).float().cuda(),
            object_pose=torch.as_tensor(object_pose[None]).float().cuda(),
            grasp_pose=torch.as_tensor(grasp_points[None]).float().cuda(),
            reorient_pose=torch.as_tensor(reorient_poses[None]).float().cuda(),
        )
    pickable_pred = pickable_pred.cpu().numpy()

    pickable_pred = pickable_pred.reshape(R, G).mean(axis=1)
    grasp_poses = grasp_poses.reshape(R, G, 7)
    grasp_points = grasp_points.reshape(R, G, 6)
    reorient_poses = reorient_poses.reshape(R, G, 7)

    return reorient_poses[:, 0, :], pickable_pred, grasp_poses[0, :, :]


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--seed", type=int, required=True, help="seed")
    parser.add_argument(
        "--face",
        choices=["front", "right", "left"],
        default="front",
        help="face",
    )
    parser.add_argument("--visualize", action="store_true", help="visualize")
    args = parser.parse_args()

    env = Env(class_ids=[2, 3, 5, 11, 12, 15], gui=True, face=args.face)
    env.random_state = np.random.RandomState(args.seed)
    env.eval = True
    env.reset()

    reorient_poses, pickable, grasp_poses = get_goal_oriented_reorient_poses(
        env
    )

    indices = np.where(pickable > 0.75)[0]
    if indices.size == 0:
        indices = np.where(pickable > 0.5)[0]
    indices = np.random.choice(indices, 1000)
    reorient_poses = reorient_poses[indices]

    if args.visualize:
        for reorient_pose in reorient_poses:
            reorientbot.pybullet.duplicate(
                env.fg_object_id,
                position=reorient_pose[:3],
                quaternion=reorient_pose[3:],
                collision=False,
            )
        return

    for reorient_pose in reorient_poses:
        env.pi.setj(env.pi.homej)

        obj_af_to_world = np.hsplit(reorient_pose, [3])
        pp.set_pose(env.fg_object_id, obj_af_to_world)

        time.sleep(0.5)

        for _ in range(480):
            pp.step_simulation()

        if 0:
            for ee_af_to_obj in grasp_poses:
                ee_af_to_obj = np.hsplit(ee_af_to_obj, [3])
                ee_af_to_world = pp.multiply(
                    pp.get_pose(env.fg_object_id), ee_af_to_obj
                )
                j = env.pi.solve_ik(
                    ee_af_to_world, rotation_axis="z", rthre=np.deg2rad(10)
                )
                if j is not None:
                    obstacles = [env.plane] + env.object_ids
                    obstacles.remove(env.fg_object_id)
                    if not env.pi.validatej(j, obstacles=obstacles):
                        j = None
                if j is not None:
                    env.pi.setj(j)
                    break
            else:
                print("Failed to solve IK")


if __name__ == "__main__":
    main()
